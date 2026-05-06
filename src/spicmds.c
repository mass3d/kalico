// Commands for sending messages on an SPI bus
//
// Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_WANT_SOFTWARE_SPI
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "spi_software.h" // spi_software_setup
#include "spicmds.h" // spidev_transfer

struct spidev_s {
    union {
        struct spi_config spi_config;
        struct spi_software *spi_software;
    };
    struct gpio_out pin;
    uint8_t flags;
};

enum {
    SF_HAVE_PIN = 1, SF_SOFTWARE = 2, SF_HARDWARE = 4, SF_CS_ACTIVE_HIGH = 8
};

#if !CONFIG_WANT_GPIO_SPI
// These are declared here to avoid a bunch of ifdefs below,
// if software SPI is enabled but not hardware SPI.
void
spi_transfer(struct spi_config spi, uint8_t receive_data,
             uint8_t data_len, uint8_t *data) {}
void
spi_prepare(struct spi_config spi) {}
#endif

void
command_config_spi(uint32_t *args)
{
    struct spidev_s *spi = oid_alloc(args[0], command_config_spi, sizeof(*spi));
    uint_fast8_t cs_active_high = args[2];
    spi->pin = gpio_out_setup(args[1], !cs_active_high);
    spi->flags |= SF_HAVE_PIN | (cs_active_high ? SF_CS_ACTIVE_HIGH : 0);
}
DECL_COMMAND(command_config_spi, "config_spi oid=%c pin=%u cs_active_high=%c");

void
command_config_spi_without_cs(uint32_t *args)
{
    struct spidev_s *spi = oid_alloc(args[0], command_config_spi, sizeof(*spi));
}
DECL_COMMAND(command_config_spi_without_cs, "config_spi_without_cs oid=%c");

struct spidev_s *
spidev_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_spi);
}

void
command_spi_set_bus(uint32_t *args)
{
#if CONFIG_WANT_GPIO_SPI
    struct spidev_s *spi = spidev_oid_lookup(args[0]);
    uint8_t mode = args[2];
    if (mode > 3 || spi->flags & (SF_SOFTWARE|SF_HARDWARE))
        shutdown("Invalid spi config");
    spi->spi_config = spi_setup(args[1], mode, args[3]);
    spi->flags |= SF_HARDWARE;
#else
    shutdown("hardware SPI is not supported/enabled");
#endif
}
DECL_COMMAND(command_spi_set_bus,
             "spi_set_bus oid=%c spi_bus=%u mode=%u rate=%u");

void
spidev_set_software_bus(struct spidev_s *spi, struct spi_software *ss)
{
#if CONFIG_WANT_SOFTWARE_SPI
    if (spi->flags & (SF_SOFTWARE|SF_HARDWARE))
        shutdown("Invalid spi config");
    spi->spi_software = ss;
    spi->flags |= SF_SOFTWARE;
#else
    shutdown("software SPI is not supported/enabled");
#endif
}

int
spidev_have_cs_pin(struct spidev_s *spi)
{
    return spi->flags & SF_HAVE_PIN;
}

struct gpio_out
spidev_get_cs_pin(struct spidev_s *spi)
{
    return spi->pin;
}

struct spi_config
spidev_get_spi_config(struct spidev_s *spi)
{
    return spi->spi_config;
}

uint8_t
spidev_is_cs_active_high(struct spidev_s *spi)
{
    return !!(spi->flags & SF_CS_ACTIVE_HIGH);
}

// SPI bus busy flag for ISR contention avoidance
static volatile uint8_t spi_bus_busy;
// SPI bus hold flag — keeps ISR from writing between multi-transaction sequences
static volatile uint8_t spi_bus_hold;

uint8_t
spidev_is_bus_busy(void)
{
    return spi_bus_busy || spi_bus_hold;
}

void
spidev_set_bus_busy(uint8_t busy)
{
    spi_bus_busy = busy;
}

// Prepare SPI bus for a series of ISR transfers.
// Caller must check spidev_is_bus_busy() first.
void
spidev_prepare_bus(struct spidev_s *spi)
{
    spi_prepare(spi->spi_config);
}

// Transfer data with CS handling, skipping spi_prepare and bus_busy.
// Caller must call spidev_prepare_bus() first and manage bus_busy.
void
spidev_transfer_prepared(struct spidev_s *spi, uint8_t data_len,
                         uint8_t *data)
{
    uint_fast8_t flags = spi->flags;
    if (flags & SF_HAVE_PIN)
        gpio_out_write(spi->pin, !!(flags & SF_CS_ACTIVE_HIGH));
    spi_transfer(spi->spi_config, 0, data_len, data);
    if (flags & SF_HAVE_PIN)
        gpio_out_write(spi->pin, !(flags & SF_CS_ACTIVE_HIGH));
}

// =====================================================================
// DMA fire-and-forget transfer.  Caller manages CS via the cb (called
// from the DMA-TC IRQ).  See spicmds.h for full semantics.
// =====================================================================
#if CONFIG_WANT_SPI_DMA && CONFIG_PHASE_STEPPER_EXPERIMENTAL_DMA
// Board-level primitive — defined in stm32h7_spi.c.  Declared locally
// with `void *` to avoid pulling stm32h7xx.h into this file.
extern int spi_dma_kick_tx(void *spi, uint8_t *tx_buf, uint16_t len,
                           spi_dma_done_fn cb, void *ctx);
extern uint8_t spi_dma_is_inflight(void *spi);

// At most one DMA can be in flight at a time (gated by spi_bus_busy).
// We wrap the user callback so we own CS and bus_busy clearing — the
// user cb only needs to know "transfer complete".
static struct {
    struct spidev_s *spi;
    spi_dma_done_fn user_cb;
    void *user_ctx;
} active_dma_wrap;

static void
dma_wrap_done(void *ctx)
{
    (void)ctx;
    struct spidev_s *spi = active_dma_wrap.spi;
    spi_dma_done_fn user_cb = active_dma_wrap.user_cb;
    void *user_ctx = active_dma_wrap.user_ctx;
    active_dma_wrap.spi = NULL;
    active_dma_wrap.user_cb = NULL;
    active_dma_wrap.user_ctx = NULL;
    if (spi && (spi->flags & SF_HAVE_PIN))
        gpio_out_write(spi->pin, !(spi->flags & SF_CS_ACTIVE_HIGH));
    spi_bus_busy = 0;
    if (user_cb)
        user_cb(user_ctx);
}

int
spidev_kick_dma_tx(struct spidev_s *spi, uint8_t *tx_buf, uint16_t len,
                   spi_dma_done_fn cb, void *ctx)
{
    uint_fast8_t flags = spi->flags;
    if (!(flags & SF_HARDWARE))
        return -1;  // DMA only available on hardware SPI buses
    if (spi_bus_busy || spi_bus_hold)
        return -1;
    spi_bus_busy = 1;
    active_dma_wrap.spi = spi;
    active_dma_wrap.user_cb = cb;
    active_dma_wrap.user_ctx = ctx;
    spi_prepare(spi->spi_config);
    if (flags & SF_HAVE_PIN)
        gpio_out_write(spi->pin, !!(flags & SF_CS_ACTIVE_HIGH));
    int rc = spi_dma_kick_tx(spi->spi_config.spi, tx_buf, len,
                             dma_wrap_done, NULL);
    if (rc != 0) {
        if (flags & SF_HAVE_PIN)
            gpio_out_write(spi->pin, !(flags & SF_CS_ACTIVE_HIGH));
        active_dma_wrap.spi = NULL;
        active_dma_wrap.user_cb = NULL;
        active_dma_wrap.user_ctx = NULL;
        spi_bus_busy = 0;
    }
    return rc;
}

uint8_t
spidev_dma_in_flight(struct spidev_s *spi)
{
    return spi_dma_is_inflight(spi->spi_config.spi);
}
#else
// Polled fallback: use the exact same code path as command_spi_send and
// normal Klipper TMC register writes.  This is intentionally the default
// until the H7 DMA chain is proven on real hardware; the field symptom for
// a wedged DMA chain is event_count advancing while XDIRECT readback stays
// unchanged.
int
spidev_kick_dma_tx(struct spidev_s *spi, uint8_t *tx_buf, uint16_t len,
                   spi_dma_done_fn cb, void *ctx)
{
    if (spi_bus_busy || spi_bus_hold)
        return -1;
    spidev_transfer(spi, 0, (uint8_t)len, tx_buf);
    if (cb)
        cb(ctx);
    return 0;
}

uint8_t
spidev_dma_in_flight(struct spidev_s *spi)
{
    (void)spi;
    return 0;
}
#endif

void
spidev_transfer(struct spidev_s *spi, uint8_t receive_data
                , uint8_t data_len, uint8_t *data)
{
    uint_fast8_t flags = spi->flags;
    if (!(flags & (SF_SOFTWARE|SF_HARDWARE)))
        // Not yet initialized
        return;

    spi_bus_busy = 1;

    if (CONFIG_WANT_SOFTWARE_SPI && flags & SF_SOFTWARE)
        spi_software_prepare(spi->spi_software);
    else
        spi_prepare(spi->spi_config);

    if (flags & SF_HAVE_PIN)
        gpio_out_write(spi->pin, !!(flags & SF_CS_ACTIVE_HIGH));

    if (CONFIG_WANT_SOFTWARE_SPI && flags & SF_SOFTWARE)
        spi_software_transfer(spi->spi_software, receive_data, data_len, data);
    else
        spi_transfer(spi->spi_config, receive_data, data_len, data);

    if (flags & SF_HAVE_PIN)
        gpio_out_write(spi->pin, !(flags & SF_CS_ACTIVE_HIGH));

    spi_bus_busy = 0;
}

void
command_spi_transfer(uint32_t *args)
{
    uint8_t oid = args[0];
    struct spidev_s *spi = spidev_oid_lookup(oid);
    uint8_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    spidev_transfer(spi, 1, data_len, data);
    spi_bus_hold = 0;  // Auto-release hold after read completes
    sendf("spi_transfer_response oid=%c response=%*s", oid, data_len, data);
}
DECL_COMMAND(command_spi_transfer, "spi_transfer oid=%c data=%*s");

void
command_spi_send(uint32_t *args)
{
    struct spidev_s *spi = spidev_oid_lookup(args[0]);
    uint8_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    spidev_transfer(spi, 0, data_len, data);
}
DECL_COMMAND(command_spi_send, "spi_send oid=%c data=%*s");

void
command_spi_send_hold(uint32_t *args)
{
    struct spidev_s *spi = spidev_oid_lookup(args[0]);
    uint8_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    // Set hold before the preface transfer. Otherwise the phase ISR can run
    // after spidev_transfer() clears spi_bus_busy but before hold is asserted.
    irq_disable();
    spi_bus_hold = 1;
    irq_enable();
    spidev_transfer(spi, 0, data_len, data);
}
DECL_COMMAND(command_spi_send_hold, "spi_send_hold oid=%c data=%*s");


/****************************************************************
 * Shutdown handling
 ****************************************************************/

struct spidev_shutdown_s {
    struct spidev_s *spi;
    uint8_t shutdown_msg_len;
    uint8_t shutdown_msg[];
};

void
command_config_spi_shutdown(uint32_t *args)
{
    struct spidev_s *spi = spidev_oid_lookup(args[1]);
    uint8_t shutdown_msg_len = args[2];
    struct spidev_shutdown_s *sd = oid_alloc(
        args[0], command_config_spi_shutdown, sizeof(*sd) + shutdown_msg_len);
    sd->spi = spi;
    sd->shutdown_msg_len = shutdown_msg_len;
    uint8_t *shutdown_msg = command_decode_ptr(args[3]);
    memcpy(sd->shutdown_msg, shutdown_msg, shutdown_msg_len);
}
DECL_COMMAND(command_config_spi_shutdown,
             "config_spi_shutdown oid=%c spi_oid=%c shutdown_msg=%*s");

void
spidev_shutdown(void)
{
    spi_bus_hold = 0;
    // Cancel any transmissions that may be in progress
    uint8_t oid;
    struct spidev_s *spi;
    foreach_oid(oid, spi, command_config_spi) {
        if (spi->flags & SF_HAVE_PIN)
            gpio_out_write(spi->pin, !(spi->flags & SF_CS_ACTIVE_HIGH));
    }

    // Send shutdown messages
    struct spidev_shutdown_s *sd;
    foreach_oid(oid, sd, command_config_spi_shutdown) {
        spidev_transfer(sd->spi, 0, sd->shutdown_msg_len, sd->shutdown_msg);
    }
}
DECL_SHUTDOWN(spidev_shutdown);
