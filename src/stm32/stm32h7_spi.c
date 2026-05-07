// SPI functions on STM32H7
//
// Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_WANT_SPI_DMA
#include "board/io.h" // readb, writeb
#include "command.h" // shutdown
#include "generic/armcm_boot.h" // armcm_enable_irq
#include "gpio.h" // spi_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown
#include "board/misc.h" // timer_is_before

struct spi_info {
    SPI_TypeDef *spi;
    uint8_t miso_pin, mosi_pin, sck_pin, function;
};

DECL_ENUMERATION("spi_bus", "spi2", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2", "PB14,PB15,PB13");

DECL_ENUMERATION("spi_bus", "spi1", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi1", "PA6,PA7,PA5");
DECL_ENUMERATION("spi_bus", "spi1a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi1a", "PB4,PB5,PB3");

#if !CONFIG_MACH_STM32F1
DECL_ENUMERATION("spi_bus", "spi2a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2a", "PC2,PC3,PB10");
#endif

#ifdef SPI3
DECL_ENUMERATION("spi_bus", "spi3a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi3a", "PC11,PC12,PC10");
#endif

#ifdef SPI4
DECL_ENUMERATION("spi_bus", "spi4", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi4", "PE13,PE14,PE12");
#endif

#ifdef GPIOI
DECL_ENUMERATION("spi_bus", "spi2b", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2b", "PI2,PI3,PI1");
#endif

#ifdef SPI5
DECL_ENUMERATION("spi_bus", "spi5", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi5", "PF8,PF9,PF7");
DECL_ENUMERATION("spi_bus", "spi5a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi5a", "PH7,PF11,PH6");
#endif

#ifdef SPI6
DECL_ENUMERATION("spi_bus", "spi6", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi6", "PG12,PG14,PG13");
#endif


static const struct spi_info spi_bus[] = {
    { SPI2, GPIO('B', 14), GPIO('B', 15), GPIO('B', 13), GPIO_FUNCTION(5) },
    { SPI1, GPIO('A', 6), GPIO('A', 7), GPIO('A', 5), GPIO_FUNCTION(5) },
    { SPI1, GPIO('B', 4), GPIO('B', 5), GPIO('B', 3), GPIO_FUNCTION(5) },
#if !CONFIG_MACH_STM32F1
    { SPI2, GPIO('C', 2), GPIO('C', 3), GPIO('B', 10), GPIO_FUNCTION(5) },
#endif
#ifdef SPI3
    { SPI3, GPIO('C', 11), GPIO('C', 12), GPIO('C', 10), GPIO_FUNCTION(6) },
#endif
#ifdef SPI4
    { SPI4, GPIO('E', 13), GPIO('E', 14), GPIO('E', 12), GPIO_FUNCTION(5) },
#endif
    { SPI2, GPIO('I', 2), GPIO('I', 3), GPIO('I', 1), GPIO_FUNCTION(5) },
#ifdef SPI5
    { SPI5, GPIO('F', 8), GPIO('F', 9), GPIO('F', 7), GPIO_FUNCTION(5) },
    { SPI5, GPIO('H', 7), GPIO('F', 11), GPIO('H', 6), GPIO_FUNCTION(5) },
#endif
#ifdef SPI6
    { SPI6, GPIO('G', 12), GPIO('G', 14), GPIO('G', 13), GPIO_FUNCTION(5)},
#endif
};

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    if (bus >= ARRAY_SIZE(spi_bus))
        shutdown("Invalid spi bus");

    // Enable SPI
    SPI_TypeDef *spi = spi_bus[bus].spi;
    if (!is_enabled_pclock((uint32_t)spi)) {
        enable_pclock((uint32_t)spi);
        gpio_peripheral(spi_bus[bus].miso_pin, spi_bus[bus].function, 1);
        gpio_peripheral(spi_bus[bus].mosi_pin, spi_bus[bus].function, 0);
        gpio_peripheral(spi_bus[bus].sck_pin, spi_bus[bus].function, 0);
    }

    // Calculate CR1 register
    uint32_t pclk = get_pclock_frequency((uint32_t)spi);
    uint32_t div = 0;
    while ((pclk >> (div + 1)) > rate && div < 7)
        div++;

    return (struct spi_config){ .spi = spi, .div = div, .mode = mode };
}

void
spi_prepare(struct spi_config config)
{
    uint32_t div = config.div;
    uint32_t mode = config.mode;
    SPI_TypeDef *spi = config.spi;

    // Load frequency
    spi->CFG1 = (div << SPI_CFG1_MBR_Pos) | (7 << SPI_CFG1_DSIZE_Pos);
    // Load mode
    uint32_t cfg2 = ((mode << SPI_CFG2_CPHA_Pos) | SPI_CFG2_MASTER
                     | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | SPI_CFG2_SSOE);
    uint32_t diff = spi->CFG2 ^ cfg2;
    spi->CFG2 = cfg2;
    uint32_t end = timer_read_time() + timer_from_us(1);
    if (diff & SPI_CFG2_CPOL_Msk)
        while (timer_is_before(timer_read_time(), end))
            ;
}

void
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    uint8_t rdata = 0;
    SPI_TypeDef *spi = config.spi;

    spi->CR2 = len << SPI_CR2_TSIZE_Pos;
    // Enable SPI and start transfer, these MUST be set in this sequence
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    while (len--) {
        writeb((void *)&spi->TXDR, *data);
        while ((spi->SR & (SPI_SR_RXWNE | SPI_SR_RXPLVL)) == 0)
            ;
        rdata = readb((void *)&spi->RXDR);

        if (receive_data) {
            *data = rdata;
        }
        data++;
    }

    while ((spi->SR & SPI_SR_EOT) == 0)
        ;

    // Clear flags and disable SPI
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
}

#if CONFIG_WANT_SPI_DMA && CONFIG_PHASE_STEPPER_EXPERIMENTAL_DMA
// =====================================================================
// DMA-driven fire-and-forget SPI TX path.
// Used by tmc_phase_stepper.c group ISR.  Allocates one DMA1 stream per
// SPI peripheral (TX only — TMC XDIRECT writes are write-only at the ISR
// level).  Caller is responsible for chip-select handling: pull CS low
// before kick_tx, raise CS in the dma_done callback (called from the
// DMA-TC IRQ at NVIC priority 1 — same level as USB/CAN, one tier
// above SysTick).
//
// DMAMUX request line numbers from RM0468 (STM32H723):
//   SPI1_TX = 38, SPI2_TX = 40, SPI3_TX = 62,
//   SPI4_TX = 84, SPI5_TX = 86.  SPI6 uses BDMA (different controller)
//   and is not handled here.
// =====================================================================

#include "spicmds.h" // spi_dma_done_fn

#define MAX_SPI_DMA_BUSES 5  // SPI1..SPI5

struct spi_dma_state {
    SPI_TypeDef *spi;            // identifies the bus this state belongs to
    DMA_Stream_TypeDef *stream;
    DMAMUX_Channel_TypeDef *mux_chan;
    uint8_t mux_request;
    volatile uint8_t inflight;
    spi_dma_done_fn cb;
    void *ctx;
};

static struct spi_dma_state spi_dma_states[MAX_SPI_DMA_BUSES];
static uint8_t spi_dma_state_count;

// Find or allocate a DMA state slot for this SPI peripheral.
static struct spi_dma_state *
spi_dma_state_for(SPI_TypeDef *spi)
{
    for (uint8_t i = 0; i < spi_dma_state_count; i++)
        if (spi_dma_states[i].spi == spi)
            return &spi_dma_states[i];
    return NULL;
}

// Map an SPI peripheral to (stream, dmamux channel, request line).
// Returns 1 on success, 0 if the SPI isn't supported here.
static int
spi_dma_default_assignment(SPI_TypeDef *spi, DMA_Stream_TypeDef **out_stream,
                           DMAMUX_Channel_TypeDef **out_mux,
                           uint8_t *out_request, IRQn_Type *out_irq)
{
    if (spi == SPI1) {
        *out_stream = DMA1_Stream0; *out_mux = DMAMUX1_Channel0;
        *out_request = 38; *out_irq = DMA1_Stream0_IRQn; return 1;
    }
    if (spi == SPI2) {
        *out_stream = DMA1_Stream1; *out_mux = DMAMUX1_Channel1;
        *out_request = 40; *out_irq = DMA1_Stream1_IRQn; return 1;
    }
#ifdef SPI3
    if (spi == SPI3) {
        *out_stream = DMA1_Stream2; *out_mux = DMAMUX1_Channel2;
        *out_request = 62; *out_irq = DMA1_Stream2_IRQn; return 1;
    }
#endif
#ifdef SPI4
    if (spi == SPI4) {
        *out_stream = DMA1_Stream3; *out_mux = DMAMUX1_Channel3;
        *out_request = 84; *out_irq = DMA1_Stream3_IRQn; return 1;
    }
#endif
#ifdef SPI5
    if (spi == SPI5) {
        *out_stream = DMA1_Stream4; *out_mux = DMAMUX1_Channel4;
        *out_request = 86; *out_irq = DMA1_Stream4_IRQn; return 1;
    }
#endif
    return 0;
}

// Counts of successful TC and error TE events — exposed via
// spi_dma_get_te_count() so the host can detect transient DMA errors that
// would otherwise be invisible.  TE fires when the AHB bus reports a fault,
// the peripheral signals an error to the DMA, or in direct mode if the
// peripheral isn't ready.  Either way we run the same cleanup as TC so the
// bus unwedges; the counter just makes the event observable.
static volatile uint32_t spi_dma_te_total;

uint32_t
spi_dma_get_te_count(void)
{
    return spi_dma_te_total;
}

// Per-stream IRQ handler shared body.  Called from the per-stream
// IRQHandler thunks below.  Cleans up DMA + SPI peripherals and invokes
// the user-supplied completion callback.  Handles BOTH transfer-complete
// (TC) and transfer-error (TE) — the cleanup path is identical, but on TE
// we increment a counter and skip the EOT wait (the SPI peripheral may be
// in an error state and never assert EOT).
static void
spi_dma_handle_tc(struct spi_dma_state *st, volatile uint32_t *isr_reg,
                  volatile uint32_t *clear_reg,
                  uint32_t teif_mask, uint32_t tcif_mask)
{
    uint32_t isr = *isr_reg;
    uint8_t te_fired = (isr & teif_mask) ? 1 : 0;
    if (te_fired)
        spi_dma_te_total++;
    *clear_reg = teif_mask | tcif_mask;
    st->stream->CR &= ~DMA_SxCR_EN;
    SPI_TypeDef *spi = st->spi;
    // Wait for trailing SCLK edge.  DMA TC fires when DMA finishes
    // PUSHING bytes to the TX FIFO — but the SPI peripheral is still
    // shifting those bytes out at the SCK rate.  EOT only asserts after
    // the last bit is fully shifted.  At 4 MHz, 5 bytes take 10us to
    // shift out; the previous 2us timeout was not nearly enough, so
    // SPE got cleared mid-transfer — clearing SPE truncates the burst at
    // the next byte boundary, and the TMC saw a short/garbled write and
    // ignored it.  100us covers 5-byte bursts down to 400 kHz SPI; well
    // under any realistic interval at any update rate.
    if (!te_fired) {
        uint32_t deadline = timer_read_time() + timer_from_us(100);
        while ((spi->SR & SPI_SR_EOT) == 0
               && timer_is_before(timer_read_time(), deadline))
            ;
    }
    spi->IFCR = 0xFFFFFFFF;
    spi->CFG1 &= ~SPI_CFG1_TXDMAEN;
    spi->CR1 = SPI_CR1_SSI;
    spi_dma_done_fn cb = st->cb;
    void *ctx = st->ctx;
    st->cb = NULL;
    st->ctx = NULL;
    st->inflight = 0;
    if (cb)
        cb(ctx);
}

// One IRQ handler per assignable stream.  Each looks up its state by
// matching the stream pointer in the dma_states[] table.  The compiler
// folds these to small thunks.
static struct spi_dma_state *
spi_dma_state_for_stream(DMA_Stream_TypeDef *stream)
{
    for (uint8_t i = 0; i < spi_dma_state_count; i++)
        if (spi_dma_states[i].stream == stream)
            return &spi_dma_states[i];
    return NULL;
}

void
DMA1_Stream0_IRQHandler(void)
{
    struct spi_dma_state *st = spi_dma_state_for_stream(DMA1_Stream0);
    if (st)
        spi_dma_handle_tc(st, &DMA1->LISR, &DMA1->LIFCR,
                          DMA_LIFCR_CTEIF0, DMA_LIFCR_CTCIF0);
}
void
DMA1_Stream1_IRQHandler(void)
{
    struct spi_dma_state *st = spi_dma_state_for_stream(DMA1_Stream1);
    if (st)
        spi_dma_handle_tc(st, &DMA1->LISR, &DMA1->LIFCR,
                          DMA_LIFCR_CTEIF1, DMA_LIFCR_CTCIF1);
}
void
DMA1_Stream2_IRQHandler(void)
{
    struct spi_dma_state *st = spi_dma_state_for_stream(DMA1_Stream2);
    if (st)
        spi_dma_handle_tc(st, &DMA1->LISR, &DMA1->LIFCR,
                          DMA_LIFCR_CTEIF2, DMA_LIFCR_CTCIF2);
}
void
DMA1_Stream3_IRQHandler(void)
{
    struct spi_dma_state *st = spi_dma_state_for_stream(DMA1_Stream3);
    if (st)
        spi_dma_handle_tc(st, &DMA1->LISR, &DMA1->LIFCR,
                          DMA_LIFCR_CTEIF3, DMA_LIFCR_CTCIF3);
}
void
DMA1_Stream4_IRQHandler(void)
{
    struct spi_dma_state *st = spi_dma_state_for_stream(DMA1_Stream4);
    if (st)
        spi_dma_handle_tc(st, &DMA1->HISR, &DMA1->HIFCR,
                          DMA_HIFCR_CTEIF4, DMA_HIFCR_CTCIF4);
}

// Register the IRQ handlers in the vector table.  buildcommands.py picks
// these up at compile time even though no init function calls them.
DECL_ARMCM_IRQ(DMA1_Stream0_IRQHandler, DMA1_Stream0_IRQn);
DECL_ARMCM_IRQ(DMA1_Stream1_IRQHandler, DMA1_Stream1_IRQn);
DECL_ARMCM_IRQ(DMA1_Stream2_IRQHandler, DMA1_Stream2_IRQn);
DECL_ARMCM_IRQ(DMA1_Stream3_IRQHandler, DMA1_Stream3_IRQn);
DECL_ARMCM_IRQ(DMA1_Stream4_IRQHandler, DMA1_Stream4_IRQn);

// Lazy per-bus DMA setup.  Called from spi_dma_kick_tx the first time we
// see a particular SPI peripheral.  Allocates an entry in spi_dma_states[]
// and configures the DMA stream + DMAMUX channel.
static struct spi_dma_state *
spi_dma_init_bus(SPI_TypeDef *spi)
{
    if (spi_dma_state_count >= MAX_SPI_DMA_BUSES)
        return NULL;
    DMA_Stream_TypeDef *stream;
    DMAMUX_Channel_TypeDef *mux;
    uint8_t request;
    IRQn_Type irq;
    if (!spi_dma_default_assignment(spi, &stream, &mux, &request, &irq))
        return NULL;
    // Enable DMA1 + DMAMUX1 clocks.  RCC layout: AHB1ENR.DMA1EN bit 0;
    // DMAMUX1 sits on the same clock domain as DMA1.
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR; // post-write read for clock enable propagation

    struct spi_dma_state *st = &spi_dma_states[spi_dma_state_count++];
    st->spi = spi;
    st->stream = stream;
    st->mux_chan = mux;
    st->mux_request = request;
    st->inflight = 0;
    st->cb = NULL;
    st->ctx = NULL;

    // DMAMUX channel: select the SPIx_TX request line.
    mux->CCR = request;
    // DMA stream: ensure disabled, then program for mem->peripheral.
    stream->CR = 0;
    while (stream->CR & DMA_SxCR_EN) ;
    stream->FCR = 0;  // direct mode (no FIFO)
    stream->PAR = (uint32_t)&spi->TXDR;
    stream->CR = (DMA_SxCR_DIR_0       // memory-to-peripheral
                  | DMA_SxCR_MINC      // memory increment
                  | DMA_SxCR_PL_1      // priority high
                  | DMA_SxCR_TCIE      // transfer-complete interrupt enable
                  | DMA_SxCR_TEIE);    // transfer-error interrupt enable
    // Without TEIE the bus wedges permanently on a single transfer error:
    // TE doesn't fire an IRQ, the TC handler never runs, dma_wrap_done
    // never clears spi_bus_busy, and every subsequent kick returns -1.
    // Field-observed after ~3 million successful transfers (78s at 10kHz
    // with 4 motors) — one transient TE was enough to kill phase stepping
    // until reboot.
    // PSIZE / MSIZE = byte (00) — bits left at zero.

    // Install the per-stream IRQ at priority 1 (matches USB/CAN convention,
    // one tier above SysTick=2 so DMA TC always fires promptly after the
    // group ISR has kicked).
    NVIC_SetPriority(irq, 1);
    NVIC_EnableIRQ(irq);
    return st;
}

// Public: kick a fire-and-forget DMA TX on `spi`.  Returns 0 on accept,
// -1 if a transfer is already in flight on this bus.  ISR-callable.
// `tx_buf` MUST be in DMA-coherent memory (use the .dma_buf section
// attribute on H7 — DTCM at 0x20000000 is CPU-private).
//
// Argument is void* to keep stm32h7xx.h out of spicmds.c.  Internally
// cast back to SPI_TypeDef*.
int
spi_dma_kick_tx(void *spi_void, uint8_t *tx_buf, uint16_t len,
                spi_dma_done_fn cb, void *ctx)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)spi_void;
    struct spi_dma_state *st = spi_dma_state_for(spi);
    if (st == NULL) {
        st = spi_dma_init_bus(spi);
        if (st == NULL)
            return -1;
    }
    if (st->inflight)
        return -1;
    st->inflight = 1;
    st->cb = cb;
    st->ctx = ctx;

    // Program the DMA stream with the new buffer + length.
    // STM32H7 has D-cache enabled (see stm32h7.c init), and the .dma_buf
    // section in D2 SRAM is cacheable by default.  Without flushing the
    // cache here, the DMA reads stale memory (the CPU's recent byte
    // writes still sit in the cache) and the SPI transmits old data —
    // so the TMC's XDIRECT register never updates even though `write_count`
    // climbs.  Clean the source range so the CPU's writes are visible to
    // DMA before the stream pulls bytes.
    SCB_CleanDCache_by_Addr((uint32_t *)tx_buf, len);
    DMA_Stream_TypeDef *stream = st->stream;
    stream->NDTR = len;
    stream->M0AR = (uint32_t)tx_buf;
    stream->CR |= DMA_SxCR_EN;

    // Configure SPI for this transfer length and enable TX-DMA request.
    spi->CR2 = len << SPI_CR2_TSIZE_Pos;
    spi->CFG1 |= SPI_CFG1_TXDMAEN;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;
    return 0;
}

uint8_t
spi_dma_is_inflight(void *spi_void)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)spi_void;
    struct spi_dma_state *st = spi_dma_state_for(spi);
    return st ? st->inflight : 0;
}
#endif // CONFIG_WANT_SPI_DMA && CONFIG_PHASE_STEPPER_EXPERIMENTAL_DMA
