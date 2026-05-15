#ifndef __SPICMDS_H
#define __SPICMDS_H

#include <stdint.h> // uint8_t

struct spidev_s *spidev_oid_lookup(uint8_t oid);
struct spi_software;
void spidev_set_software_bus(struct spidev_s *spi, struct spi_software *ss);
int spidev_have_cs_pin(struct spidev_s *spi);
struct gpio_out spidev_get_cs_pin(struct spidev_s *spi);
struct spi_config spidev_get_spi_config(struct spidev_s *spi);
uint8_t spidev_is_cs_active_high(struct spidev_s *spi);
void spidev_transfer(struct spidev_s *spi, uint8_t receive_data
                     , uint8_t data_len, uint8_t *data);
uint8_t spidev_is_bus_busy(void);
void spidev_set_bus_busy(uint8_t busy);
void spidev_prepare_bus(struct spidev_s *spi);
void spidev_transfer_prepared(struct spidev_s *spi, uint8_t data_len,
                              uint8_t *data);

// DMA fire-and-forget transfer.  Caller manages CS via the cb (called
// from the DMA-TC IRQ, ISR-context).  tx_buf MUST be in DMA-reachable
// memory — on STM32H7 use the .dma_buf section attribute (DTCM at
// 0x20000000 is CPU-private and unreachable by DMA1).
typedef void (*spi_dma_done_fn)(void *ctx);
int  spidev_kick_dma_tx(struct spidev_s *spi, uint8_t *tx_buf, uint16_t len,
                        spi_dma_done_fn cb, void *ctx);
// Chained variant: skip spi_prepare().  Use ONLY when the previous
// transfer in this IRQ chain already configured the SPI peripheral
// (same bus, same speed, same mode — typical phase-stepping case
// where all motors share SPI2 with identical config).  Saves ~250ns
// per call on H7; over 3 chained motors that's ~750ns of tick budget
// recovered, often the margin between fitting in the tick period and
// the 50% skip pattern.
int  spidev_kick_dma_tx_chained(struct spidev_s *spi, uint8_t *tx_buf,
                                uint16_t len, spi_dma_done_fn cb, void *ctx);
uint8_t spidev_dma_in_flight(struct spidev_s *spi);

#endif // spicmds.h
