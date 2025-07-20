
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "nrf24l01_hw.h"

// Define NRF24L01 pin mappings for the F4VE board
#define NRF24_CE_PORT     GPIOB
#define NRF24_CE_PIN      GPIO6
#define NRF24_CSN_PORT    GPIOB
#define NRF24_CSN_PIN     GPIO7
#define NRF24_SPI         SPI1
#define NRF24_SPI_PORT    GPIOB
#define NRF24_SPI_AF      GPIO_AF5


void nrf24_hw_init(void)
{
    // Enable GPIOB + SPI1 clocks
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI1);

    // CE / CSN as outputs (PB6 / PB7)
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set(GPIOB, GPIO7);  // CSN high (inactive)
    gpio_clear(GPIOB, GPIO6); // CE low

    // SPI1 pins: PB3=SCK, PB4=MISO, PB5=MOSI
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4 | GPIO5);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO3 | GPIO4 | GPIO5);

    // Prepare SPI1 as master, mode 0, 8‑bit, clk = fPCLK/8
    spi_disable(SPI1);
    spi_init_master(SPI1,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_8,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    // <<< MISSING PIECE >>>
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    // Now turn the SPI engine on
    spi_enable(SPI1);
}


void nrf24_ce_digitalWrite(bool level)
{
    if (level)
        gpio_set(NRF24_CE_PORT, NRF24_CE_PIN);
    else
        gpio_clear(NRF24_CE_PORT, NRF24_CE_PIN);
}

void nrf24_csn_digitalWrite(bool level)
{
    if (level)
        gpio_set(NRF24_CSN_PORT, NRF24_CSN_PIN);
    else
        gpio_clear(NRF24_CSN_PORT, NRF24_CSN_PIN);
}

uint8_t nrf24_spi_transfer(uint8_t data)
{
    spi_send(NRF24_SPI, data);
    return spi_read(NRF24_SPI);
}

void nrf_spi_csl() {
	nrf24_csn_digitalWrite(false);
}

void nrf_spi_csh() {
	nrf24_csn_digitalWrite(true);
}

void nrf_init() {
	nrf24_hw_init();
}

unsigned char nrf_spi_xfer_byte(unsigned char data)
{
     return spi_xfer(SPI1, data);
}

