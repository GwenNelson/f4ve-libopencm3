
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
    // Enable clocks
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI1);

    // Setup CE and CSN as output
    gpio_mode_setup(NRF24_CE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRF24_CE_PIN);
    gpio_mode_setup(NRF24_CSN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRF24_CSN_PIN);

    // Setup SPI pins: SCK (PB3), MISO (PB4), MOSI (PB5)
    gpio_mode_setup(NRF24_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4 | GPIO5);
    gpio_set_af(NRF24_SPI_PORT, NRF24_SPI_AF, GPIO3 | GPIO4 | GPIO5);

    // Setup SPI1
    spi_disable(NRF24_SPI);
    spi_init_master(NRF24_SPI,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_8,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);
    spi_enable(NRF24_SPI);
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
