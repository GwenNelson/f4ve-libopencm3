#include <stdint.h>
#include <stddef.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#define FLASH_CS_PORT GPIOB
#define FLASH_CS_PIN  GPIO0

#define NRF_CSN_PORT GPIOB
#define NRF_CSN_PIN  GPIO7
#define NRF_CE_PORT  GPIOB
#define NRF_CE_PIN   GPIO6

#define NRF_REG_CONFIG     0x00
#define NRF_REG_RF_CH      0x05
#define NRF_REG_RF_SETUP   0x06
#define NRF_REG_STATUS     0x07
#define NRF_REG_TX_ADDR    0x10
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_REG_EN_AA      0x01
#define NRF_REG_SETUP_RETR 0x04


void clock_setup(void) {
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	rcc_periph_clock_enable(RCC_USART1);
}

void putchar_usart(char c) {
    if (c == '\n') {
        usart_send_blocking(USART1, '\r');
    }
    usart_send_blocking(USART1, c);
}

void puthex(uint32_t val) {
    for (int i = 28; i >= 0; i -= 4) {
        uint8_t nibble = (val >> i) & 0xF;
        char hex = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
        putchar_usart(hex);
    }
}

void uart_puts(char* s) {
	while(*s) putchar_usart(*s++);
}

void usart_setup(void) {
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10); // USART1 TX/RX

	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}

uint32_t flash_read_id(void) {
	uint32_t id = 0;

	// Pull CS low to begin communication
	gpio_clear(GPIOB, GPIO0);


	// Send JEDEC ID command
	spi_send(SPI1, 0x9F);
	(void)spi_read(SPI1); // annoyingly, we have to read a dummy read first, bla

	// Dummy writes to generate clocks for reads
	spi_send(SPI1, 0x00);
	uint8_t mfr = spi_read(SPI1);

	spi_send(SPI1, 0x00);
	uint8_t type = spi_read(SPI1);

	spi_send(SPI1, 0x00);
	uint8_t capacity = spi_read(SPI1);

	uart_puts("\n\t mfr=");      puthex(mfr);
	uart_puts("\n\t type=");     puthex(type);
	uart_puts("\n\t capacity="); puthex(capacity);

	// Release CS
	gpio_set(GPIOB, GPIO0);

	// Combine into a 24-bit result
	id = (mfr << 16) | (type << 8) | capacity;
	return id;
}

void init_spi_flash() {
    // Enable GPIOB and SPI1 clocks
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI1);

    // Set up SPI1 pins: SCK=PB3, MISO=PB4, MOSI=PB5 (AF5)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO3 | GPIO4 | GPIO5);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO3 | GPIO4 | GPIO5); // SCK MISO MISO

    // Set up CS pin manually
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_set(GPIOB, GPIO0); // Deselect flash

    // SPI1 configuration
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);
    spi_enable(SPI1);

}

void init_nrf_gpio() {
	// get the chip ready
	gpio_mode_setup(NRF_CSN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRF_CSN_PIN);
	gpio_set(NRF_CSN_PORT, NRF_CSN_PIN); // CSN high (inactive)

	gpio_mode_setup(NRF_CE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRF_CE_PIN);
	gpio_clear(NRF_CE_PORT, NRF_CE_PIN); // CE low (standby)


}

uint8_t nrf_read_register(uint8_t reg) {
    gpio_clear(NRF_CSN_PORT, NRF_CSN_PIN);
    spi_send(SPI1, reg & 0x1F); // Read command
    (void)spi_read(SPI1);       // Dummy read
    spi_send(SPI1, 0xFF);       // Send NOP to receive
    uint8_t val = spi_read(SPI1);
    gpio_set(NRF_CSN_PORT, NRF_CSN_PIN);
    return val;
}

void nrf_check_status(void) {
    uart_puts("\nNRF STATUS: ");
    uint8_t status = nrf_read_register(0x07); // STATUS
    puthex(status);
}

void nrf_write_register(uint8_t reg, uint8_t val) {
    gpio_clear(NRF_CSN_PORT, NRF_CSN_PIN);
    spi_send(SPI1, 0x20 | (reg & 0x1F));
    spi_send(SPI1, val);
    gpio_set(NRF_CSN_PORT, NRF_CSN_PIN);
}

uint8_t nrf_get_status(void) {
    gpio_clear(NRF_CSN_PORT, NRF_CSN_PIN);
    spi_send(SPI1, 0xFF);
    uint8_t s = spi_read(SPI1);
    gpio_set(NRF_CSN_PORT, NRF_CSN_PIN);
    return s;
}

void nrf_flush_tx(void) {
    gpio_clear(NRF_CSN_PORT, NRF_CSN_PIN);
    spi_send(SPI1, 0xE1); // FLUSH_TX
    gpio_set(NRF_CSN_PORT, NRF_CSN_PIN);
}


void nrf_send_packet(const uint8_t *data, size_t len) {
    // Set up some basic config
    nrf_write_register(NRF_REG_CONFIG, 0x0E); // PWR_UP, TX mode
    nrf_write_register(NRF_REG_RF_CH, 76);    // 2.476 GHz
    nrf_write_register(NRF_REG_RF_SETUP, 0x06); // 1 Mbps

    // Write TX address (5 bytes)
    gpio_clear(NRF_CSN_PORT, NRF_CSN_PIN);
    spi_send(SPI1, 0x20 | NRF_REG_TX_ADDR);
    spi_send(SPI1, 0x00);
    spi_send(SPI1, 0x00);
    spi_send(SPI1, 0x00);
    spi_send(SPI1, 0x00);
    spi_send(SPI1, 0x00);
    gpio_set(NRF_CSN_PORT, NRF_CSN_PIN);

    nrf_write_register(NRF_REG_STATUS, 0x70);

    // Send payload
    gpio_clear(NRF_CSN_PORT, NRF_CSN_PIN);
    spi_send(SPI1, NRF_CMD_W_TX_PAYLOAD);
    for (size_t i = 0; i < len; ++i)
        spi_send(SPI1, data[i]);
    gpio_set(NRF_CSN_PORT, NRF_CSN_PIN);

    // Pulse CE
    gpio_set(NRF_CE_PORT, NRF_CE_PIN);
    __asm__ volatile("nop");
    gpio_clear(NRF_CE_PORT, NRF_CE_PIN);

    // Poll STATUS
    uint8_t s;
    do {
        s = nrf_get_status();
    } while (!(s & 0x30)); // TX_DS or MAX_RT

    uart_puts("\nTX ");
    if (s & 0x20)
        uart_puts("OK");
    else if (s & 0x10)
        uart_puts("FAIL");

    // Clear interrupt flags
    nrf_write_register(NRF_REG_STATUS, 0x70);
    nrf_flush_tx();
}



int main() {
	clock_setup();
	usart_setup();
	init_spi_flash();
	
	init_nrf_gpio();

	uart_puts("SPI flash info:\n");
	flash_read_id();

	uart_puts("\nNRF24L01+ status check:\n");
	nrf_check_status();

	uart_puts("\n\n");

	const uint8_t payload[8] = {0xFE,0xED,0xFA,0xCE,0xDE,0xAD,0xBE,0xEF}; // FEEDFACEDEADBEEF

nrf_write_register(NRF_REG_EN_AA, 0x00);      // Disable auto‑ack
nrf_write_register(NRF_REG_SETUP_RETR, 0x00); // No retransmits

	for(;;) {
		uart_puts("Sending beacon...");
		nrf_send_packet(payload,8);
		for (volatile int i = 0; i < 100000; i++) __asm__("nop");
		uart_puts("SENT\n");
	}

	for(;;);
}
