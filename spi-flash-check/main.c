#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#define FLASH_CS_PORT GPIOB
#define FLASH_CS_PIN  GPIO0

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

int main() {
	clock_setup();
	usart_setup();
	init_spi_flash();

	uart_puts("SPI flash info:\n");
	flash_read_id();


	for(;;);
}
