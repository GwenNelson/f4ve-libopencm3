#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void clock_setup(void) {
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
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

/*uint16_t usart_recv_blocking(uint32_t usart)
{
    while (!(USART_SR(usart) & USART_SR_RXNE)); // Wait for data
    return usart_recv(usart); // Return received byte
}*/


int main(void) {
	clock_setup();
	usart_setup();

	while (1) {

		 if (USART_SR(USART1) & USART_SR_RXNE) {
		        uint8_t ch = usart_recv(USART1);  // Only called when new data available
		        usart_send_blocking(USART1, ch);           // Echo it back
		    }

//		usart_send_blocking(USART1, usart_recv_blocking(USART1)); // Transmit known byte forever
//		for (int i = 0; i < 1000000; i++) __asm__("nop");
	}
}
