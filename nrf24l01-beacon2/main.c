#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <nrf24l01.h>
#include <nrf24l01_regs.h>
#include <nrf24l01_hw.h>

void clock_setup(void) {
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(168000000 / 1000); // 1ms tick if 168 MHz
//	systick_interrupt_enable(); // Optional, if you want to use interrupts
	systick_counter_enable();

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

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
        while ((STK_CSR & STK_CSR_COUNTFLAG) == 0);
    }
}


void nrf24_hw_init();

int main(void) {
	clock_setup();

	usart_setup();

	nrf_payload beacon_payload_buf = {
		.size = 32,
		.data = "HELLO!"
	};

	nrf_reg_buf addr_buf = {
		.size = 5,
		.data = {0x00,0x00,0x00,0x00,0x00}
	};

	// configuration register
	nrf_reg_buf config_buf = {
		.size = 1,
		.data = { 1 << NRF_REGF_PWR_UP |
	       		  1 << NRF_REGF_EN_CRC}
	};

	// auto-ack register
	nrf_reg_buf aa_buf = {
		.size = 1,
		.data = { 1 << NRF_REGF_ENAA_P0 }// enable auto-ack
	};

	// en_rxaddr pipe 0
	nrf_reg_buf en_rxaddr_buf = {
		.size = 1,
		.data = {1 << NRF_REGF_ERX_P0}
	};

	// setup_retr (retry delay and count)
	nrf_reg_buf setup_retr_buf = {
		.size = 1,
		.data = {0x4F} // 500us delay, 15 attempts
	};

	// RF channel buffer, channel 76 (2.476GHz)
	nrf_reg_buf rf_ch_buf = {
		.size = 1,
		.data = {76}
	};

	// RF configuration
	nrf_reg_buf rf_setup_buf = {
		.size = 1,
		.data = {0b00000110} // 1Mbps, 0dBm
	};

	// STATUS register
	nrf_reg_buf rf_status_buf = {
		.size =1,
		.data = {0x70}
	};

	// payload width register
	nrf_reg_buf rx_pw_p0_buf = {
		.size = 1,
		.data = {32}
	};

	uart_puts("Configuring the nRF24 module...");
	nrf24_hw_init();

	nrf_write_reg(NRF_REG_CONFIG,    &config_buf);
	nrf_write_reg(NRF_REG_EN_AA,     &aa_buf);
	nrf_write_reg(NRF_REG_TX_ADDR,   &addr_buf);
	nrf_write_reg(NRF_REG_RX_ADDR_P0,&addr_buf);
	nrf_write_reg(NRF_REG_EN_RXADDR, &en_rxaddr_buf);
	nrf_write_reg(NRF_REG_SETUP_RETR,&setup_retr_buf);
	nrf_write_reg(NRF_REG_RF_CH,     &rf_ch_buf);
	nrf_write_reg(NRF_REG_RF_SETUP,  &rf_setup_buf);
	nrf_write_reg(NRF_REG_RX_PW_P0,  &rx_pw_p0_buf);
	nrf_write_reg(NRF_REG_STATUS,    &rf_status_buf);
	uart_puts("DONE\n");

	uart_puts("Sending beacon");
	for(;;) {
		nrf_send_blocking(&beacon_payload_buf);
		uart_puts(".");
		delay_ms(100);
	}
}
