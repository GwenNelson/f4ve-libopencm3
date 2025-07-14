#include <stdint.h>

#define RCC_AHB1ENR   (*(volatile uint32_t*)0x40023830)
#define GPIOA_MODER   (*(volatile uint32_t*)0x40020000)
#define GPIOA_ODR     (*(volatile uint32_t*)0x40020014)
#define LED_PIN       (1 << 6)  // PA6

void delay(volatile uint32_t t) {
    while (t--) __asm__("nop");
}

int main(void) {
    // Enable GPIOA clock (bit 0 in RCC AHB1ENR)
    RCC_AHB1ENR |= (1 << 0);

    // Set PA6 as output (MODER6 = 0b01)
    GPIOA_MODER &= ~(0b11 << (6 * 2));  // Clear
    GPIOA_MODER |=  (0b01 << (6 * 2));  // Set to output

    // Blink loop
    while (1) {
        GPIOA_ODR ^= LED_PIN;  // Toggle PA6
        delay(1000000);
    }
}
