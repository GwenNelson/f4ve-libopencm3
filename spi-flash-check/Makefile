PROJECT = spi-flash-check
OBJS = main.o

CFLAGS  = -g -Os -Wall -Wextra -std=c11 -ffreestanding -mthumb -mcpu=cortex-m4 -Ilibopencm3/include -DSTM32F4 -DSTM32F407VG
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS = -nostartfiles -Tlinker.ld -Wl,--gc-sections -Wl,-Map=$(PROJECT).map

LIBS = libopencm3/lib/libopencm3_stm32f4.a

all: $(PROJECT).elf

$(PROJECT).elf: $(OBJS)
	arm-none-eabi-gcc $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

main.o: main.c
	arm-none-eabi-gcc $(CFLAGS) -Ilibopencm3/include -c $< -o $@

clean:
	rm -f *.o *.elf *.map

gdb:
	gdb -x ../gdb-init -ex "load-stm32 spi-flash-check.elf" spi-flash-check.elf
