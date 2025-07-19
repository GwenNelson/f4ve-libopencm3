PROJECT = nrf24l01-beacon2

NRFLIB = ../libnrf24l01p
NRFINC = $(NRFLIB)/include
NRFOBJS = \
	$(NRFLIB)/src/nrf24l01_hw_stm32_f4ve.o \
	$(NRFLIB)/src/nrf24l01.o \
	$(NRFLIB)/src/nrf24l01_regs.o

OBJS =\
	$(NRFOBJS) \
	main.o


CFLAGS  = -g -Os -Wall -Wextra -std=c11 -ffreestanding -mthumb -mcpu=cortex-m4 -I$(NRFINC) -Ilibopencm3/include -DSTM32F4 -DSTM32F407VG
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS = -nostartfiles -Tlinker.ld -Wl,--gc-sections -Wl,-Map=$(PROJECT).map

LIBS = libopencm3/lib/libopencm3_stm32f4.a

all: $(PROJECT).elf

$(PROJECT).elf: $(OBJS)
	arm-none-eabi-gcc $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

main.o: main.c
	arm-none-eabi-gcc $(CFLAGS) -Ilibopencm3/include -c $< -o $@

%.o: %.c
	arm-none-eabi-gcc $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.elf *.map

gdb:
	gdb -x ../gdb-init -ex "load-stm32 nrf24l01-beacon2.elf" nrf24l01-beacon2.elf
