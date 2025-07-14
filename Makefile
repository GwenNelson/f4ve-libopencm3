PROJECTS =\
	  blinky\
	  serial-echo\
	  spi-flash-check

all: libopencm3 $(PROJECTS)

libopencm3:
	make -C libopencm3 TARGET=stm32/f4

$(PROJECTS):
	make -C $@

clean:
	for p in $(PROJECTS); do \
		$(MAKE) -C $$p clean; \
	done

.PHONY: $(PROJECTS) libopencm3 all clean
