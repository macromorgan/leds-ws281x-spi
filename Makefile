obj-m	+= leds-ws281x-spi.o

KVERSION := $(shell uname -r)
all:
	$(MAKE) modules
