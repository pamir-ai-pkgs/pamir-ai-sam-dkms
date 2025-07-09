obj-m += pamir-ai-sam.o

pamir-ai-sam-objs := pamir-sam-main.o pamir-sam-protocol.o pamir-sam-input.o pamir-sam-led.o pamir-sam-power.o pamir-sam-display.o pamir-sam-debug.o pamir-sam-system.o pamir-sam-chardev.o

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install

.PHONY: all clean install