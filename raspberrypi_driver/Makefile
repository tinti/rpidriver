obj-m := uc_msp430.o
KDIR := /mnt/w1/tinti/work/rpi/linux
PWD := $(shell pwd)
all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean
