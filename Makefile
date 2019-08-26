#makefile for XR17v35x PCIe UARTs for Linux 2.6.32 and newer
#lspci -vvvn

KERNEL_SRC = /lib/modules/`uname -r`/build

all: build

obj-m += mypcie.o

xrpci-objs :=	mypcie.o

EXTRA_CFLAGS += -DDEBUG=1

build:
	$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) modules

install:
	cp mypcie.ko /lib/modules/$(shell uname -r)/kernel/drivers/char
clean:
	rm -f *~ *.o *.order *.symvers *.mod.c *.o.rc *.o.ur-safe
	rm -f *~ *.ko
