EXTRA_CFLAGS += -g

ifneq ($(KERNELRELEASE),)
   obj-m := sop.o
	 
else
   KERNELDIR ?= /lib/modules/$(shell uname -r)/build
   PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean: 
	rm -rf .tmp*
	rm -f *.o *.ko* *.gz .*.cmd *.mod.c
	rm -f Module.* modules.order description-pak

endif
