

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.

ifneq ($(KERNELRELEASE),)

#     obj-m := ipex_lt.o
#module-objs := other.o

# Otherwise we were called directly from the command
# line; invoke the kernel build system.

else

#     KERNELDIR ?= $(shell pwd)/../../linux-source-2.6.26-2-686-DEBUG
     KERNELDIR ?= /lib/modules/$(shell uname -r)/build
     PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

endif

ipex_lt.o: ipex_lt.h
