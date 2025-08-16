# Makefile para Arduino Serial Mouse Driver
# Para compilação externa (out-of-tree)

obj-m := arduino-serial-mouse.o

# Localizar diretório do kernel corretamente
KERNEL_BUILD := /home/carlao/WSL2-Linux-Kernel

# Flags de compilação
ccflags-y := -Wall -Wextra

all:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) clean
	rm -f Module.symvers modules.order

install:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) modules_install

help:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) help

.PHONY: all clean install help
