obj-m += imx662.o

#dtbo-y += imx662.dtbo

# Путь к ядру RPi4
KDIR ?= $(shell pwd)/../../rpi-kernel

# Настройки кросс-компиляции
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

#targets += $(dtbo-y)    

#always-y := $(dtbo-y)

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(shell pwd) modules

clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(shell pwd) clean

%.dtbo: %.dts
	dtc -@ -I dts -O dtb -o $@ $<

