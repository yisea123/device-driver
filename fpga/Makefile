KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules
obj-m := fpga_io.o fpga_debug_eim.o 

#specify flags for the module compilation
EXTRA_CFLAGS=-g -O3 -march=armv7-a -mtune=cortex-a8 -mfpu=neon -ftree-vectorize -mfloat-abi=softfp -D CONFIG_FPGA_EIM -D CONFIG_REG_DOWMLOAD

#build: kernel_modules

#kernel_modules:
default:
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules   
clean:
	make -C $(KDIR) M=$(PWD) clean

