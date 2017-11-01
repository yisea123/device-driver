KDIR := ../../kernel_root
KVERS := $(shell uname -r)
KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules
obj-m := fpga_pwm.o photosensor.o
# fpga_debug_eim.o fpga_eim_sdma.o

#specify flags for the module compilation
EXTRA_CFLAGS=-g -O0 -D CONFIG_FPGA_EIM -D CONFIG_REG_DOWMLOAD

#build: kernel_modules

#kernel_modules:
default:
	if [ ! -f Module.symvers ] ; then ln -sv ../fpga/Module.symvers ; fi;
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
