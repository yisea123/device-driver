KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules
obj-m := fpga_adc.o fpga_gpio.o fpga_pwm.o photosensor.o

#specify flags for the module compilation
EXTRA_CFLAGS=-g -O0 -D CONFIG_FPGA_EIM -D DEBUG

#build: kernel_modules

#kernel_modules:
default:
	if [ ! -f Module.symvers ] ; then ln -sv ../fpga/Module.symvers ; fi;
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules 
clean:
	make -C $(KDIR) M=$(PWD) clean
