KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules
obj-m := imagedigitiser.o imagesensor.o fpga_cis.o fpga_afe.o spi_wm8215.o scanunitdrv.o

scanunitdrv-objs := scanunit.o scanunit_ioctl.o

#specify flags for the module compilation
EXTRA_CFLAGS=-g -O0 -D DEBUG -D CONFIG_FPGA_EIM -D CONFIG_MICROSEMI_010T

#build: kernel_modules

#kernel_modules:
default:
	if [ ! -f Module.symvers ] ; then ln -sv ../fpga/Module.symvers ; fi;
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules 
clean:
	make -C $(KDIR) M=$(PWD) clean
