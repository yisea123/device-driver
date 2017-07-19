KVERS := $(shell uname -r)
#KDIR := /lib/modules/$(KVERS)/build
#KDIR :=~/fsl_yocto/fsl-release-bsp/build-base/tmp/work/imx6qsabresd-poky-linux-gnueabi/linux-imx/4.1.15-r0/build/
#KDIR := ./kernel_root
KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules
# obj-m := mech_motor.o mech_sensor.o mech_unit.o paperpath.o acceptor.o

# define include file directory for all scanner devices drivers
#subdir-ccflags-y := -I$(src)/include
INCLUDES := $(obj)/../../../work/*/fw/include
#subdir-ccflags-y := -I $(INCLUDES)


#obj-m := mechdrv.o paperpath.o acceptor.o mechunit.o
#obj-m := mechdrv.o paperpath.o acceptor.o 
obj-m := mechdrv.o 

mechdrv-objs := mech_motor.o \
		mech_sensor.o \
		mech_unit.o 


#paperpath-objs := paperpath_driver.o

#acceptor-objs := accept_driver.o 

#mechunit-objs := mechanism.o 
	     
#specify flags for the module compilation
EXTRA_CFLAGS=-g -O0 -D CONFIG_FPGA_EIM -I $(INCLUDES) -D DEBUG
#EXTRA_CFLAGS+=$(INCLUDES)
#build: kernel_modules

#kernel_modules:
default:
	if [ ! -f Module.symvers ] ; then ln -sv ../fpga/Module.symvers ; fi;
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules 
clean:
	make -C $(KDIR) M=$(PWD) clean

