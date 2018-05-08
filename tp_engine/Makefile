KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules

INCLUDES := $(obj)/../../../../sources/common/include -I $(obj)/../../../../sources/common/fw/include

obj-m := tp_engine_drv.o
tp_engine_drv-objs = tp_engine.o tp_engine_sensor.o tp_engine_ph.o tp_engine_ph_motor.o \
			tp_engine_ribbon_motor.o tp_engine_pap_motor.o

#specify flags for the module compilation
EXTRA_CFLAGS=-g -O0 -D CONFIG_FPGA_EIM -I $(INCLUDES) -D DEBUG
#build: kernel_modules

#kernel_modules:
default:
	if [ ! -f Module.symvers ] ; then ln -sv ../fpga/Module.symvers ; fi;
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules 
clean:
	make -C $(KDIR) M=$(PWD) clean
