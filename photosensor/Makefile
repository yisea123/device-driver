KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules

ifeq ($(machine), mx6q_voucher_scanner)
	obj-m := fpga_adc.o fpga_gpio.o fpga_pwm.o photosensor.o
else ifeq ($(machine), mx6q_voucher_scanner_dual_recycle_boxes)
	obj-m := fpga_adc.o fpgax_gpio.o fpgax_pwm.o xphotosensor.o
else ifeq ($(machine), mx6ul_voucher_dispenser)
	obj-m := photosensor.o
	photosensor-objs := dispenser_photosensor.o
else ifeq ($(machine), mx6ul_thermal_printer)
	obj-m := photosensor.o
	photosensor-objs := dispenser_photosensor.o
else ifeq ($(machine), mx6ul_voucher_dispenser_move_box)
	obj-m := photosensor.o
	photosensor-objs := dispenser_photosensor.o
else ifeq ($(machine), mx6ul_voucher_dispenser_paper_pattern)
	obj-m := photosensor.o
	photosensor-objs := dispenser_photosensor.o
endif

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
