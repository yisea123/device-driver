INCLUDES := $(obj)/../../../work/$(master)/fw/include
KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules

ifeq ($(machine),mx6q_voucher_scanner)
	obj-m := dcmotor.o steppermotor.o fpga_motor.o fpga_dcmotor.o fpga_stepmotor.o sysfs_motor.o
else ifeq ($(machine),mx6q_voucher_scanner_printer)
	obj-m := dcmotor.o steppermotor.o fpga_motor.o fpga_dcmotor.o fpga_stepmotor.o sysfs_motor.o
else ifeq ($(machine),mx6q_voucher_scanner_dual_recycle_boxes)
	obj-m := dcmotor.o steppermotor.o fpgax_motor.o fpgax_dcmotor.o fpgax_stepmotor.o sysfs_motor.o
else ifeq ($(machine),mx6ul_voucher_dispenser)
	obj-m := dcmotor.o gpio_dcmotor.o pwm-gwi.o steppermotor.o pwm_stepmotor.o sysfs_motor.o
else ifeq ($(machine),mx6ul_voucher_dispenser_move_box)
	obj-m := dcmotor.o gpio_dcmotor.o pwm-gwi.o steppermotor.o pwm_stepmotor.o sysfs_motor.o
else ifeq ($(machine),mx6ul_voucher_dispenser_paper_pattern)
	obj-m := dcmotor.o gpio_dcmotor.o pwm-gwi.o steppermotor.o pwm_stepmotor.o sysfs_motor.o
else ifeq ($(machine),mx6ul_thermal_printer)
	obj-m := dcmotor.o pwm_dcmotor.o gpio_dcmotor.o pwm-gwi.o steppermotor.o pwm_stepmotor.o sysfs_motor.o
endif

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
