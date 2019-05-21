KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules

ifeq ($(machine),mx6q_voucher_scanner)
	obj-m := fpga_io.o fpga_debug_eim.o
else ifeq ($(machine),mx6q_voucher_scanner_dual_recycle_boxes)
	obj-m := fpga_io.o fpga_debug_eim.o fpgax_io.o fpgax_debug.o
else ifeq ($(machine),mx6q_voucher_scanner_dual_recycle_boxes_printer)
	obj-m := fpga_io.o fpga_debug_eim.o fpgax_io.o fpgax_debug.o
else ifeq ($(machine),mx6q_voucher_scanner_printer)
	obj-m := fpga_io.o fpga_debug_eim.o fpgax_io.o fpgax_debug.o
else ifeq ($(machine),mx6q_voucher_scanner_paper_pattern)
	obj-m := fpga_io.o fpga_debug_eim.o
endif


#specify flags for the module compilation
EXTRA_CFLAGS=-g -O3 -march=armv7-a -mtune=cortex-a8 -mfpu=neon -ftree-vectorize -mfloat-abi=softfp -D CONFIG_FPGA_EIM -D CONFIG_REG_DOWNLOAD

#build: kernel_modules

#kernel_modules:
default:
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules   
clean:
	make -C $(KDIR) M=$(PWD) clean

