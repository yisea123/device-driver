KDIR := ../../kernel_root
PWD := $(shell pwd)
# kernel modules
obj-m := hsusb.o

#build: kernel_modules
#kernel_modules:      
default:  
	. /opt/fsl-imx-fb/4.1.15-1.1.0/environment-setup-cortexa9hf-vfp-neon-poky-linux-gnueabi; \
	make -C $(KDIR) M=$(PWD) modules
clean:  
	make -C $(KDIR) M=$(PWD) clean
