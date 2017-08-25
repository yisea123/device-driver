MAKEFLAGS += -rR -s --no-print-directory

default:
	make -C fpga
	make -C scanunit
	make -C photosensor
	make -C motor
	make -C mechanism
	make -C usbdrv
	make -C upgrade

clean:
	make clean -C fpga
	make clean -C scanunit
	make clean -C photosensor
	make clean -C motor
	make clean -C mechanism
	make clean -C usbdrv
	make clean -C upgrade
