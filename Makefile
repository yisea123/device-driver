MAKEFLAGS += -rR -s --no-print-directory


PHONY += mx6q_voucher_scanner:
mx6q_voucher_scanner:
	make -C fpga
	make -C scanunit
	make -C photosensor
	make -C motor
	make -C mechanism
	make -C usbdrv
	make -C upgrade

PHONY += mx6ul_thermal_printer:
mx6ul_thermal_printer:
	make -C photosensor
	make -C motor
	make -C tp_printer_header
	make -C tp_engine
	
PHONY += mx6ul_voucher_dispenser:
mx6ul_voucher_dispenser:
	make -C photosensor
	make -C motor
	make -C mechanism
	make -C usbdrv
	make -C upgrade

default:
	make -C fpga
	make -C scanunit
	make -C photosensor
	make -C motor
	make -C mechanism
	make -C usbdrv
	make -C upgrade
	make -C tp_printer_header
	make -C tp_engine


clean:
	make clean -C fpga
	make clean -C scanunit
	make clean -C photosensor
	make clean -C motor
	make clean -C mechanism
	make clean -C usbdrv
	make clean -C upgrade
	make clean -C tp_printer_header
	make clean -C tp_engine

