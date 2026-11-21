# Alternative flashing paths

## STM32 DFU (USB bootloader, no programmer needed)
1. Wire BOOT0 to 3V3 and reset the board.
2. `dfu-util -a 0 -D build/thunders_master.bin --dfuse-address 0x08000000`
3. Move BOOT0 back to GND and reset.

## UART ISP (ROM bootloader on USART1 PA9/PA10)
1. BOOT0=1, reset; the ROM loader listens at 115200 8N1.
2. `stm32flash -w build/thunders_master.bin -v -g 0x0 /dev/ttyUSB0`
