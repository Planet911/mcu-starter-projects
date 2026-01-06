This code creates a composite USB device with
* HID device
* WinUSB bulk device
* CDC Virtual COM Port (VCP)

On Windows no driver install is needed, it uses built-in OS drivers.

This project is 99.999% Alex Taradov's code, based on

https://github.com/ataradov/vcp/tree/master/stm32g441

mixed with WinUSB and HID code from here

https://github.com/ataradov/free-dap/blob/master/platform/samd11/usb_descriptors.h


The VCP connects through fifo's to UART2 on pins A2 and A3. 

The loopback_task() function in main.c creates a loopback on the HID and WinUSB endpoints.
The scripts in the Python dir can be used to send/receive data through the loopback.

Tested on this dev board:

https://github.com/WeActStudio/WeActStudio.STM32G431CoreBoard
