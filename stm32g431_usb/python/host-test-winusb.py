import usb.core
import usb.util
import sys,time
import os

'''
Step 1:
Install libusb. Download the latest release 7zip archive which has prebuilt DLL
https://github.com/libusb/libusb/releases
Copy libusb-1.0.dll to somewhere on your path, e.g. Windows/System32/

Step 2:
Install python
Install bindings for libusb with the command line:
pip install pyusb

Step 3:
Make sure your MCU is running the WINUSB device firmware and check it's listed in Device Manager.
When you are experimenting with the USB stack, if a device fails to respond on first connect
to WCID 0xEE string request, Windows will likely make a note of it in the registry and not
request 0xEE string from that device again, even after you fix firmware. From docs/winusb.txt:

Windows will only read the Microsoft-specific descriptors the first time the
device is connected, which is undesirable during development of a WinUSB
device.  Windows can be forced to re-read the descriptors by performing the
following steps:
	1. In the Device Manager, right-click the device (check VID/PID) and
	   select "Uninstall." This will unbind the driver.
	2. In the registry, delete the section associated with your device in
	   Computer\\HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\usbflags\\VID_PID
	   The device keys in UsbFlags begin with the VID and PID of the
	   device they describe.
	3. Disconnect and re-connect your device. The next time the device
	   is connected, Windows will request the WinUSB descriptors.

Step 4:
Run this script and you should see some output like this:

Manufacturer: Demo Device Manuftr.
Product:      Composite Demo Device A
Serial:       12345678
Endpoint loopback:  74 kbytes/sec
'''

#https://github.com/pyusb/pyusb/blob/master/docs/tutorial.rst
#https://pid.codes/howto/

VENDOR_ID =  0x6666
PRODUCT_ID = 0x8802

dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

if dev is None:
    raise ValueError('Device not found.')
    sys.exit(1)

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

# print(dev) #show all device info
# exit()

str_manuf = usb.util.get_string(dev, dev.iManufacturer)
str_product = usb.util.get_string(dev, dev.iProduct)
str_serial = usb.util.get_string(dev, dev.iSerialNumber)
print('Manufacturer: ' + str_manuf)
print('Product:      ' + str_product)
print('Serial:       ' + str_serial)


pktsz = 64 # packet size in bytes
msg = os.urandom(pktsz)
npkts = 2000

# dev.write(0x03, msg) # To endpoint 3_OUT write msg with 100ms timeout
# ret = dev.read(0x84, pktsz, 100) # From endpoint 4_IN read 64 bytes with 100ms timeout
# print(''.join('{:02X} '.format(a) for a in msg))
# print(''.join('{:02X} '.format(a) for a in ret))
# exit()

start = time.perf_counter()
for i in range(npkts):
    dev.write(0x03, msg) # To endpoint 3_OUT write msg with 100ms timeout
    ret = dev.read(0x84, pktsz, 100) # From endpoint 4_IN read 64 bytes with 100ms timeout
    # sret = ''.join([chr(x) for x in ret])
    # print(''.join('{:02X} '.format(a) for a in ret))
finish = time.perf_counter()
print('Endpoint loopback: ', int((pktsz*npkts)/(finish-start)/1024), 'kbytes/sec')
