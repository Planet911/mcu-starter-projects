#pip install hidapi
#documentation at
#https://trezor.github.io/cython-hidapi/

import hid, time

show_all_devices = False
#show_all_devices = True

if show_all_devices:
    for device_dict in hid.enumerate():
        keys = list(device_dict.keys())
        keys.sort()
        for key in keys:
            print("%s : %s" % (key, device_dict[key]))
        print()
    exit()

try:
    h = hid.device()
    h.open(0x6666, 0x8802)

    print("Manufacturer: %s" % h.get_manufacturer_string())
    print("Product: %s" % h.get_product_string())
    print("Serial No: %s" % h.get_serial_number_string())

    h.set_nonblocking(1) # enable non-blocking mode
    print("Writing data...")
    pkt = [1, 2, 3, 4] + [5] * 60
    print(pkt)

    report_id = 0
    h.write([report_id] + pkt) # hidapi sends 64 bytes only when the first byte is 0 (meaning there is no report id)

    time.sleep(0.05)

    print("Reading data...")
    while True:
        d = h.read(64)
        if d:
            print(d)
        else:
            break

    h.close()
    print("Done")

except IOError as ex:
    print(ex)
    print("Couldn't open device, check VID:PID")
