import serial

bluetooth_serial = serial.Serial("COM8", baudrate=57600, timeout=0)


def bluetooth_read():
    while True:
        data = bluetooth_serial.readline()
        data_decoded = memoryview(data).cast('B')

        for i in range(int(len(data_decoded) / 2)):
            print('X: ' + str(data_decoded[i + 1]))
            print('Y: ' + str(data_decoded[i]))

bluetooth_read()
