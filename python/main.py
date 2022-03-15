import time
import threading
import serial

from omniwheel import OmniwheelController


def print_serial_data(ser):
    while True:
        if ser.inWaiting():
            line = ser.readline().decode()
            if line.strip().startswith("{"):
                line = line[1:len(line) - 1]
                steps = line.split(";")
            else:
                print(ser.readline().decode(), end="")
        else:
            time.sleep(0.01)

try:
    ser = serial.Serial('/dev/ttyACM0', 4000000)
except:
    ser = serial.Serial('/dev/ttyACM1', 4000000)

serialThread = threading.Thread(target=print_serial_data, args=(ser,))
serialThread.start()

controller = OmniwheelController(ser, interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
