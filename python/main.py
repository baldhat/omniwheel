import cmath
import math

from pyPS4Controller.controller import Controller
import serial
from datetime import datetime, timedelta

try:
    ser = serial.Serial('/dev/ttyACM0', 4000000)
except:
    ser = serial.Serial('/dev/ttyACM1', 4000000)


def send(string):
    ser.write(string.encode())


def to_polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    if r > 1:
        r = 1
    t = cmath.polar(x + y * 1j)[1]
    return t, r


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.direction = 0
        self.velocity = 0
        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0
        self.last_update = datetime.now()
        self.interactiveMode = False

    '''
    Pressing X enables the Interactive Driving mode
    '''

    def on_x_release(self):
        ser.write(b'{I}')
        self.interactiveMode = True
        print("Enabled Interactive Mode")

    def on_x_press(self):
        pass

    '''
    Pressing Circle disables Interactive Driving mode
    '''

    def on_square_release(self):
        ser.write(b'{E}')
        self.interactiveMode = False
        print("Disabled Interactive Mode")

    def on_square_press(self):
        pass

    def on_L3_up(self, value):
        self.last_x = - value / 32768
        self.update()

    def on_L3_down(self, value):
        self.last_x = - value / 32768
        self.update()

    def on_L3_left(self, value):
        self.last_y = - value / 32768
        self.update()

    def on_L3_right(self, value):
        self.last_y = - value / 32768
        self.update()

    def on_L3_y_at_rest(self):
        self.last_x = 0
        self.update(True)

    def on_L3_x_at_rest(self):
        self.last_y = 0
        self.update(True)

    def on_R3_left(self, value):
        self.last_rot = - value / 32768
        self.update()

    def on_R3_right(self, value):
        self.last_rot = - value / 32768
        self.update()

    def on_R3_down(self, value):
        pass

    def on_R3_up(self, value):
        pass

    def on_R3_x_at_rest(self):
        self.last_rot = 0
        self.update(True)

    def on_R3_y_at_rest(self):
        self.last_rot = 0
        self.update(True)

    def update(self, urgent=False):
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot

        if (datetime.now() - self.last_update > timedelta(milliseconds=50) or urgent) and self.interactiveMode:
            commandString = '{I;' + str(round(new_direction, 2)) + \
                            ';' + str(round(velocity, 2)) + \
                            ';' + str(round(rotation, 2)) + ';}'
            send(commandString)
            self.last_update = datetime.now()
            print(commandString)


controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
