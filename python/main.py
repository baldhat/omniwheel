import math

from pyPS4Controller.controller import Controller
import serial
from datetime import datetime, timedelta


ser = serial.Serial('/dev/ttyACM0', 4000000)

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.direction = 0
        self.velocity = 0
        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0
        self.last_update = datetime.now()

    '''
    Pressing X enables the Interactive Driving mode
    '''
    def on_x_release(self):
        ser.write(b'{I}')

    def on_x_press(self):
        pass

    '''
    Pressing Circle disables Interactive Driving mode
    '''
    def on_square_release(self):
        ser.write(b'{E}')

    def on_square_press(self):
        pass

    def on_L3_up(self, value):
        self.last_x = - value / 32768
        self.update()

    def on_L3_down(self, value):
        self.last_x = - value / 32768
        self.update()

    def on_L3_left(self, value):
        self.last_x = - value / 32768
        self.update()

    def on_L3_right(self, value):
        self.last_x = - value / 32768
        self.update()

    def on_L3_x_at_rest(self):
        self.last_x = 0
        self.update(True)

    def on_L3_y_at_rest(self):
        self.last_y = 0
        self.update(True)

    def toPolar(self, x, y):
        r = math.sqrt(x**2 + y**2)
        t = 0
        if x == 0:
            if y > 0:
                t = 0
            elif y < 0:
                t = math.pi
        else:
            t = math.atan(y / x) if x > 0 else math.atan(y / x) + math.pi if x < 0 and y > 0 else math.atan(y / x) - math.pi
        return t, r

    def update(self, urgent=False):
        new_direction, velocity = self.toPolar(self.last_x, self.last_y)
        rotation = self.last_rot

        if datetime.now() - self.last_update > timedelta(milliseconds=50) or urgent:
            self.send('{I;' + str(new_direction) + ';'
                            + str(velocity) + ';'
                            + str(rotation) + '}')
            self.last_update = datetime.now()
            print("Direction:", new_direction)
            print("Velocity:", velocity)

    def send(self, string):
        ser.write(bytes(string.encode()))

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()