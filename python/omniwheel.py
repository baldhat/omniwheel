from pyPS4Controller.controller import Controller
from datetime import datetime, timedelta
import cmath
import math
from serial import Serial

def send(ser, string):
    ser.write(string.encode())


def to_polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    if r > 1:
        r = 1
    t = cmath.polar(x + y * 1j)[1]
    return t, r


class OmniwheelController(Controller):

    def __init__(self, ser, **kwargs):
        Controller.__init__(self, **kwargs)
        self.direction = 0
        self.velocity = 0
        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0
        self.last_update = datetime.now()
        self.interactiveMode = False
        self.ser: Serial = ser

        self.micro_step_options = [1, 2, 4, 8, 16, 32]
        self.micro_steps = self.get_micro_steps()
        self.max_wheel_velocity = self.get_max_wheel_velocity()

    def on_x_release(self):
        """
            Pressing X enables the Interactive Driving mode
        """
        self.ser.write(b'{I}')
        self.interactiveMode = True

    def on_x_press(self):
        pass


    def on_square_release(self):
        """
            Pressing Square disables Interactive Driving mode
        """
        self.ser.write(b'{E}')
        self.interactiveMode = False

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
        self.update()

    def on_L3_x_at_rest(self):
        self.last_y = 0
        self.update()

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
        self.update()

    def on_R3_y_at_rest(self):
        pass

    def on_up_arrow_press(self):
        if not self.interactiveMode:
            self.micro_steps = self.micro_step_options[(self.micro_step_options.index(self.micro_steps) + 1) % len(self.micro_step_options)]
            print("Setting micro steps to", self.micro_steps)
            self.ser.write(b'{M;' + str(self.micro_steps).encode() + b'}')

    def on_up_down_arrow_release(self):
        pass

    def on_right_arrow_press(self):
        if not self.interactiveMode:
            self.max_wheel_velocity = self.max_wheel_velocity + 0.01
            print("Setting max wheel velocity to", self.max_wheel_velocity)
            self.ser.write(b'{S;' + str(self.max_wheel_velocity).encode() + b'}')

    def on_left_arrow_press(self):
        if not self.interactiveMode:
            self.max_wheel_velocity = self.max_wheel_velocity - 0.01
            print("Setting max wheel velocity to", self.max_wheel_velocity)
            self.ser.write(b'{S;' + str(self.max_wheel_velocity).encode() + b'}')

    def on_left_right_arrow_release(self):
        pass

    def update(self):
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot

        if self.interactiveMode:
            commandString = '{I;' + str(round(new_direction, 2)) + \
                            ';' + str(round(velocity, 2)) + \
                            ';' + str(round(rotation, 2)) + ';}'
            send(self.ser, commandString)
            self.last_update = datetime.now()

    def get_micro_steps(self):
        self.ser.write(b'{m}')
        while not self.ser.inWaiting():
            pass
        micro_steps = int(self.ser.readline())
        print("Using micro steps:", micro_steps)
        return micro_steps

    def get_max_wheel_velocity(self):
        self.ser.write(b'{s}')
        while not self.ser.inWaiting():
            pass
        max_wheel_velocity = float(self.ser.readline())
        print("Using max wheel velocity:", max_wheel_velocity)
        return max_wheel_velocity
