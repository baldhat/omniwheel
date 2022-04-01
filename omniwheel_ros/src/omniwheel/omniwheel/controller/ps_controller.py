from evdev import InputDevice, categorize, ecodes, KeyEvent

import math
import time
from select import select
from glob import glob

import rclpy
from rclpy.node import Node

from omniwheel_interfaces.msg import ControllerValue
from omniwheel_interfaces.srv import EnableMotors

from omniwheel.helper.helper import to_polar


class PSController(Node):
    
    def __init__(self):
        super().__init__('controller_publisher')
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.motors_enabled = False
        self.conroller_value_timer = self.create_timer(0.05, self.update_controller_values)

        connected = False
        while not connected:
            try:
                self.gamepad = InputDevice('/dev/input/event2')
                connected = True
                self.color(255, 0, 0)
            except:
                time.sleep(1)

        self.REL = 127
        
        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0

        self.last_update = time.time()

        self.get_logger().info("Ready...")

    def run(self):
        while True:
            r, w, x = select([self.gamepad.fd], [], [], 0.01)
            if len(r) > 0:
                for event in self.gamepad.read():
                    if event.type == ecodes.EV_ABS:
                        self.handle_joysticks(event)
                    elif event.type == ecodes.EV_KEY:
                        self.handle_buttons(event)
            rclpy.spin_once(self, timeout_sec=0.01)

    def color(self, red, green, blue):
        colors = ['red', 'green', 'blue']
        for color in colors:
            for path in glob(f'/sys/class/leds/0005:054C:*:{color}'):
                if color == 'red': f = open(path + '/brightness', 'a'); f.write(str(red)); f.close()
                if color == 'green': f = open(path + '/brightness', 'a'); f.write(str(green)); f.close()
                if color == 'blue': f = open(path + '/brightness', 'a'); f.write(str(blue)); f.close()

    def handle_joysticks(self, event):
        value = (event.value - self.REL) / self.REL
        if abs(value) < 0.08:
            value = 0.0
        if event.code == 0:
            self.last_x = value
        if event.code == 1:
            self.last_y = -value  # reversed
        if event.code == 3:
            self.last_rot = -value  # reversed
        if event.code == 4:
            pass

    def has_value_changed(self, rot, x, y):
        return self.last_x != x or self.last_y != y or self.last_rot != rot

    def handle_buttons(self, event):
        keyevent = categorize(event)
        if keyevent.keystate == KeyEvent.key_down:
            if keyevent.keycode[0] == 'BTN_A':
                self.enable_motors()
            if keyevent.keycode[0] == 'BTN_WEST':
                self.disable_motors()

    def update_controller_values(self):
        if self.motors_enabled:
            new_direction, velocity = to_polar(self.last_x, self.last_y)
            rotation = self.last_rot
            new_direction = new_direction - math.pi / 2  # the robot has 0 degrees at the front
            msg = self.publish_pose(new_direction, rotation, velocity)
            self.get_logger().debug('"%f %f %f"' % (msg.direction, msg.velocity, msg.rotation))
            self.last_update = time.time()

    def publish_pose(self, new_direction, rotation, velocity):
        msg = ControllerValue()
        msg.direction = float(new_direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.publisher_.publish(msg)
        return msg

    def enable_motors(self):
        self.send_enable_motors(True)

    def disable_motors(self):
        self.send_enable_motors(False)
        
    def send_enable_motors(self, value):
        request = EnableMotors.Request()
        request.enable = value
        enable_motors_future = self.enable_motors_client.call_async(request)
        enable_motors_future.add_done_callback(self.handle_enable_motors_response)
        
    def handle_enable_motors_response(self, future):
        try:
            response = future.result()
            self.motors_enabled = response.enabled
            self.color(0, 255, 0) if self.motors_enabled else self.color(0, 0, 255)
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Motors Enabled' if response.enabled else 'Motors Disabled')


def main(args=None):
    rclpy.init(args=args)

    controller_publisher = PSController()
    
    try:
        controller_publisher.run()
    except KeyboardInterrupt:
        print('Bye')


if __name__ == '__main__':
    main()
