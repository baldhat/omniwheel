from evdev import InputDevice, categorize, ecodes, KeyEvent

import math
import time
from select import select
from glob import glob

import rclpy
from rclpy.node import Node

from omniwheel_interfaces.msg import ControllerValue, MotorState
from omniwheel_interfaces.srv import EnableMotors

from omniwheel.helper.helper import to_polar


def color(red, green, blue):
    """ Set the color of the PS4 Controller to the given RGB value.

        For this to work without root permission problems, the udev rules in the udev directory need to be copied
        to the /etc/udev/rules.d directory and the user needs to be added to the input group. For these changes
        to take effect, the user might need to logout and login again.
    """
    colors = ['red', 'green', 'blue']
    for color in colors:
        for path in glob(f'/sys/class/leds/0005:054C:*:{color}'):
            if color == 'red': f = open(path + '/brightness', 'a'); f.write(str(red)); f.close()
            if color == 'green': f = open(path + '/brightness', 'a'); f.write(str(green)); f.close()
            if color == 'blue': f = open(path + '/brightness', 'a'); f.write(str(blue)); f.close()


def show_connected():
    """ Show that the controller was connected by flashing turquoise for one second """
    color(0, 255, 255)
    time.sleep(1)
    color(255, 0, 0)


class PSController(Node):
    """ Publishes the command read from a PS4 Controller to the controller_value topic.

        Publishers:
            - controller_value
        Subscribers:
            - motor_state
        Service clients:
            - enable_motors
    """
    def __init__(self):
        super().__init__('controller_publisher')
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.create_subscription(MotorState, 'motor_state', self.motor_state_callback, 10)
        self.controller_value_timer = self.create_timer(0.05, self.update_controller_values)

        # Flag showing if the motors of the robot are currently enabled. This value is updated whenever a motor_state
        # message is received.
        self.motors_enabled = False

        # Flag showing if the most recently sent message to the controller_value topic consisted of all zeros.
        # This is used to determine whether another such message should be sent.
        self.last_sent_zeros = False

        # Wait for the PS4 Controller to be connected
        connected = False
        while not connected:
            try:
                self.gamepad = InputDevice('/dev/input/event2')
                connected = True
                show_connected()
            except:
                time.sleep(1)

        # The value the controller returns when the stick is in the middle position
        self.JOYSTICK_ZERO_POINT = 127

        # The last read values of the controller joysticks
        self.controller_x = 0
        self.controller_y = 0
        self.controller_rotation = 0

        self.get_logger().info("Ready...")

    def run(self):
        """ Constantly check for controller changes and spin the ROS node.
        """
        while True:
            r, w, x = select([self.gamepad.fd], [], [], 0.01)
            if len(r) > 0:
                for event in self.gamepad.read():
                    if event.type == ecodes.EV_ABS:
                        self.handle_joysticks(event)
                    elif event.type == ecodes.EV_KEY:
                        self.get_logger().info("button press")
                        self.handle_buttons(event)
            rclpy.spin_once(self, timeout_sec=0.01)

    def handle_joysticks(self, event):
        """ Handle controller events corresponding to joystick movements.
            Sets the values of the three monitored axes.
            Smaller values around the zero-point are considered noise.
        """
        value = (event.value - self.JOYSTICK_ZERO_POINT) / self.JOYSTICK_ZERO_POINT
        if abs(value) < 0.08:
            value = 0.0
        if event.code == 0:  # left stick horizontal
            self.controller_x = value
        if event.code == 1:  # left stick  vertical
            self.controller_y = -value  # reversed
        if event.code == 3:  # right stick horizontal
            self.controller_rotation = -value  # reversed

    def handle_buttons(self, event):
        """ Handle controller events corresponding to button presses.
            Only act on the key_down event to prevent handling the button push twice.
        """
        key_event = categorize(event)
        if key_event.keystate == KeyEvent.key_down:
            if key_event.keycode[0] == 'BTN_A':
                self.get_logger().info("Button press a")
                self.send_enable_motors(True)
            if key_event.keycode[0] == 'BTN_WEST':
                self.get_logger().info("Button press west")
                self.send_enable_motors(False)

    def update_controller_values(self):
        """ Publish the controller_value, if the motors are enabled and we have not already sent a command with all
            zeros. This is to prevent constantly sending zero commands and interfering if another node enabled the
            motors and is sending their own controller_values.
        """
        if self.motors_enabled and \
                not self.would_send_second_all_zeros():
            self.check_zeros()
            new_direction, velocity = to_polar(self.controller_x, self.controller_y)
            new_direction = new_direction - math.pi / 2  # the robot has 0 degrees at the front
            self.publish_controller_values(new_direction, self.controller_rotation, velocity)

    def would_send_second_all_zeros(self):
        """ Return whether we would send all zero controller values for the second time.
            This is the case, if we already sent zeros and the current controller values are all zero.
        """
        return (self.last_sent_zeros
                and abs(self.controller_x) <= 0.1 and abs(self.controller_y) <= 0.1 and abs(self.controller_rotation) <= 0.1)

    def check_zeros(self):
        """ Check whether the current controller values are considered zeros.
            If so, set the flag showing that we are sending all zero values.
        """
        if self.controller_x <= 0.1 and self.controller_y <= 0.1 and self.controller_y <= 0.1:
            self.last_sent_zeros = True
        else:
            self.last_sent_zeros = False

    def publish_controller_values(self, new_direction, rotation, velocity):
        """ Create a controller_value message and publish it to the correspodning topic.
        """
        msg = ControllerValue()
        msg.direction = float(new_direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.publisher_.publish(msg)
        return msg

    def motor_state_callback(self, msg):
        """ Called when receiving a motor_state message.
            Set the motors_enabled flag to the corresponding value and change the color of the controller RGB LEDs.
        """
        self.motors_enabled = msg.enabled
        color(0, 255, 0) if self.motors_enabled else color(255, 0, 0)

    def send_enable_motors(self, value):
        """ Call the EnableMotors Service to enable or disable the motors of the robot.
        """
        request = EnableMotors.Request()
        request.enable = value
        enable_motors_future = self.enable_motors_client.call_async(request)
        enable_motors_future.add_done_callback(self.handle_enable_motors_response)

    def handle_enable_motors_response(self, future):
        """ Callback for the enable_motor service.
            Set the motors_enabled flag to the corresponding value and change the color of the controller RGB LEDs.
        """
        try:
            response = future.result()
            self.motors_enabled = response.enabled
            color(0, 255, 0) if self.motors_enabled else color(255, 0, 0)
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    PSController().run()


if __name__ == '__main__':
    main()
