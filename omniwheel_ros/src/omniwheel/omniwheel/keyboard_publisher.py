import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

import math
import cmath
import rclpy
from rclpy.node import Node

from omniwheel_interfaces.msg import ControllerValue
from omniwheel_interfaces.srv import EnableMotors


def to_polar(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    if r > 1:
        r = 1
    t = cmath.polar(x + y * 1j)[1]
    return t, r

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(ControllerValue, 'controller_value', 10)
        self.enable_motors_client = self.create_client(EnableMotors, 'enable_motors')
        self.enable_motors_future = None
        self.motors_enabled = False

        pygame.init()
        pygame.display.init()
        self.WIDTH = 1400
        self.HEIGHT = 1000
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (1920 - 1405, 40)
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))

        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0

    def run(self):
        x = self.last_x
        y = self.last_y
        rot = self.last_rot
        for event in pygame.event.get():
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_q:
                    self.last_rot = 0
                if event.key == pygame.K_e:
                    self.last_rot = 0
                if event.key == pygame.K_w:
                    self.last_y = 0
                if event.key == pygame.K_a:
                    self.last_x = 0
                if event.key == pygame.K_s:
                    self.last_y = 0
                if event.key == pygame.K_d:
                    self.last_x = 0
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.last_rot = 1
                if event.key == pygame.K_e:
                    self.last_rot = -1
                if event.key == pygame.K_w:
                    self.last_y = 1
                if event.key == pygame.K_a:
                    self.last_x = -1
                if event.key == pygame.K_s:
                    self.last_y = -1
                if event.key == pygame.K_d:
                    self.last_x = 1
                if event.key == pygame.K_SPACE:
                    self.send_enable_motors(not self.motors_enabled)
        if self.last_x != x or self.last_y != y or self.last_rot != rot:
            self.update()

    def update(self):
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot
        new_direction = new_direction - math.pi / 2  # the robot has 0 degrees at the front

        if self.motors_enabled:
            msg = ControllerValue()
            msg.direction = float(new_direction)
            msg.velocity = float(velocity)
            msg.rotation = float(rotation)
            self.publisher_.publish(msg)
            self.get_logger().debug('"%f %f %f"' % (msg.direction, msg.velocity, msg.rotation))

    def send_enable_motors(self, value):
        request = EnableMotors.Request()
        request.enable = value
        self.enable_motors_future = self.enable_motors_client.call_async(request)

    def handle_enable_motors_response(self):
        try:
            response = self.enable_motors_future.result()
            self.motors_enabled = response.enabled
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Motors Enabled' if response.enabled else 'Motors Disabled')
        self.enable_motors_future = None


def main(args=None):
    rclpy.init(args=args)

    keyboard_publisher = KeyboardPublisher()

    try:
        while rclpy.ok():
            keyboard_publisher.run()
            rclpy.spin_once(keyboard_publisher, timeout_sec=0)
            if keyboard_publisher.enable_motors_future is not None and keyboard_publisher.enable_motors_future.done():
                keyboard_publisher.handle_enable_motors_response()
    except KeyboardInterrupt:
        print('Bye')


if __name__ == '__main__':
    main()
