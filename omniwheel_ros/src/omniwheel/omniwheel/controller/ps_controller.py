import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

import math
import time

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
        
        pygame.init()
        pygame.display.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        
        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0

        self.last_update = time.time()

        self.get_logger().info("Ready...")

    def run(self):
        x = self.last_x
        y = self.last_y
        rot = self.last_rot
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.handle_joysticks(event)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.handle_buttons(event)
        if self.should_update_controller_value(rot, x, y):
            self.update()

    def should_update_controller_value(self, rot, x, y):
        return self.has_value_changed(rot, x, y) or (
                    time.time() - self.last_update > 0.05 and (x != 0 or y != 0 or rot != 0))

    def handle_joysticks(self, event):
        if event.axis == 0:  # left joystick horizontal
            self.last_x = event.value if abs(event.value) > 0.1 else 0
        if event.axis == 1:  # left joystick vertical
            self.last_y = - event.value if abs(event.value) > 0.1 else 0
        if event.axis == 3:  # right joystick horizontal
            self.last_rot = - event.value if abs(event.value) > 0.1 else 0
        if event.axis == 4:  # right joystick vertical
            pass

    def has_value_changed(self, rot, x, y):
        return self.last_x != x or self.last_y != y or self.last_rot != rot

    def handle_buttons(self, event):
        if event.button == 0:
            self.enable_motors()
        elif event.button == 3:
            self.disable_motors()
        else:
            self.get_logger().info(event.button)

    def update(self):
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot
        new_direction = new_direction - math.pi / 2  # the robot has 0 degrees at the front

        if self.motors_enabled:
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
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Motors Enabled' if response.enabled else 'Motors Disabled')


def main(args=None):
    rclpy.init(args=args)

    controller_publisher = PSController()
    
    try:
        while rclpy.ok():
            controller_publisher.run()
            rclpy.spin_once(controller_publisher, timeout_sec=0.04)
    except KeyboardInterrupt:
        print('Bye')


if __name__ == '__main__':
    main()
