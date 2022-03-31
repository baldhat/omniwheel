import time

import pygame
import math
import numpy as np

from omniwheel.path_visualizer.renderer import Renderer
from omniwheel.helper.helper import to_polar
from omniwheel.path_visualizer.domain.robot import Robot

import rclpy
from rclpy.node import Node

from omniwheel_interfaces.msg import ControllerValue


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.robot = Robot(self)

        self.controller_publisher = self.create_publisher(ControllerValue, 'controller_value', 10)

        self.renderer = Renderer()
        self.clock = pygame.time.Clock()
        self.running = True

        self.last_x = 0
        self.last_y = 0
        self.last_rot = 0
        self.last_update = 0.0

        self.move_keys = [pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d, pygame.K_q, pygame.K_e]

        self.shift_down = False
        self.middle_mouse_down = False

    def tick(self):
        self.renderer.render(self.robot)
        self.handle_events()
        self.clock.tick(20)

    def handle_events(self):
        x, y, rot = self.last_x, self.last_y, self.last_rot
        for event in pygame.event.get():
            if event.type == pygame.KEYUP:
                self.handleKeyUp(event)
            if event.type == pygame.KEYDOWN:
                self.handleKeyDown(event)
            if event.type == pygame.MOUSEWHEEL:
                self.renderer.zoom(event.y)
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.handleMouseDown(event)
            if event.type == pygame.MOUSEBUTTONUP:
                self.handleMouseUp(event)
            if event.type == pygame.MOUSEMOTION:
                self.handleMouseMotion(event)
            if event.type == pygame.QUIT:
                self.running = False
        if self.shouldUpdateControllerValue(rot, x, y):
            self.update()

    def shouldUpdateControllerValue(self, rot, x, y):
        return self.hasValueChanged(rot, x, y) or (
                    time.time() - self.last_update > 0.05 and (rot != 0 or x != 0 or y != 0))

    def hasValueChanged(self, rot, x, y):
        return self.last_x != x or self.last_y != y or self.last_rot != rot

    def handleKeyDown(self, event):
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
            self.robot.switch_motor_enabled()
        if event.key == pygame.K_c:
            self.robot.resetPosition()

    def handleKeyUp(self, event):
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
        if event.key == pygame.K_LSHIFT:
            self.robot.send_planned_waypoints()

    def handleMouseDown(self, event):
        if event.button == 2:  # middle mouse button
            self.middle_mouse_down = True
        if event.button == 1:
            self.robot.add_waypoint(self.renderer.to_real_pos(event.pos), not pygame.key.get_pressed()[pygame.K_LSHIFT])

    def handleMouseUp(self, event):
        if event.button == 2:  # middle mouse button
            self.middle_mouse_down = False

    def handleMouseMotion(self, event):
        if self.middle_mouse_down:
            self.renderer.display_offset += np.array((event.rel[0], event.rel[1]))

    def update(self):
        new_direction, rotation, velocity = self.calculateControllerValue()
        self.publishControllerValue(new_direction, rotation, velocity)
        self.last_update = time.time()

    def calculateControllerValue(self):
        new_direction, velocity = to_polar(self.last_x, self.last_y)
        rotation = self.last_rot
        new_direction = new_direction - math.pi / 2  # the robot has 0 degrees at the front
        return new_direction, rotation, velocity

    def publishControllerValue(self, new_direction, rotation, velocity):
        msg = ControllerValue()
        msg.direction = float(new_direction)
        msg.velocity = float(velocity)
        msg.rotation = float(rotation)
        self.controller_publisher.publish(msg)
        self.get_logger().debug('"%f %f %f"' % (msg.direction, msg.velocity, msg.rotation))


def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()

    try:
        while rclpy.ok() and path_visualizer.running:
            rclpy.spin_once(path_visualizer, timeout_sec=0)
            path_visualizer.tick()
    except KeyboardInterrupt:
        print('Bye')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
