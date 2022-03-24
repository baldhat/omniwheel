import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import math

import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Pose

DARKGRAY = (100, 100, 100)


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.pose_subscription = self.create_subscription(Pose, 'omniwheel_pose', self.pose_callback, 10)
        self.pose_subscription  # avoid warning
        pygame.init()
        pygame.display.init()
        self.WIDTH = 1400
        self.HEIGHT = 1000
        os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (1920 - 1405, 40)
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Omniwheel")
        self.screen.fill(DARKGRAY)
        self.font = pygame.font.Font('freesansbold.ttf', 16)
        self.bigFont = pygame.font.Font('freesansbold.ttf', 20)
        self.clock = pygame.time.Clock()
        self.image = pygame.image.load("src/omniwheel/omniwheel/assets/omniwheel.png")

        self.position = np.zeros(2)
        self.orientation = 0

        self.MAP_SCALE = 100
        self.waypoints = [(0, 0)]

    def pose_callback(self, msg):
        self.orientation = msg.orientation.z
        self.position = (msg.position.x, msg.position.y)
        self.waypoints.append(self.position)

    def render(self):
        self.screen.fill(DARKGRAY)
        self.drawPath()
        self.renderRobot()
        pygame.display.flip()
        self.clock.tick(20)

    def renderRobot(self):
        self.blitRotateCenter(self.image, self.toPixelPos(self.position), self.orientation)

    def toPixelPos(self, position):
        return np.array([position[0], -position[1]]) * self.MAP_SCALE + np.array([self.WIDTH / 2.2, self.HEIGHT / 2])

    def blitRotateCenter(self, image, center, angle):
        rotated_image = pygame.transform.rotate(image, angle * 180 / math.pi)
        new_rect = rotated_image.get_rect(center=image.get_rect(center=center).center)
        self.screen.blit(rotated_image, new_rect)

    def drawPath(self):
        self.waypoints[0] = (0, 0)
        for i, value in enumerate(self.waypoints):
            if i < len(self.waypoints) - 1:
                start = self.toPixelPos(value)
                end = self.toPixelPos(self.waypoints[i + 1])
                pygame.draw.line(self.screen, (0, 0, 255), start, end)

    def resetPosition(self):
        self.position = (0, 0)
        self.waypoints = [(0, 0)]
        self.orientation = 0


def main(args=None):
    rclpy.init(args=args)

    path_visualizer = PathVisualizer()

    try:
        while rclpy.ok():
            rclpy.spin_once(path_visualizer, timeout_sec=0)
            path_visualizer.render()
    except KeyboardInterrupt:
        print('Bye')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
