
import pygame

from omniwheel.path_visualizer.renderer import Renderer
from omniwheel.path_visualizer.domain.robot import Robot
from omniwheel.path_visualizer.event_handler import EventHandler

import rclpy
from rclpy.node import Node


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.robot = Robot(self)
        self.renderer = Renderer(1400, 1000)
        self.event_handler = EventHandler(self, self.robot, self.renderer, lambda: self.stop())

        self.clock = pygame.time.Clock()
        self.running = True

    def tick(self):
        self.renderer.render(self.robot)
        self.event_handler.handle_events()
        self.clock.tick(20)

    def stop(self):
        self.running = False


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
