
import pygame

from omniwheel.path_visualizer.renderer import Renderer
from omniwheel.path_visualizer.domain.robot import Robot
from omniwheel.path_visualizer.event_handler import EventHandler

import rclpy
from rclpy.node import Node


class PathVisualizer(Node):
    """
    Visualizes the omniwheel robot with the help of a renderer and an event handler.
    """
    def __init__(self):
        super().__init__('path_visualizer')

        self.robot = Robot(self)
        self.renderer = Renderer(1400, 1000)
        self.event_handler = EventHandler(self, self.robot, self.renderer, self.stop)

        self.clock = pygame.time.Clock()
        self.running = True

    def run(self):
        """
        As long as the stop handler hasn't been called, spin the ros node, render the robot data and position and
        check for events.
        The frame- and update-rate is locked to 20Hz.
        """
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0)
            self.renderer.render(self.robot)
            self.event_handler.handle_events()
            self.clock.tick(20)

    def stop(self):
        """ Callback for the quit-event of the event handler. """
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()
    path_visualizer.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
