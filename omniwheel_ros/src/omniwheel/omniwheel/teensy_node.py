import rclpy
import numpy as np
from rclpy.node import Node

from omniwheel_interfaces.msg import ControllerValue, Pose, MotorState
from omniwheel_interfaces.srv import EnableMotors, DriveConfig, SetPose

from serial import Serial

import time


class TeensyNode(Node):
    """
    This class represents the interface between the ROS graph and the Teensy driving the stepper motors, which is
     connected via a serial connection.

    ROS subscriber:
        - /controller_value
    ROS publisher:
        - /omniwheel_pose
    ROS service servers:
        - /enable_motors
        - /drive_config
    """
    
    def __init__(self):
        super().__init__('teensy_node')
        self.subscription = self.create_subscription(ControllerValue, 'controller_value', self.controller_callback, 10)
        self.enable_service = self.create_service(EnableMotors, 'enable_motors', self.enable_motors_callback)
        self.config_service = self.create_service(DriveConfig, 'drive_config', self.drive_config_callback)
        self.position_service = self.create_service(SetPose, 'set_position', self.set_position_callback)
        self.odometry = self.create_publisher(Pose, 'omniwheel_pose', 10)
        self.enable_publisher = self.create_publisher(MotorState, 'motor_state', 10)

        self.ser = Serial('/dev/ttyACM0', 4000000)

        self.motors_enabled = False
        self.velocity = self.getTeensyVelocity()
        self.acceleration = self.getTeensyAcceleration()
        self.microsteps = self.getTeensyMicrosteps()
        self.position = np.zeros(2)
        self.orientation = 0
        self.odometry_timer = self.create_timer(0.05, self.timer_callback)

        self.last_twist_command = time.time()
        
        self.MOTOR_REVS_PER_METER = 47.5
        self.RADIUS = 0.135

        self.get_logger().info("Ready...")

    def timer_callback(self):
        message = Pose()
        message.x, message.y, message.rot = float(self.position[0]), float(self.position[1]), float(self.orientation)
        self.odometry.publish(message)
        if time.time() - self.last_twist_command > 0.1 and self.motors_enabled:
            self.soft_stop()

    def soft_stop(self):
        commandString = '{I;0.0;0.0;0.0;}'
        self.ser.write(commandString.encode())
        self.get_logger().info("Soft Stopped")
        
    def enable_motors_callback(self, request, response):
        if request.enable:
            self.ser.write(b'{I}')
            response.enabled = True
            self.motors_enabled = True
        else:
            self.ser.write(b'{E}')
            self.motors_enabled = False
            response.enabled = False

        message = MotorState()
        message.enabled = response.enabled
        self.enable_publisher.publish(message)
        
        self.get_logger().info('Motors Enabled' if response.enabled else 'Motors Disabled')

        return response
        
    def drive_config_callback(self, request, response):
        if request.acceleration <= 0:
            response.acceleration = self.acceleration
        else:
            acc = round(request.acceleration, 2)
            command = ("{A;%f}" % acc).encode()
            self.get_logger().info(command)
            #self.ser.write(command)
            self.acceleration = acc
        
        if request.velocity <= 0:
            response.velocity = self.velocity
        else:
            vel = round(request.velocity, 2)
            command = ("{S;%f}" % vel).encode()
            self.get_logger().info(command)
            #self.ser.write(command)
            self.velocity = vel
        
        if request.microsteps <= 0 or request.microsteps not in self.valid_microsteps:
            response.microsteps = self.microsteps
        else:
            command = ("{M;%d}" % request.microsteps).encode()
            self.get_logger().info(command)
            #self.ser.write(command)
            self.microsteps = request.microsteps
    
        return response

    def controller_callback(self, msg):
        self.get_logger().debug('"%f %f %f"' % (msg.direction, msg.velocity, msg.rotation))
        if self.motors_enabled:
            commandString = '{I;' + str(round(msg.direction, 2)) + \
                                ';' + str(round(msg.velocity, 2)) + \
                                ';' + str(round(msg.rotation, 2)) + ';}'
            self.ser.write(commandString.encode())
            self.last_twist_command = time.time()

    def set_position_callback(self, request, response):
        self.position = np.array((request.pose.x, request.pose.y))
        self.orientation = request.pose.rot
        response.pose.x, response.pose.y, response.pose.rot = request.pose.x, request.pose.y, request.pose.rot
        return response
        
    def getTeensyVelocity(self):
        self.ser.write(b'{s}')
        while not self.ser.inWaiting():
            pass
        try:
            value = float(self.ser.readline())
        except:
            self.ser.write(b'{E}')
            value = self.getTeensyVelocity()
        return value
        
    def getTeensyAcceleration(self):
        self.ser.write(b'{a}')
        while not self.ser.inWaiting():
            pass
        return float(self.ser.readline())
    
    def getTeensyMicrosteps(self):
        self.ser.write(b'{m}')
        while not self.ser.inWaiting():
            pass
        return int(self.ser.readline())

    def checkSerial(self):
        while self.ser.inWaiting():
            line = self.ser.readline().decode().strip()
            if line.startswith("{"):
                line = line[1:len(line) - 1]
                steps = line.split(";")
                self.updatePosition(steps)
            else:
                self.get_logger().info(line)

    def updatePosition(self, steps):
        steps = [steps[0], steps[1], steps[2]]
        revs = np.array([int(step_strings.strip().replace("{", "").replace("}", "")) / (200 * self.microsteps) for step_strings in steps])
        dists = revs / self.MOTOR_REVS_PER_METER
        va, vb, vc = dists[0], dists[1], dists[2]
        omega = np.sum(dists) / (3 * self.RADIUS)
        alpha = - ((np.pi / 2 - np.arctan2((np.sqrt(3) * (va - vb)), (va + vb - 2*vc))) + np.pi)
        dist = np.sqrt(((va + vb - 2 * vc) / 3)**2 + (va - vb)**2 / 3)
        self.orientation += omega
        if not np.isnan(alpha) and (dist > 0 or omega > 0):
            dx = dist * np.cos(alpha + self.orientation + np.pi / 2)
            dy = dist * np.sin(alpha + self.orientation + np.pi / 2)
            self.position += np.array([dx, dy])


def main(args=None):
    rclpy.init(args=args)

    controller_subscriber = TeensyNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(controller_subscriber, timeout_sec=0.02)
            controller_subscriber.checkSerial()
    except KeyboardInterrupt:
        print('Bye')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
