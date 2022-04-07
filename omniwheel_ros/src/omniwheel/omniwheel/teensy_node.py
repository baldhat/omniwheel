import rclpy
import numpy as np
from rclpy.node import Node
import tf_transformations

from omniwheel_interfaces.msg import ControllerValue, MotorState
from omniwheel_interfaces.srv import EnableMotors, DriveConfig, SetPose
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

from serial import Serial

import time


class TeensyNode(Node):
    """
    This class represents the interface between the ROS graph and the Teensy driving the stepper motors, which is
    connected via a serial connection.

    Subscribers:
        - controller_value
    Publishers:
        - wheel_odometry
        - motor_state
        - battery_state
    Service servers:
        - enable_motors
        - drive_config
        - set_pose
    """
    
    def __init__(self):
        super().__init__('teensy_node')
        # topic subscriptions
        self.subscription = self.create_subscription(ControllerValue, 'controller_value', self.controller_callback, 10)
        # service servers
        self.enable_service = self.create_service(EnableMotors, 'enable_motors', self.enable_motors_callback)
        self.config_service = self.create_service(DriveConfig, 'drive_config', self.drive_config_callback)
        self.position_service = self.create_service(SetPose, 'set_pose', self.set_pose_callback)
        # topic publishers
        self.odometry = self.create_publisher(Odometry, '/wheel_odometry', 10)
        self.enable_publisher = self.create_publisher(MotorState, 'motor_state', 10)
        self.battery_publisher = self.create_publisher(BatteryState, 'battery_state', 10)
        # timers
        self.odometry_timer = self.create_timer(0.05, self.odom_timer_callback)
        self.battery_timer = self.create_timer(5, self.battery_timer_callback)

        try:
            self.ser = Serial('/dev/ttyACM0', 4000000)
        except:
            # Sometimes the teensy connects under this name
            self.ser = Serial('/dev/ttyACM1', 4000000)

        # Flag depicting the current status of the motor drivers
        self.motors_enabled = False
        self.max_velocity = self.fetch_velocity()  # The maximum wheel velocity as retrieved from the teensy
        self.acceleration = self.fetch_max_acceleration()  # The maximum wheel acceleration as retrieved from the teensy
        self.micro_steps = self.fetch_micro_steps()  # The maximum wheel velocity as retrieved by the teensy
        self.valid_micro_steps = [1, 2, 4, 8, 16, 32]  # Possible values for the micro step configuration

        self.position = np.zeros(2)  # Current position of the robot in the odom frame
        self.orientation = 0  # Current orientation of the robot in the odom frame
        self.velocity, self.omega = np.array([0, 0]), 0.0  # Current velocities of the robot (linear and angular)

        self.last_twist_command = time.time()  # timestamp of the last time a controller_value msg was received
        
        self.MOTOR_REVS_PER_METER = 47.5  # Constant showing the number of motor revolutions needed to drive one meter
        self.RADIUS = 0.135  # Distance from the center of the robot to the contact points of the wheels

        self.get_logger().info("Ready...")

    def odom_timer_callback(self):
        """ Callback for the odometry timer.
        Publishes the pose and twist of the robot in the wheel_odometry topic.
        Also checks when the last time a controller_value msg was received. If none was received for 100ms, a soft stop
        is undertaken.
        """
        odom_msg = Odometry()
        self.create_odom_msg_header(odom_msg)
        self.create_odom_msg_pose(odom_msg)
        self.create_odom_msg_twist(odom_msg)
        self.odometry.publish(odom_msg)

        if time.time() - self.last_twist_command > 0.1 and self.motors_enabled:
            self.soft_stop()

    def create_odom_msg_twist(self, odom_msg):
        """ Creates the twist part for the Odometry messages """
        odom_msg.twist.twist.linear.x = float(self.velocity[0])
        odom_msg.twist.twist.linear.y = float(self.velocity[1])
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = float(self.omega)
        odom_msg.twist.covariance = np.identity(6).flatten()

    def create_odom_msg_pose(self, odom_msg):
        """ Creates the pose part for the Odometry messages """
        odom_msg.pose.pose.position.x = float(self.position[0])
        odom_msg.pose.pose.position.y = float(self.position[1])
        odom_msg.pose.pose.position.z = 0.0
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.orientation)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        odom_msg.pose.covariance = np.identity(6).flatten()

    def create_odom_msg_header(self, odom_msg):
        """ Creates the header and child_frame parts for the Odometry messages """
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

    def battery_timer_callback(self):
        """ Callback of the battery timer.
        Publishes the BatteryState message with the current battery voltage to the battery_state topic.
        """
        battery_state = BatteryState()
        battery_state.voltage = self.fetch_battery_voltage()
        battery_state.header.stamp = self.get_clock().now().to_msg()
        self.battery_publisher.publish(battery_state)

    def fetch_battery_voltage(self):
        """ Fetches the current battery voltage from the teensy"""
        self.ser.write(b'{b}')
        while not self.ser.inWaiting():
            pass
        line = self.ser.readline().decode().strip()
        if line.startswith('{'):  # If it happens that the teensy was just sending a step update
            self.handle_position_update(line)  # handle the update and try reading again
            line = self.ser.readline()
        value = float(line)
        return value

    def soft_stop(self):
        """ Try to stop the robot by sending a movement command with zero velocities. """
        commandString = '{I;0.0;0.0;0.0;}'
        self.ser.write(commandString.encode())
        self.get_logger().debug("Soft Stopped")
        
    def enable_motors_callback(self, request, response):
        """ Callback for incoming request of the enable_motors service.
        Writes the requested motor state to the teensy and publishes the new motor state to the motor_state topic.
        """
        self.ser.write(b'{I}' if request.enable else b'{E}')

        self.motors_enabled = request.enable
        response.enabled = self.motors_enabled
        self.publish_motor_state(response)

        self.get_logger().info('Motors Enabled' if response.enabled else 'Motors Disabled')
        return response

    def publish_motor_state(self, response):
        """ Creates a MotorState message and publishes it to the motor_state topic. """
        message = MotorState()
        message.enabled = response.enabled
        self.enable_publisher.publish(message)

    def drive_config_callback(self, request, response):
        """ Callback for the DriveConfig service.
        Handles the request by calling the handlers for the three values.
        """
        self.handle_acceleration_change(request, response)
        self.handle_velocity_change(request, response)
        self.handle_micro_step_change(request, response)
        return response

    def handle_micro_step_change(self, request, response):
        """ Handles the micro_step part of the DriveConfig service call.
        Check if the requested value is valid and if so, requests the change with the teensy.
        """
        if request.microsteps > 0 and request.microsteps in self.valid_micro_steps:
            command = ("{M;%d}" % request.microsteps).encode()
            self.get_logger().info(command)
            self.ser.write(command)
            self.micro_steps = request.microsteps
        response.microsteps = self.micro_steps

    def handle_velocity_change(self, request, response):
        """ Handles the velocity part of the DriveConfig service call.
        Check if the requested value is valid and if so, requests the change with the teensy.
        """
        if request.velocity > 0:
            vel = round(request.velocity, 2)
            command = ("{S;%f}" % vel).encode()
            self.get_logger().info(command)
            self.ser.write(command)
            self.max_velocity = vel
        response.velocity = self.max_velocity

    def handle_acceleration_change(self, request, response):
        """ Handles the acceleration part of the DriveConfig service call.
        Check if the requested value is valid and if so, requests the change with the teensy.
        """
        if request.acceleration > 0:
            acc = round(request.acceleration, 2)
            command = ("{A;%f}" % acc).encode()
            self.get_logger().info(command)
            self.ser.write(command)
            self.acceleration = acc

        response.acceleration = self.acceleration

    def controller_callback(self, msg):
        """ Callback for the controller_value subscriber
        Writes the given values to the teensy serial connection.
        """
        self.get_logger().debug('"%f %f %f"' % (msg.direction, msg.velocity, msg.rotation))
        if self.motors_enabled:
            commandString = '{I;' + str(round(msg.direction, 2)) + \
                                ';' + str(round(msg.velocity, 2)) + \
                                ';' + str(round(msg.rotation, 2)) + ';}'
            self.ser.write(commandString.encode())
            self.last_twist_command = time.time()

    def set_pose_callback(self, request, response):
        """ Callback for the set_pose service requests
        Sets the position of the robot to the requested value.
        """
        self.position = np.array((request.pose.x, request.pose.y))
        self.orientation = request.pose.rot
        response.pose.x, response.pose.y, response.pose.rot = request.pose.x, request.pose.y, request.pose.rot
        return response
        
    def fetch_velocity(self):
        """ Fetches the currently set max wheel velocity from the teensy. """
        self.ser.write(b'{s}')
        while not self.ser.inWaiting():
            pass
        try:
            value = float(self.ser.readline())
        except:
            self.ser.write(b'{E}')
            value = self.fetch_velocity()
        return value
        
    def fetch_max_acceleration(self):
        """ Fetches the currently set max wheel acceleration from the teensy. """
        self.ser.write(b'{a}')
        while not self.ser.inWaiting():
            pass
        return float(self.ser.readline())
    
    def fetch_micro_steps(self):
        """ Fetches the currently set micro step configuration from the teensy. """
        self.ser.write(b'{m}')
        while not self.ser.inWaiting():
            pass
        return int(self.ser.readline())

    def checkSerial(self):
        """ Check for a new serial message from the teensy.
        Depending on the message, it either gets logged or is interpreted as a update on the executed motor steps
        """
        while self.ser.inWaiting():
            line = self.ser.readline().decode().strip()
            if line.startswith("{"):
                self.handle_position_update(line)
            else:
                self.get_logger().info(line)

    def handle_position_update(self, line):
        """ Parses the line into a list of integers containing the number of steps executed in the last interval. """
        line = line[1:len(line) - 1]
        steps = line.split(";")
        steps = [steps[0], steps[1], steps[2]]
        steps = np.array([
            int(step_strings.strip().replace("{", "").replace("}", ""))
            for step_strings in steps
        ])
        self.updatePosition(steps)

    def updatePosition(self, steps):
        """ Updates the position of the robot based on the received steps.
        The steps get converted into distances driven by each wheel. The distances are used to determine the
        driven direction, distance and rotation of the robot.
        """
        dists = steps / (200 * self.micro_steps * self.MOTOR_REVS_PER_METER)
        va, vb, vc = dists[0], dists[1], dists[2]
        rotation = np.sum(dists) / (3 * self.RADIUS)
        alpha = - ((np.pi / 2 - np.arctan2((np.sqrt(3) * (va - vb)), (va + vb - 2*vc))) + np.pi)
        dist = np.sqrt(((va + vb - 2 * vc) / 3)**2 + (va - vb)**2 / 3)
        self.orientation += rotation
        self.omega = rotation * 20  # We receive updates every 50ms, so the velocity is distance / 50ms
        if not np.isnan(alpha) and dist > 0:
            dx = dist * np.cos(alpha + self.orientation + np.pi / 2)
            dy = dist * np.sin(alpha + self.orientation + np.pi / 2)
            self.velocity = np.array([dx * 20, dy * 20])  # Same reason as omega
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
