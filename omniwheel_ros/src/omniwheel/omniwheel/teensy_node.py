import rclpy
from rclpy.node import Node

from omniwheel_interfaces.msg import ControllerValue
from omniwheel_interfaces.srv import EnableMotors
from omniwheel_interfaces.srv import DriveConfig

from serial import Serial


class ControllerSubscriber(Node):
    
    def __init__(self):
        super().__init__('teensy_node')
        self.subscription = self.create_subscription(ControllerValue, 'controller_value', self.controller_callback, 10)
        self.enable_service = self.create_service(EnableMotors, 'enable_motors', self.enable_motors_callback)
        self.config_service = self.create_service(DriveConfig, 'drive_config', self.drive_config_callback)
        self.subscription
        self.ser = Serial('/dev/ttyACM0', 4000000)
        
        self.velocity = self.getTeensyVelocity()
        self.acceleration = self.getTeensyAcceleration()
        self.microsteps = self.getTeensyMicrosteps()
        
    def enable_motors_callback(self, request, response):
        if request.enable:
            self.ser.write(b'{I}')
            response.enabled = True
        else:
            self.ser.write(b'{E}')
            response.enabled = False
        
        self.get_logger().info('Motors Enabled' if response.enabled == True  else 'Motors Disabled')

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
        commandString = '{I;' + str(round(msg.direction, 2)) + \
                            ';' + str(round(msg.velocity, 2)) + \
                            ';' + str(round(msg.rotation, 2)) + ';}'
        self.ser.write(commandString.encode())
        
    def getTeensyVelocity(self):
        self.ser.write(b'{s}')
        while not self.ser.inWaiting():
            pass
        return float(self.ser.readline())
        
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

def main(args=None):
    rclpy.init(args=args)

    controller_subscriber = ControllerSubscriber()
    
    try:
        rclpy.spin(controller_subscriber)
    except KeyboardInterrupt:
        print('Bye')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
