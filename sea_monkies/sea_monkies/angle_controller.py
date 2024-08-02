import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ManualControl, Altitude
from std_msgs.msg import Int16
from std_msgs.msg import Float64

import numpy as np
import math
import matplotlib.pyplot as plt


class PIDHeadingNode(Node):
    def __init__(self):
        super().__init__('heading_node')
        #subscribers/publishers for necessary topics
        self.move_publisher = self.create_publisher(
            Float64,
            'bluerov2/r',
            10
        )
        self.heading_subscriber = self.create_subscription(
            Int16,
            'bluerov2/heading',
            self.heading_callback,
            10
        )

        self.desired_heading_subscriber = self.create_subscription(
            Int16,
            'bluerov2/desired_heading',
            self.desired_heading_callback,
            10
        )
        self.heading_derivative_subscriber = self.create_subscription(
            Imu,
            'bluerov2/imu',
            self.heading_derivative_callback,
            10
        )
        """
        PID CONSTANTS
        """
        self.kp = 2
        self.kd = 1
        self.min_output = -20.0 #-30
        self.max_output = 20.0 #30
        self.previous_error = 0.0
        """"""

        self.get_logger().info('starting publisher node')
        #self.pid_yaw = PIDController(0.5, 0.1, 0.05, 1.0, -50, 50)
        """
        tracking constants
        """
        self.heading = None
        self.desired_heading = None
        self.heading_derivative = 0.0      
        self.array = np.array([])


    def compute(self, error):
        """
        computes and logs the correction power based on the angle error and angular velocity in rad/s as derivative
        """
        #tracking
        self.array = np.append(self.array, [(error)])

        #p and d calcs and return output
        self.derivative = self.heading_derivative
        proportional = self.kp * error
        output = proportional + (self.kd * self.derivative)

        #more tracking
        self.get_logger().info(f'\n Kp: {proportional} Kd: {self.kd *self.derivative} CurrentHeading: {self.heading}')
        
        #updates/clamping output
        output = max(min(output, self.max_output), self.min_output)
        self.previous_error = error
        return output

    def heading_callback(self, msg):
        """logs and stores int16 heading from subscriber"""
        self.heading = msg.data
        if self.desired_heading != None:
            self.calc_publish_heading()
        #self.get_logger().info(f'Depth: {self.depth}, Timestamp: {self.timestamp}')


    def desired_heading_callback(self, msg):
        """logs and stores desired heading from publisher in int16 and converts degrees to rads"""
        self.desired_heading = msg.data
        self.desired_heading = self.desired_heading

        
    def heading_derivative_callback(self, msg):
        """logs and stores angular velocity in deg/s from imu"""
        self.heading_derivative = msg.angular_velocity.z * 180/math.pi
    def calc_publish_heading(self):
        """calculates angular error using mod function and receives/publishes correction to manual control"""
        #checks to make sure that a heading has been recieved
        if self.heading is not None:
            #error calc
            error1 = ((self.desired_heading - self.heading + 180) % 360 - 180)/1.8
           # error2 = math.abs(100*math.sin(math.pi/180*x))*math.sin(x*math.pi/180)/math.abs(math.sin(x*math.pi/180))
            heading_correction = self.compute(error1)

            #publishing movement
            movement = Float64()

            movement.data = heading_correction
            self.get_logger().info(f'\nCurrent Power: {heading_correction}/100\nHeading: {self.heading}')
            self.move_publisher.publish(movement)


def main(args=None):
    rclpy.init(args=args)
    move_node = PIDHeadingNode()
    try:
        rclpy.spin(move_node)
    except KeyboardInterrupt:
        print('\nKeyboardInterrupt received, shutting down...')
    finally:
        move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()




"""
- node:
    pkg: "intro_to_ros"
    exec: "pid"
    name: "pid"
    namespace: ""
"""