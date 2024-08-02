#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64

import random as rand
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import math

class ImageSubscriber(Node):
    def __init__(self):
        self.angle = 0
        self.desired_heading = 0
        self.center = 320
        self.timer_bool = 1
        self.tag_id_list = {10, 3, 8, 25} #6 doesn't work #set not list, faster to search but not indexed
        self.at_detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
        
        super().__init__("image_subscriber")
        
        self.publisher_timer = self.create_timer(3.0, self.movements)
        self.get_logger().info("starting timer")
        
        # self.timer_period = 0.5  # seconds
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.cvb = CvBridge()

        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )
        self.get_logger().info('starting camera subscriber')

        self.heading_publisher = self.create_publisher(
            Int16,
            'bluerov2/desired_heading',
            10
        )

        self.movement_publisher = self.create_publisher(
            Float64,
            'bluerov2/x',
            10
        )
        self.get_logger().info('starting random movement publisher')

        self.heading_subscriber = self.create_subscription(
            Int16,
            'bluerov2/heading',
            self.heading_callback,
            10
        )
        
        self.light_pub = self.create_publisher(
            Int16, "bluerov2/lights", 10
        )
        
        self.get_logger().info('starting nodes')
        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )
        #self.get_logger().info('getting camera data')

    def tag_in_list(self, ids, tags):
        tag_bool = True
        for tag in tags:
            if tag.tag_id not in ids:
                tag_bool = False
        return tag_bool
                

    def angles(self):
        msg = Int16()
        d = rand.randint(-2, 2) #(1,4)
        msg.data = (d * 45 + self.angle) #90
        self.heading_publisher.publish(msg)
        time.sleep(1) #2

    def movements(self):
        if self.timer_bool == 2:
            msg = Float64()
            i = rand.randint(-1,1)
            i *= 50.0
            msg.data = i
            self.movement_publisher.publish(msg)
            time.sleep(2) #none
            self.angles()
            #self.timer_bool = 3
        else:
            return

    def heading_callback(self, msg):
        self.angle = msg.data
    
    def image_callback(self, msg: Image):
        """
        Callback function for the image subscriber.
        It receives an image message and saves it.

        Args:
            msg (Image): The image message
        """
        # Convert Image message to OpenCV image
        image = self.cvb.imgmsg_to_cv2(msg)

        # Save the image
        cv2.imwrite("image.png", image)
        
        # Apriltag stuff below
        img = cv2.imread("image.png", cv2.IMREAD_GRAYSCALE)
        
        camera_params = (1061, 1061, 2000/2, 1121/2) #fx, fy, cx, cy

        tags = self.at_detector.detect(img, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.1)
        color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        
        if len(tags) > 0 and self.tag_in_list(self.tag_id_list, tags) == True:
            self.timer_bool = 1
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

                cv2.putText(color_img, str(tag.tag_id),
                            org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.8,
                            color=(0, 0, 255))
                
                translation = tag.pose_t #translation vector/position of tag in camera frame (x,y,z axes)
                distance = tag.pose_t[2]/2.8 #z-axis for distance
                tag_id = tag.tag_id
                self.center = tag.center #x,y coordinates of center of tag detected

                if self.center[0] > 400:
                    # turning right if the apriltag is on the right
                    self.desired_heading = self.angle + (self.center[0]-400)*3/8 
                elif self.center[0] < 240:
                    # turning left if the apriltag is on the right
                    self.desired_heading = self.angle - (240 - self.center[0])*3/8
                else:
                    self.desired_heading = self.angle
                    
                    # moving forward when an apriltag is detected
                    msg = Float64()
                    msg.data = 50.0
                    self.movement_publisher.publish(msg)
                    
                    if distance <= 2.2:
                        msg = Int16()
                        msg.data = 0
                        self.light_pub.publish(msg)
                        # shine lights by setting msg data to 0
                    else:
                        msg = Int16()
                        msg.data = 1
                        self.light_pub.publish(msg)
                        # turn off lights by setting msg data to 1

                msg = Int16()
                msg.data = int(self.desired_heading)
                self.heading_publisher.publish(msg)
            
                self.get_logger().info(f"Tag ID: {tag_id}")
                self.get_logger().info(f"Center: {self.center}")
                self.get_logger().info(f"Translation: {translation}")
                self.get_logger().info(f"Distance: {distance}")
                self.get_logger().info("---")
            
            # Going forward when detecting apriltags

            
        else:
            msg = Int16()
            msg.data = 1
            self.light_pub.publish(msg)
            #if self.timer_bool != 3:
            self.timer_bool = 2

            cv2.imwrite("detected.png", color_img)



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()