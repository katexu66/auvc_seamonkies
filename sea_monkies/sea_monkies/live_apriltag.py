#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16

import cv2
import numpy as np
import matplotlib.pyplot as plt
from dt_apriltags import Detector

class ImageSubscriber(Node):
    def __init__(self):
        self.angle = 0
        self.desired_heading = 0
        self.center = 320
        self.at_detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
        
        super().__init__("image_subscriber")
        
        # self.timer_period = 0.5  # seconds
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.cvb = CvBridge()

        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )
        self.get_logger().info('starting camera subscriber')

        self.publisher = self.create_publisher(
            Int16,
            'bluerov2/desired_heading',
            10
        )

        self.heading_subscriber = self.create_subscription(
            Int16,
            'bluerov2/heading',
            self.heading_callback,
            10
        )
        
        self.get_logger().info('starting nodes')
        self.subscription = self.create_subscription(
            Image, "bluerov2/camera", self.image_callback, 10
        )
        #self.get_logger().info('getting camera data')

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
        
        if len(tags) > 0:
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
                #rotation = tag.pose_R
                tag_id = tag.tag_id
                #corners = tag.corners #x,y coordinates of 4 corners detected
                self.center = tag.center #x,y coordinates of center of tag detected

                if self.center[0] > 400:
                    self.desired_heading = self.angle + self.center[0]-400*3/8
                elif self.center[0] < 240:
                    self.desired_heading = self.angle - self.center[0]-400*3/8
                else:
                    self.desired_heading = self.angle

                msg = Int16()
                msg.data = int(self.desired_heading)
                self.publisher.publish(msg)
            
                self.get_logger().info(f"Tag ID: {tag_id}")
                #self.get_logger().info(f"Corners: {corners}")
                self.get_logger().info(f"Center: {self.center}")
                self.get_logger().info(f"Translation: {translation}")
                self.get_logger().info(f"Distance: {distance}")
                #self.get_logger().info(f"Rotation: {rotation}")
                self.get_logger().info("---")

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