# ROS2 for Python
import rclpy
from rclpy.node import Node
# Containers for publishing 
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class ColourDetector():
    def __init__(self, node):
        self.bridge = CvBridge()
        # Create instances of subscriber and publisher
        self.image_sub = node.create_subscription(Image, 'camera/image_raw', self.image_callback)
        self.actuator_pub = node.create_publisher(Bool, '/actuator_enabled')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_pixels = cv2.countNonZero(mask)

        actuator_enabled = False
        if blue_pixels > 500:
            actuator_enabled = True
        
        self.actuator_pub.publish(actuator_enabled)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('image_processor')
    image_processor = ColourDetector(node)
    rclpy.spin(node)

if __name__ == '__main__':
    main()




