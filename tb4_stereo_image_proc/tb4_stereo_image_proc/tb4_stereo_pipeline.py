import numpy as np 
import math 
import argparse
import rclpy 
from matplotlib.pyplot import get
from time import sleep 
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist            
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2 # OpenCV library

class TurtleStereoProcess(Node):
    """
    The class TurtleStereoProcess, attributes as a ROS Node that acts as a primary entrypoint
        in the ROS system for communication especially for publishing left and right camera frame
        to stereo_image_proc package.
    Attributes:
        Node: Is a class from rclpy.node.Node
            More Information at: https://docs.ros2.org/latest/api/rclpy/api/node.html 
    Topics:
        - Publishers:
                - Topic name:
                    - Topic type:
                    - Topic desciption:
    """
    def __init__(self):
        """
        Turtle goal constructor to initialize nodes, subscribers, publishers and parameters
        """
        super().__init__('turtlebot4_stereo_processing')
        self.publisher_left_camera = self.create_publisher(Image,'/color/left/image', 10)
        self.publisher_right_camera = self.create_publisher(Image,'/color/left/image', 10)
        # Initialize bridge between ROS2 and OpenCV
        self.bridge = CvBridge()


        

def main(args=None):
    """
    Main method to instantiate ROS nodes and TurtleStereoProcess class to publish
    goal pose
    Args:
        None
    """
    rclpy.init(args=args)
    turtlebot4_stereo = TurtleStereoProcess()
    rclpy.spin(turtlebot4_stereo)
    turtlebot4_stereo.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()