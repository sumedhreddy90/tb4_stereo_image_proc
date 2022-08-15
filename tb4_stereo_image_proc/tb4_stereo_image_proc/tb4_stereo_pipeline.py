import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
import cv2 # OpenCV library
import mediapipe as mp
import numpy as np
from matplotlib import pyplot as plt
from rclpy.qos import qos_profile_sensor_data
import time

class TurtlePerception3D(Node):
    """
    - TurtlePerception3D class inherits from (or is a subclass of) Node attributes as a ROS Node
     that acts as a primary entrypoint in the ROS system for communication. It can be used to 
     create ROS entities such as publishers, subscribers, services, etc.
    - This class is an implementation of the 3D object detection using openCV and MediaPipe.
        Here, we use inbuilt opencv methods such as
         - TODO
    Attributes:
        Node: Is a class from rclpy.node.Node
            More Information at: https://docs.ros2.org/latest/api/rclpy/api/node.html 
    Topics:
        - Publishers: None
        - Subscribers:
                - Topic name: /color/image
                    - Topic type: sensor_msgs.msg/Image
                    - Topic desciption: To subcribe image frames from OKA-D camera sensor of the TB4
    """
    def __init__(self):
        """
        TurtlePerception3D class constructor to initialize nodes,
        subscribers, publishers and parameters
        """
        super().__init__('tb4_perception_3d')

        self.right_cam_info = self.create_subscription(DisparityImage,'/disparity',
                                                self.disparity_callback, qos_profile=qos_profile_sensor_data)
        # Initialize bridge between ROS2 and OpenCV
        self.bridge = CvBridge()
        self.left_info = CameraInfo()
        self.right_info = CameraInfo()

    def disparity_callback(self, disparity_data):
        """
        TurtlePerception3D class constructor to initialize nodes,
        subscribers, publishers and parameters
        Args: frames_data
        """
        try:

            
            self.get_logger().info('Receiving Turtlebot4 Disparity Images')

            # current frame
            current_frame = self.bridge.imgmsg_to_cv2(disparity_data)
            gray_current_frame = current_frame.copy()
            resized_image = cv2.resize(current_frame, (720, 480))

            # Poping each and every frame
            cv2.imshow("TurtleBot4 Left Camera View", resized_image)

            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)


def main(args=None):
    """
    Main method to instantiate ROS nodes and TurtlePerception3D class to publish image
    processed object detection
    Args:
        None
    """
    rclpy.init(args=args)
    turtle_perception_3d = TurtlePerception3D()
    rclpy.spin(turtle_perception_3d)
    turtle_perception_3d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()