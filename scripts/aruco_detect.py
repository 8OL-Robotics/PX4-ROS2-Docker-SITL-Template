# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import argparse
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np

# from ultralytics import YOLO # YOLO library

# Load the YOLOv8 model
# model = YOLO('yolov8m.pt')


class ImageSubscriber(Node,):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_subscriber")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
        self.float_arr_pub = self.create_publisher(Float32MultiArray, "float_arr", 10)
        #bool publish 
        self.visibility_pub = self.create_publisher(Bool, "marker_visibility", 10)

        # check if recording is enables in env         
        self.image_pub = self.create_publisher(Image, "processed_image", 10)

        
        # Create a parameter object for marker detection
        self.parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict)
        


        # Set the parameters according to your dynamic reconfigure script
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 53
        self.parameters.adaptiveThreshWinSizeStep = 4
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.01
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.errorCorrectionRate = 0.6
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.markerBorderBits = 1
        self.parameters.maxErroneousBitsInBorderRate = 0.04
        self.parameters.minDistanceToBorder = 3
        self.parameters.minMarkerDistanceRate = 0.1
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.minOtsuStdDev = 5.0
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.parameters.perspectiveRemovePixelPerCell = 8
        self.parameters.polygonalApproxAccuracyRate = 0.01

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, "camera", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        image = current_frame
        # Object Detection
        # Detect markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(
            image, self.aruco_dict, parameters=self.parameters
        )

        cv2.aruco.drawDetectedMarkers(image, corners, ids)

 


        # plot difference
        # resize image
        cv2.imshow("Image", image)
        img_msg = self.br.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(img_msg)
        
        cv2.waitKey(1)

        # Show Results


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
