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
        

        # create aruco board
        objPoints = np.array(
            [
                [[40, 40, 0], [760, 40, 0], [760, 760, 0], [40, 760, 0]],
                [[360, 360, 0], [440, 360, 0], [440, 440, 0], [360, 440, 0]],
            ],
            dtype=np.float32,
        )
        self.board = cv2.aruco.Board(
            objPoints, dictionary=self.aruco_dict, ids=np.array([[29], [1]])
        )

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
        self.desired = np.array(
            [
                179.2055,
                99.2273,
                459.6814,
                101.1491,
                457.6014,
                381.6531,
                177.1040,
                379.4215,
            ],
            dtype=np.float32,
        )

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

        output_arr = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
        if ids is not None:
            visibility_pub_msg = Bool()
            visibility_pub_msg.data = True
            self.visibility_pub.publish(visibility_pub_msg)
    
            for i in range(len(ids)):
                id = ids[i]
                if id == 29:
                    output_arr[0] = float(corners[i][0][0][0])
                    output_arr[1] = float(corners[i][0][0][1])
                    output_arr[2] = float(corners[i][0][1][0])
                    output_arr[3] = float(corners[i][0][1][1])
                    output_arr[4] = float(corners[i][0][2][0])
                    output_arr[5] = float(corners[i][0][2][1])
                    output_arr[6] = float(corners[i][0][3][0])
                    output_arr[7] = float(corners[i][0][3][1])

                    # make arr to store centered square
                    centered_arr = np.zeros((4, 2))
                    # get center of square
                    cX, cY = (
                        output_arr[0] + (output_arr[4] - output_arr[0]) / 2,
                        output_arr[1] + (output_arr[5] - output_arr[1]) / 2,
                    )

                    # draw
                    cv2.circle(image, (int(cX), int(cY)), 5, (0, 0, 255), -1)

                    # subtract center from all points
                    centered_arr[0][0] = output_arr[0] - cX
                    centered_arr[0][1] = output_arr[1] - cY
                    centered_arr[1][0] = output_arr[2] - cX
                    centered_arr[1][1] = output_arr[3] - cY
                    centered_arr[2][0] = output_arr[4] - cX
                    centered_arr[2][1] = output_arr[5] - cY
                    centered_arr[3][0] = output_arr[6] - cX
                    centered_arr[3][1] = output_arr[7] - cY

                    # scale by 1/9
                    centered_arr = centered_arr / 9

                    # add center back to all points

                    output_arr[0] = centered_arr[0][0] + cX
                    output_arr[1] = centered_arr[0][1] + cY
                    output_arr[2] = centered_arr[1][0] + cX
                    output_arr[3] = centered_arr[1][1] + cY
                    output_arr[4] = centered_arr[2][0] + cX
                    output_arr[5] = centered_arr[2][1] + cY
                    output_arr[6] = centered_arr[3][0] + cX
                    output_arr[7] = centered_arr[3][1] + cY

                if id == 1:
                    output_arr[0] = float(corners[i][0][0][0])
                    output_arr[1] = float(corners[i][0][0][1])
                    output_arr[2] = float(corners[i][0][1][0])
                    output_arr[3] = float(corners[i][0][1][1])
                    output_arr[4] = float(corners[i][0][2][0])
                    output_arr[5] = float(corners[i][0][2][1])
                    output_arr[6] = float(corners[i][0][3][0])
                    output_arr[7] = float(corners[i][0][3][1])
        
        else:
            visibility_pub_msg = Bool()
            visibility_pub_msg.data = False
            self.visibility_pub.publish(visibility_pub_msg)
            
    


        msg = Float32MultiArray()
        msg.data = output_arr
        self.float_arr_pub.publish(msg)


        # draw
        if len(corners) >= 1:
            # draw midpoints
            cx = 640 / 2
            cy = 480 / 2
            cv2.circle(image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cX, cY = (
                output_arr[0] + (output_arr[4] - output_arr[0]) / 2,
                output_arr[1] + (output_arr[5] - output_arr[1]) / 2,
            )
            # draw
            cv2.circle(image, (int(cX), int(cY)), 5, (0, 255, 255), -1)            
            cv2.line(
                image,
                (int(output_arr[0]), int(output_arr[1])),
                (int(output_arr[2]), int(output_arr[3])),
                (255, 0, 0),
                2,
            )
            cv2.line(
                image,
                (int(output_arr[2]), int(output_arr[3])),
                (int(output_arr[4]), int(output_arr[5])),
                (255, 0, 0),
                2,
            )
            cv2.line(
                image,
                (int(output_arr[4]), int(output_arr[5])),
                (int(output_arr[6]), int(output_arr[7])),
                (255, 0, 0),
                2,
            )
            cv2.line(
                image,
                (int(output_arr[6]), int(output_arr[7])),
                (int(output_arr[0]), int(output_arr[1])),
                (255, 0, 0),
                2,
            )
            cv2.line(
                image,
                (int(output_arr[0]), int(output_arr[1])),
                (int(self.desired[0]), int(self.desired[1])),
                (0, 0, 255),
                2,
            )
            cv2.line(
                image,
                (int(output_arr[2]), int(output_arr[3])),
                (int(self.desired[2]), int(self.desired[3])),
                (0, 0, 255),
                2,
            )
            cv2.line(
                image,
                (int(output_arr[4]), int(output_arr[5])),
                (int(self.desired[4]), int(self.desired[5])),
                (0, 0, 255),
                2,
            )
            cv2.line(
                image,
                (int(output_arr[6]), int(output_arr[7])),
                (int(self.desired[6]), int(self.desired[7])),
                (0, 0, 255),
                2,
            )

        # plot desired
        cv2.line(
            image,
            (int(self.desired[0]), int(self.desired[1])),
            (int(self.desired[2]), int(self.desired[3])),
            (0, 255, 0),
            2,
        )
        cv2.line(
            image,
            (int(self.desired[2]), int(self.desired[3])),
            (int(self.desired[4]), int(self.desired[5])),
            (0, 255, 0),
            2,
        )
        cv2.line(
            image,
            (int(self.desired[4]), int(self.desired[5])),
            (int(self.desired[6]), int(self.desired[7])),
            (0, 255, 0),
            2,
        )
        cv2.line(
            image,
            (int(self.desired[6]), int(self.desired[7])),
            (int(self.desired[0]), int(self.desired[1])),
            (0, 255, 0),
            2,
        )

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
