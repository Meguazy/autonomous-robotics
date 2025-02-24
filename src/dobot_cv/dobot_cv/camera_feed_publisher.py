import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pickle
import numpy as np
import time

class CameraPublisher(Node):
    def __init__(self, width=640, height=480):
        super().__init__('dobot_cv')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(5, self.publish_frame)  # Publish at 500 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        # Load the camera matrix from pickle and set as parameter
        with open("/home/finucci/magician_ros2_control_system_ws/src/dobot_cv/camera_params/cameraMatrix.pkl", "rb") as f:
            self.camera_matrix = pickle.load(f)
        with open("/home/finucci/magician_ros2_control_system_ws/src/dobot_cv/camera_params/dist.pkl", "rb") as f:
            self.dist = pickle.load(f)
                
        newCameraMatrix, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist, (width, height), 1, (width, height))

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist, None, newCameraMatrix, (width, height), cv2.CV_32FC1
        )
        # Set camera properties (optional)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            # Convert OpenCV image to ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
            
            # Log the frame publishing
            self.get_logger().info("Published a new frame")
        else:
            # Log the failure to capture frame
            self.get_logger().warn("Failed to capture frame from camera")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
