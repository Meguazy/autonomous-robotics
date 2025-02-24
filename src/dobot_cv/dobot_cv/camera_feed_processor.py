import cv2
import rclpy
import roboflow
import io
import numpy as np
import rcl_interfaces
import pickle

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from std_msgs.msg import String  # For publishing tag information as a JSON-like string

class CameraProcessor(Node):
    def __init__(self, families = "tag36h11", tag_size = 0.03, estimate_tag_pose = True):
        super().__init__('dobot_cv')
        self.publisher = self.create_publisher(String, '/tag_detections', 10)  # Publisher for tag data
        
        self.bridge = CvBridge()
        
        # Initializing the detector
        self.at_detector = Detector(
            families=families,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.5,
            debug=0,
        )
        
        self.tag_size = tag_size  # Tag size in meters
        self.tag_size_mm = tag_size * 1000
        self.estimate_tag_pose = estimate_tag_pose
                
        with open("/home/finucci/magician_ros2_control_system_ws/src/dobot_cv/camera_params/cameraMatrix.pkl", "rb") as f:
            self.camera_matrix = pickle.load(f)
            
        # Extract fx, fy, cx, cy
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # Log and store camera parameters
        self.camera_params = (fx, fy, cx, cy)
        
        # Roboflow API key and project details
        rf = roboflow.Roboflow(api_key="ncH2Wguo0cJ3wV8FZ7sv")
        project = rf.workspace().project("brick2-uslhy")
        self.model = project.version("2").model
        # Set confidence and overlap thresholds (optional)
        self.model.confidence = 50  # Minimum confidence threshold (50%)
        self.model.overlap = 25  # Maximum overlap between bounding boxes (25%)
                        
                        
        self.tag_0_center_offsetted = None
        self.tag_4_center = None
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_frame_callback,
            10
        )
        
    def process_lego_brick(self, frame):
        detections = []
        # Save the frame temporarily as a file for Roboflow inference
        cv2.imwrite("temp_frame.jpg", frame)

        # Perform inference on the saved frame
        prediction = self.model.predict("temp_frame.jpg")

        # Parse predictions and draw bounding boxes
        for prediction_item in prediction.json()["predictions"]:
            x, y, width, height = (
                prediction_item["x"],
                prediction_item["y"],
                prediction_item["width"],
                prediction_item["height"],
            )
            class_name = prediction_item["class"]
            confidence = prediction_item["confidence"]

            # Calculate bounding box coordinates
            x1 = int(x - width / 2)
            y1 = int(y - height / 2)
            x2 = int(x + width / 2)
            y2 = int(y + height / 2)
            
            center = [(x1 + x2)/2, (y1 + y2)/2]

            # Add the corresponding detection to detections2
            detections.append({
                "tag_id": class_name,
                "center": center,
                "corners": [x1, x2, y1, y2]
            })
            
        return detections
            
    # Calculate the Euclidean distance between two tags
    def calculate_distance(self, point1, point2):
        return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def process_frame_callback(self, msg):
        self.get_logger().info("Processing frame...")
        
        try:
            # Convert ROS2 Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Log the frame received
            self.get_logger().info("Received a new frame")
        except Exception as e:
            self.get_logger().error(f"Failed to process the image: {str(e)}")
        
        lego_detections = self.process_lego_brick(frame)
        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect tags
        detections = self.at_detector.detect(
            img=gray_frame,
            estimate_tag_pose=self.estimate_tag_pose,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )
        
         # Find the detection with tag_id == 0 to use as the origin
        tag_0_center = None
        
        for detection in detections: 
            if detection.tag_id == 0:
                tag_0_center = tuple(map(float, detection.center))
                self.tag_0_center_offsetted = (
                    tag_0_center[0] - 150,
                    tag_0_center[1] - 150
                )
                # Calculate the pixel size of the tag (in pixels)
                tag_0_corners = np.array(detection.corners, dtype=np.float64)
                tag_0_width_pixels = np.linalg.norm(tag_0_corners[0] - tag_0_corners[1])  # Horizontal distance (in pixels)
                tag_0_height_pixels = np.linalg.norm(tag_0_corners[1] - tag_0_corners[2])  # Vertical distance (in pixels)

                # Average pixel size (assuming the tag is square-shaped)
                tag_0_side_pixel_dim = (tag_0_width_pixels + tag_0_height_pixels) / 2.0

                # Calculate the pixels to mm ratio
                self.pixels_to_mm_ratio = self.tag_size_mm / tag_0_side_pixel_dim
                break
            
        
        for detection in detections: 
            if detection.tag_id == 4 and self.tag_0_center_offsetted is not None:
                self.tag_4_center = tuple(map(float, detection.center))
                
                relative_position_mm_off = (
                    -(self.tag_4_center[1] - self.tag_0_center_offsetted[1]) * self.pixels_to_mm_ratio,
                    -(self.tag_4_center[0] - self.tag_0_center_offsetted[0]) * self.pixels_to_mm_ratio
                )
                
                self.factor_x_mm = abs(relative_position_mm_off[0]) / 150
                self.factor_y_mm = abs(relative_position_mm_off[1]) / 150
                
                break
        
        tag_data_list = []
        
        if self.tag_0_center_offsetted is not None:
            # Compute the relative coordinates for other tags based on the inverted axis system
            for detection in detections:
                if detection.tag_id != 0:
                    tag_id = detection.tag_id
                    center = tuple(map(float, detection.center))  # The center of the tag
                                                                    
                    relative_position_mm = (
                        -(center[1] - self.tag_0_center_offsetted[1]) * self.pixels_to_mm_ratio,
                        -(center[0] - self.tag_0_center_offsetted[0]) * self.pixels_to_mm_ratio
                    )
                    
                    relative_position_mm_off = (
                        relative_position_mm[0] / self.factor_x_mm,
                        relative_position_mm[1] / self.factor_y_mm
                    )
                    
                    # Format the coordinates to 2 decimal places
                    x = relative_position_mm_off[0]
                    y = relative_position_mm_off[1]
                    
                    # Add tag data to the list
                    tag_data_list.append({
                        "tag_id": str(tag_id),
                        "x": round(x, 2),
                        "y": round(y, 2)
                    })

        if self.tag_0_center_offsetted is not None:
            # Compute the relative coordinates for other tags based on the inverted axis system
            for detection in lego_detections:
                tag_id = detection['tag_id']
                center = tuple(map(float, detection['center']))  # The center of the tag
                                                        
                # Swap and invert axes                    
                relative_position_mm = (
                    -(center[1] - self.tag_0_center_offsetted[1]) * self.pixels_to_mm_ratio,
                    -(center[0] - self.tag_0_center_offsetted[0]) * self.pixels_to_mm_ratio
                )
                    
                relative_position_mm_off = (
                    relative_position_mm[0] / self.factor_x_mm,
                    relative_position_mm[1] / self.factor_y_mm
                )
                
                # Format the coordinates to 2 decimal places
                x = relative_position_mm_off[0]
                y = relative_position_mm_off[1]
                
                # Add tag data to the list
                tag_data_list.append({
                    "tag_id": str(tag_id),
                    "x": round(x, 2),
                    "y": round(y, 2)
                })
        
        # Publish all tag data in a single message
        message = String()
        message.data = str(tag_data_list)  # Convert list to JSON-like string
        self.publisher.publish(message)
        self.get_logger().info(f"Published consolidated message: {message.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
