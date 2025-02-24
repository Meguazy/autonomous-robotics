import ast
import sys
import rclpy
import json

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from std_msgs.msg import String  # For semaphore topic
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
from rclpy.qos import qos_profile_sensor_data


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('dobot_cv')

        # Semaphore subscriber
        self.green_light_detected = False
        self.semaphore_subscriber = self.create_subscription(
            String,
            '/semaphore',
            self.semaphore_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Tag detections subscription
        self.tag_subscription = self.create_subscription(
            String,  # Replace with your actual message type
            '/tag_detections',
            self.tag_callback,
            10  # QoS depth
        )

        # Store messages in a buffer
        self.msg_buffer = []

        # Action client for PointToPoint
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=ReentrantCallbackGroup())
        
        # Gripper service client
        self.cli = self.create_client(
            srv_type=GripperControl,
            srv_name='dobot_gripper_service',
            callback_group=ReentrantCallbackGroup()
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper service not available, waiting again...')
        self.req = GripperControl.Request()

        self.tasks_list = []
        self.goal_num = 0
        
    def tag_callback(self, msg):
        # Add received message to the buffer
        self.msg_buffer.append(msg)
        self.get_logger().info('Adding message to the detections buffer...')
        if len(self.msg_buffer) > 10:  # Keep the buffer size to the last 10 messages
            self.get_logger().info('Removing message from the detections buffer...')
            self.msg_buffer.pop(0)

    def semaphore_callback(self, msg):
        if msg.data.lower() == "green":
            self.get_logger().info("Green light detected! Starting pick-and-place...")
            
            if self.goal_num > 0:
                self.goal_num = 0
            self.green_light_detected = True

            self.process_tag_detections()
            self.execute()

            # Reset green_light_detected so it can trigger again

        elif msg.data.lower() == "red":
            self.get_logger().info("Red light detected. Waiting for green light...")


    def process_tag_detections(self):
        # Calculate means from the message buffer
        mean_values = self.calculate_means(self.msg_buffer)
        self.log_means(mean_values)
        
        mean_x_red_brick = mean_values.get('red-lego', {}).get('mean_x', None)
        mean_y_red_brick = mean_values.get('red-lego', {}).get('mean_y', None)
        
        mean_x_black_brick = mean_values.get('black-lego', {}).get('mean_x', None)
        mean_y_black_brick = mean_values.get('black-lego', {}).get('mean_y', None)
        
        mean_x_white_brick = mean_values.get('white-lego', {}).get('mean_x', None)
        mean_y_white_brick = mean_values.get('white-lego', {}).get('mean_y', None)
        
        mean_x_tag_3 = mean_values.get('3', {}).get('mean_x', None)
        mean_y_tag_3 = mean_values.get('3', {}).get('mean_y', None)
        
        mean_x_tag_2 = mean_values.get('2', {}).get('mean_x', None)
        mean_y_tag_2 = mean_values.get('2', {}).get('mean_y', None)

        mean_x_tag_1 = mean_values.get('1', {}).get('mean_x', None)
        mean_y_tag_1 = mean_values.get('1', {}).get('mean_y', None)


        self.tasks_list = [
            ["move", [mean_x_red_brick, mean_y_red_brick, 10.0, 65.0], 1, 1.0, 1.0],  # Move with 50% speed
            ["gripper", "open", False],
            ["move", [mean_x_red_brick, mean_y_red_brick, -8.0, 65.0], 1, 0.1, 0.1], # Move faster
            ["gripper", "close", True],
            ["move", [mean_x_red_brick, mean_y_red_brick, 100.0, 65.0], 1, 1.0, 1.0],
            ["move", [mean_x_tag_1, mean_y_tag_1, 100.0, 65.0], 1, 1.0, 1.0],  # Full speed
            ["move", [mean_x_tag_1, mean_y_tag_1, -8.0, 65.0], 1, 1.0, 1.0],
            ["gripper", "open", False],
            ["move", [mean_x_tag_1, mean_y_tag_1, 10.0, 65.0], 1, 1.0, 1.0],
            
            ["move", [mean_x_black_brick, mean_y_black_brick, 10.0, 65.0], 1, 1.0, 1.0],  # Move with 50% speed
            ["move", [mean_x_black_brick, mean_y_black_brick, -8.0, 65.0], 1, 0.1, 0.1], # Move faster
            ["gripper", "close", True],
            ["move", [mean_x_black_brick, mean_y_black_brick, 100.0, 65.0], 1, 1.0, 1.0],
            ["move", [mean_x_tag_2, mean_y_tag_2, 100.0, 65.0], 1, 1.0, 1.0],  # Full speed
            ["move", [mean_x_tag_2, mean_y_tag_2, -8.0, 65.0], 1, 1.0, 1.0],
            ["gripper", "open", False],
            ["move", [mean_x_tag_2, mean_y_tag_2, 10.0, 65.0], 1, 1.0, 1.0],
            
            ["move", [mean_x_white_brick, mean_y_white_brick, 10.0, 65.0], 1, 1.0, 1.0],  # Move with 50% speed
            ["move", [mean_x_white_brick, mean_y_white_brick, -8.0, 65.0], 1, 0.1, 0.1], # Move faster
            ["gripper", "close", True],
            ["move", [mean_x_white_brick, mean_y_white_brick, 100.0, 65.0], 1, 1.0, 1.0],
            ["move", [mean_x_tag_3, mean_y_tag_3, 100.0, 65.0], 1, 1.0, 1.0],  # Full speed
            ["move", [mean_x_tag_3, mean_y_tag_3, -8.0, 65.0], 1, 1.0, 1.0],
            ["gripper", "open", False],
            ["move", [mean_x_tag_3, mean_y_tag_3, 10.0, 65.0], 1, 1.0, 1.0],
            ["gripper", "close", False],
            
            ["move", [150.0, 0.0, 100.0, 0.0], 1, 1.0, 1.0]  # Move quickly to reset position
        ]

    def calculate_means(self, msgs):
        tag_data = {}

        for msg in msgs:
            try:
                # Replace single quotes with double quotes to make it valid JSON
                msg_data_str = msg.data.replace("'", '"')
                
                # msg is a String object, so we need to deserialize its data
                msg_data = json.loads(msg_data_str)  # Convert the JSON string to a Python list of dictionaries
                
                # Iterate over each tag in the message data
                for tag in msg_data:
                    tag_id = tag['tag_id']
                    x = tag['x']
                    y = tag['y']

                    if tag_id not in tag_data:
                        tag_data[tag_id] = {'sum_x': 0, 'sum_y': 0, 'count': 0}

                    tag_data[tag_id]['sum_x'] += x
                    tag_data[tag_id]['sum_y'] += y
                    tag_data[tag_id]['count'] += 1
            except Exception as e:
                self.get_logger().warning(f"Error processing message: {e}")

        # Now calculate the mean values for each tag
        mean_values = {}
        for tag_id, data in tag_data.items():
            mean_values[tag_id] = {
                'mean_x': data['sum_x'] / data['count'],
                'mean_y': data['sum_y'] / data['count'],
            }

        return mean_values


    def log_means(self, mean_values):
        for tag_id, means in mean_values.items():
            self.get_logger().info(f"Tag ID: {tag_id}, Mean X: {means['mean_x']:.2f}, Mean Y: {means['mean_y']:.2f}")

    def execute(self):
        """Execute tasks if green light is detected."""
        self.get_logger().info(f"Green Light Detected: {self.green_light_detected}")

        if not self.green_light_detected:
            self.get_logger().info("Waiting for green light to start execution...")
            return

        if self.goal_num >= len(self.tasks_list):
            self.get_logger().info("All tasks completed. Shutting down.")
            self.green_light_detected = False
            rclpy.shutdown()
            sys.exit()

        self.get_logger().info(f'*** TASK NUM ***: {self.goal_num}/{len(self.tasks_list)-1}')

        task = self.tasks_list[self.goal_num]
        self.get_logger().info(f"Executing task: {task}")

        if task[0] == "gripper":
            self.send_request(*task[1:])
            self.goal_num += 1
            self.timer = self.create_timer(0.1, self.timer_callback, callback_group=ReentrantCallbackGroup())

        elif task[0] == "move":
            self.send_goal(*task[1:])
            self.goal_num += 1

        else:
            self.get_logger().error(f"Unknown task type: {task[0]}")
            self.goal_num += 1  # Prevent getting stuck

    def timer_callback(self):
        if self.srv_future.done():
            result = self.srv_future.result()
            self.get_logger().info(f'Result of service call: {result}')
            self.timer.cancel()
            self.execute()

    def send_request(self, gripper_state, keep_compressor_running):
        self.req.gripper_state = gripper_state
        self.req.keep_compressor_running = keep_compressor_running
        self.srv_future = self.cli.call_async(self.req)

    def send_goal(self, _target, _type, velocity_ratio=1.0, acceleration_ratio=1.0):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = _target
        goal_msg.motion_type = _type
        goal_msg.velocity_ratio = velocity_ratio
        goal_msg.acceleration_ratio = acceleration_ratio

        self.get_logger().info(f"Sending goal: {goal_msg}")

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Result of action call: {result}')
            self.execute()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')


def main(args=None):
    rclpy.init(args=args)

    action_client = PickAndPlace()
    executor = MultiThreadedExecutor()

    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()
