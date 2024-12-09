import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random

class VisitorAgent(Node):
    def __init__(self):
        super().__init__('visitor_agent')
        self.ci_publisher = self.create_publisher(String, 'ci_requests', 10)
        self.bi_publisher = self.create_publisher(String, 'bi_requests', 10)
        self.ci_subscription = self.create_subscription(
            String,
            'ci_responses',
            self.ci_response_callback,
            10)
        self.bi_subscription = self.create_subscription(
            String,
            'bi_responses',
            self.bi_response_callback,
            10)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.status = "waiting"
        self.destination = random.choice(['Library', 'LHC', 'B1 Hostel'])
        self.get_logger().info(f'Visitor wants to go to {self.destination}')

    def timer_callback(self):
        if self.status == "waiting":
            self.request_escort()
        elif self.status == "at_building":
            self.request_building_navigation()

    def request_escort(self):
        request = {
            'type': 'escort_request',
            'visitor': self.get_name(),
            'destination': self.destination
        }
        self.ci_publisher.publish(String(data=json.dumps(request)))

    def request_building_navigation(self):
        request = {
            'type': 'navigation_request',
            'visitor': self.get_name(),
            'start': 'Entrance',
            'end': 'Meeting Room'
        }
        self.bi_publisher.publish(String(data=json.dumps(request)))

    def ci_response_callback(self, msg):
        data = json.loads(msg.data)
        if data['type'] == 'escort_response' and data['visitor'] == self.get_name():
            if 'error' not in data:
                self.status = "at_building"
                self.get_logger().info(f'Arrived at {self.destination}')
            else:
                self.get_logger().info(f'Escort error: {data["error"]}')

    def bi_response_callback(self, msg):
        data = json.loads(msg.data)
        if data['type'] == 'navigation_response' and data['visitor'] == self.get_name():
            if 'error' not in data:
                self.status = "in_meeting"
                self.get_logger().info(f'In meeting at {self.destination}')
            else:
                self.get_logger().info(f'Building navigation error: {data["error"]}')

def main(args=None):
    rclpy.init(args=args)
    visitor = VisitorAgent()
    rclpy.spin(visitor)
    visitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

