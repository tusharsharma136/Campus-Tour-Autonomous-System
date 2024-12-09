import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random

class BIAgent(Node):
    def __init__(self):
        super().__init__('bi_agent')
        self.subscription = self.create_subscription(
            String,
            'bi_requests',
            self.request_callback,
            10)
        self.publisher = self.create_publisher(String, 'bi_responses', 10)
        self.building = self.declare_parameter('building', 'Library').get_parameter_value().string_value
        self.status = "available"
        self.oos_end_time = 0
        self.get_logger().info(f'BI Agent for {self.building} is ready')

    def request_callback(self, msg):
        data = json.loads(msg.data)
        if data['type'] == 'navigation_request':
            response = self.process_navigation_request(data)
        elif data['type'] == 'status_request':
            response = self.process_status_request(data)
        else:
            response = {'type': 'error', 'message': 'Unknown request type'}
        
        self.publisher.publish(String(data=json.dumps(response)))

    def process_navigation_request(self, data):
        if self.status == "available":
            # Simulate navigation. In a real scenario, this would use actual building layout.
            path = [data['start'], f"Floor_{random.randint(1,3)}", data['end']]
            return {
                'type': 'navigation_response',
                'path': path,
                'building': self.building,
                'visitor': data['visitor']
            }
        else:
            return {
                'type': 'navigation_response',
                'error': 'BI Agent is out of service',
                'building': self.building,
                'visitor': data['visitor']
            }

    def process_status_request(self, data):
        if data.get('set_oos'):
            self.status = "OOS"
            self.oos_end_time = rclpy.time.Time.from_msg(self.get_clock().now().to_msg()).seconds + data['duration']
        return {
            'type': 'status_response',
            'status': self.status,
            'building': self.building
        }

def main(args=None):
    rclpy.init(args=args)
    bi_agent = BIAgent()
    rclpy.spin(bi_agent)
    bi_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

