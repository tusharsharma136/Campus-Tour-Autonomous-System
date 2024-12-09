import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
from rclpy.timer import Timer

class CIAgent(Node):
    def __init__(self):
        super().__init__('ci_agent')
        self.subscription = self.create_subscription(
            String,
            'ci_requests',
            self.request_callback,
            10)
        self.publisher = self.create_publisher(String, 'ci_responses', 10)
        self.status = "available"
        self.busy_timer = None
        self.escort_duration = 10.0  # Time in seconds for escorting a visitor
        self.get_logger().info('CI Agent is ready')

    def request_callback(self, msg):
        data = json.loads(msg.data)
        if data['type'] == 'escort_request':
            response = self.process_escort_request(data)
        else:
            response = {'type': 'error', 'message': 'Unknown request type'}
        
        self.publisher.publish(String(data=json.dumps(response)))

    def process_escort_request(self, data):
        if self.status == "available":
            self.status = "busy"
            # Simulate campus navigation. In a real scenario, this would use actual campus layout.
            path = ['Entrance', 'Main Path', data['destination']]
            
            # Set a timer to free up the CI agent after the escort duration
            self.busy_timer = self.create_timer(self.escort_duration, self.free_agent)
            
            return {
                'type': 'escort_response',
                'path': path,
                'visitor': data['visitor'],
                'estimated_time': self.escort_duration
            }
        else:
            return {
                'type': 'escort_response',
                'error': 'CI Agent is busy',
                'visitor': data['visitor']
            }

    def free_agent(self):
        self.status = "available"
        if self.busy_timer:
            self.busy_timer.cancel()
            self.busy_timer = None
        self.get_logger().info('CI Agent is now available')

def main(args=None):
    rclpy.init(args=args)
    ci_agent = CIAgent()
    rclpy.spin(ci_agent)
    ci_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

