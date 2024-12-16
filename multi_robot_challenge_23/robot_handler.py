import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int64
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from multi_robot_challenge_interfaces.action import NavigateToPoint  

class Robot_handler(Node):
    def __init__(self):
        super().__init__('robot_handler')
        
        # Namespace parameter
        self.declare_parameter('namespace', 'tb3_0')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        # Action Client
        self._action_client = ActionClient(self, NavigateToPoint, f'/{self.namespace}/navigate_to_point')

        # Subscriptions
        self.create_subscription(Point, '/point', self.clbk_help_coordination, 10)
        self.create_subscription(Int64, f'/{self.namespace}/marker_id', self.clbk_marker_id, 10)
        self.create_subscription(Odometry, f'/{self.namespace}/odom', self.odom_callback, 10)

        # Publishers
        self.pub_request_help = self.create_publisher(Point, '/leader/help_request', 10)
        self.pub_marker_id = self.create_publisher(Int64, '/leader/marker_id', 10)
        
        self.acro_marker_id = None
        self.navigation_active = False
        self.current_goal = None
        self.current_pose = None
        self.scored_ids = set()

        self.get_logger().info(f'{self.namespace} - Robot_handler startet ...')

    def clbk_help_coordination(self, msg):
        data = msg.data.split(',')
        x, y = float(data[0]), float(data[1])
        self.get_logger().info(f'Hjelpekoordinasjon mottatt. Navigerer til posisjon ({x}, {y}).')
        self.send_point(x, y)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def clbk_marker_id(self, msg):
        self.acro_marker_id = msg.data

        # Sender markør id nr til lederen
        if self.acro_marker_id not in self.scored_ids:
            self.scored_ids.add(self.acro_marker_id)
            
            marker_msg = Int64()
            marker_msg.data = self.acro_marker_id
            self.pub_marker_id.publish(marker_msg)

        # Dersom markør 4, send hjelp request
        if self.acro_marker_id == 4 and self.current_pose:
            help_msg = Point(
                x=self.current_pose.position.x,
                y=self.current_pose.position.y,
                z=0.0)
            
            self.pub_request_help.publish(help_msg)
            self.get_logger().info(f'Hjelpeforespørsel sendt: ({help_msg.x}, {help_msg.y})')

def main(args=None):
    rclpy.init(args=args)
    robot_handler = Robot_handler()
    rclpy.spin(robot_handler)
    robot_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
