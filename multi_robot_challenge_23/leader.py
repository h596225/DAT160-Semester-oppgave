import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import String, Int64  # Int64 brukes for markør-ID-er

class Leader(Node):
    def __init__(self):
        super().__init__('RobotLeaderNode')

        # Subscriptions
        self.sub_help_request = self.create_subscription(Point, '/leader/help_request', self.clbk_help_request, 10)
        self.sub_marker_id = self.create_subscription(Int64, '/leader/marker_id', self.clbk_marker_id, 10)

        # Publishers
        self.help_coordination_pub = self.create_publisher(String, '/point', 10)

        # State tracking
        self.discovered_markers = set()
        self.total_score = 0

        self.get_logger().info('Leader startet ...')

    def clbk_help_request(self, msg):
        x, y = msg.x, msg.y
        help_msg = f"{x},{y}"
        self.get_logger().info(f'Hjelpeforespørsel mottatt for posisjon ({x}, {y}). Sender til robotene.')
        self.help_coordination_pub.publish(String(data=help_msg))

    def clbk_marker_id(self, msg):
        marker_id = msg.data
        self.update_score(marker_id)

    def update_score(self, marker_id):
        if marker_id in self.discovered_markers:
            return  # Ignorer dublikater
        self.discovered_markers.add(marker_id)

        # Scoring system
        if marker_id in {0, 1, 3}:
            self.total_score += 100
            self.get_logger().info(f'Markør {marker_id} funnet: Liten flamme (+100). Totalscore: {self.total_score}')
        elif marker_id == 2:
            self.total_score += 100
            self.get_logger().info(f'Markør {marker_id} funnet: Menneske (+100). Totalscore: {self.total_score}')
        elif marker_id == 4:
            self.total_score += 100
            self.get_logger().info(f'Markør {marker_id} funnet: Stor flamme (+100). Totalscore: {self.total_score}')

def main(args=None):
    rclpy.init(args=args)
    robot_leader = Leader()
    rclpy.spin(robot_leader)
    robot_leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()