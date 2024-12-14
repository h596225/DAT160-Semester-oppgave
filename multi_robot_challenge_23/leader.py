import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid

import random

class Leader(Node):
    def __init__(self):
        super().__init__('RobotLeaderNode')

        # Subscriptions
        self.create_subscription(OccupancyGrid, 'filtered_map', self.clbk_map, 10)

        self.sub_tb3_0_help = self.create_subscription(Bool, '/tb3_0/need_help', self.clbk_tb3_0_help, 10)
        self.sub_tb3_1_help = self.create_subscription(Bool, '/tb3_1/need_help', self.clbk_tb3_1_help, 10)

        self.sub_tb3_0_marker = self.create_subscription(Pose, '/tb3_0/marker_map_pose', self.clbk_tb3_0_marker, 10)
        self.sub_tb3_1_marker = self.create_subscription(Pose, '/tb3_1/marker_map_pose', self.clbk_tb3_1_marker, 10)

        # Destination point publisher for all robots
        self.pub_destination_point = self.create_publisher(Point, '/destination_point', 10)

         # Map data for generating valid points
        self.map_data = None

        # State variables
        self.tb3_0_help_needed = False
        self.tb3_1_help_needed = False
        self.tb3_0_marker_pose = None
        self.tb3_1_marker_pose = None

        self.get_logger().info('Leader startet ...')

        # Timer
        timer_period = 0.1  # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def clbk_map(self, msg):
        # Store map data for point generation
        self.map_data = msg


    def clbk_tb3_0_help(self, msg):
        self.tb3_0_help_needed = msg.data


    def clbk_tb3_1_help(self, msg):
        self.tb3_1_help_needed = msg.data


    def clbk_tb3_0_marker(self, msg):
        self.tb3_0_marker_pose = msg


    def clbk_tb3_1_marker(self, msg):
        self.tb3_1_marker_pose = msg


    def generate_random_point(self):
        # Generer et random punkt i kartet for å utforske
        if not self.map_data:
            return None

        x_max = self.map_data.info.width
        y_max = self.map_data.info.height

        # Sjekker 10 ganger om random punktet er et gyldig sted i kartet
        for _ in range(10):
            x = random.uniform(0, x_max) * self.map_data.info.resolution
            y = random.uniform(0, y_max) * self.map_data.info.resolution

            point = Point(x=x, y=y)

            if self.is_point_valid(point):
                return point
        
        return None


    def is_point_valid(self, point):
        # Sjekker at punktet ikke er blokkert
        if not self.map_data:
            return False

        map_x = int((point.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        map_y = int((point.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        map_index = map_y * self.map_data.info.width + map_x
        return self.map_data.data[map_index] < 50


    def timer_callback(self):
        # Sjekker dersom hjelp trengs
        help_points = self.check_help()
        
        # Ellers skal de explore kartet
        if not help_points:
            random_point = self.generate_random_point()
            if random_point:
                self.pub_destination_point.publish(random_point)


    def check_help(self):
        help_points = []
        
        if self.tb3_0_help_needed and self.tb3_1_marker_pose:
            help_point = Point(x=self.tb3_1_marker_pose.position.x, y=self.tb3_1_marker_pose.position.y)
            help_points.append(help_point)
        
        if self.tb3_1_help_needed and self.tb3_0_marker_pose:
            help_point = Point(x=self.tb3_0_marker_pose.position.x, y=self.tb3_0_marker_pose.position.y)
            help_points.append(help_point)
        
        for point in help_points:
            self.pub_destination_point.publish(point)
        
        return help_points


def main(args=None):
    rclpy.init(args=args)
    robot_leader = Leader()
    rclpy.spin(robot_leader)
    robot_leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()