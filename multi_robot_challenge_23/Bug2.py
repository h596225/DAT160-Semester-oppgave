import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from multi_robot_challenge_interfaces.action import NavigateToPoint  

import math

class Bug2(Node):
    def __init__(self):
        super().__init__('bug2_nav')

        # Namespace parameter
        self.declare_parameter('namespace', 'tb3_0')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

         # Action Server
        self._action_server = ActionServer(self, NavigateToPoint, 'navigate_to_point', self.execute_callback)
        
        # Subscriptions
        self.create_subscription(LaserScan, 'scan', self.clbk_lidar, 10)
        self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)

        # Publisher for bevegelse
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Navigasjons parametere
        self.lidar_ranges = []
        self.lidar_threshold = 0.5
        self.current_pose = None
        self.target_point = None
        
        # Bevegelse parametere
        self.speed = 0.3
        self.turn_speed = 0.5

        self.get_logger().info(f'{self.namespace} - Bug2 startet ...')

 
    # Endret til server-klient service
    async def execute_callback(self, goal_handle):
            self.get_logger().info(f'Mottok mål: {goal_handle.request.point}')

            self.target_point = goal_handle.request.point

            feedback = NavigateToPoint.Feedback()

            while not self.is_goal_reached():
                # Sender tilbakemelding om fremdrift
                distance = self.calculate_distance()
                feedback.progress = max(0, min(100, 100 - distance * 10))
                goal_handle.publish_feedback(feedback)

                # Utfør navigasjon
                self.navigate_to_goal()

                # venter litt før neste iterasjon
                await rclpy.sleep(0.1)

            # Målet er nådd
            goal_handle.succeed()
            result = NavigateToPoint.Result()
            result.success = True
            result.result_message = 'Målet ble nådd!'
            self.get_logger().info(f'{self.namespace} fullførte målet.')
            return result


    def clbk_lidar(self, msg):
        self.lidar_ranges = msg.ranges


    def clbk_odom(self, msg):
        # Current pose
        pose = msg.pose.pose
        self.current_pose = (pose.position.x, pose.position.y)


    def calculate_distance(self):
        if not self.current_pose or not self.target_point:
            return float('inf')
        return math.sqrt(
            (self.target_point.x - self.current_pose[0])**2 +
            (self.target_point.y - self.current_pose[1])**2
        )

    def is_goal_reached(self):
        return self.calculate_distance() < 0.5


    def navigate_to_goal(self):
        if not self.target_point or not self.current_pose:
            return

        twist = Twist()

        # Beregn retningen mot målet
        goal_angle = math.atan2(
            self.target_point.y - self.current_pose[1],
            self.target_point.x - self.current_pose[0]
        )

        # Beveg roboten mot målet
        twist.linear.x = self.speed
        twist.angular.z = self.turn_speed * goal_angle
        self.cmd_vel_pub.publish(twist)



    def point_reached(self):
        # Når roboten har ankommet punktet, vil den rotere rundt for å se etter acro
        twist = Twist()
        self.get_logger().info(f'{self.namespace} tar en 360 no scope')
        
        total_rotation = 0
        while total_rotation < 2 * math.pi:
            twist.angular.z = self.turn_speed
            self.cmd_vel_pub.publish(twist)
            total_rotation += self.turn_speed * 0.1


def main(args=None):
    rclpy.init(args=args)
    ros2_nav = Bug2()
    rclpy.spin(ros2_nav)
    ros2_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

