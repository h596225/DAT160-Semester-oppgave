import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import math
import asyncio
from rclpy.action import ActionServer
from multi_robot_challenge_interfaces.action import NavigateToPoint

class Bug2(Node):
    def __init__(self, robot_name, follow_side='right'):
        super().__init__('bug_robot')
        self.robot_name = robot_name
        self.follow_side = follow_side.lower()

        # Action Server
        self._action_server = ActionServer(
            self, NavigateToPoint, 'navigate_to_point', self.execute_callback
        )

        # Subscribers and publishers with proper namespacing
        self.laser_subscriber = self.create_subscription(LaserScan, f'/{robot_name}/scan', self.laser_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)

        # Wall-following variables
        self.active = True
        self.regions = None
        self.state = 0
        self.state_dict = {
            0: 'find the wall',
            1: 'turn',
            2: 'follow the wall',
        }

        # Timers
        self.create_timer(0.1, self.wall_follower)

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Fikk målpunkt: {goal_handle.request.point}")
        self.target_point = goal_handle.request.point

        feedback = NavigateToPoint.Feedback()

        while not self.is_goal_reached():
            # Send feedback
            feedback.progress = self.calculate_progress()
            goal_handle.publish_feedback(feedback)

            # Navigate towards the goal
            self.navigate_to_goal()
            await asyncio.sleep(0.1)

        # Complete the goal
        goal_handle.succeed()
        result = NavigateToPoint.Result()
        result.success = True
        result.result_message = "Målet er nådd!"
        return result

    def laser_callback(self, msg):
        # Process laser scan data for wall following
        ranges = msg.ranges
        if self.follow_side == 'right':
            self.regions = {
                'far': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'front': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'f_side': min(min(ranges[300:339]), 10.0),
                'side': min(min(ranges[270:309]), 10.0),
            }
        elif self.follow_side == 'left':
            self.regions = {
                'far': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'front': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'f_side': min(min(ranges[20:59]), 10.0),
                'side': min(min(ranges[50:89]), 10.0),
            }

    def change_state(self, state):
        if state != self.state:
            self.state = state

    def take_action(self):
        regions = self.regions
        d = 1.0  # Distance threshold

        if regions['front'] > d and regions['f_side'] > d and regions['side'] > d:
            self.change_state(0)
        elif regions['front'] < d:
            self.change_state(1)
        elif regions['f_side'] < d:
            self.change_state(2)
        else:
            self.change_state(0)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = -0.3 if self.follow_side == 'right' else 0.3
        return msg

    def turn(self):
        msg = Twist()
        msg.angular.z = 0.5 if self.follow_side == 'right' else -0.5
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.5
        return msg

    def wall_follower(self):
        if not self.active or self.regions is None:
            return

        self.take_action()

        if self.state == 0:
            cmd = self.find_wall()
        elif self.state == 1:
            cmd = self.turn()
        elif self.state == 2:
            cmd = self.follow_the_wall()
        else:
            self.get_logger().error('Unknown state!')
            cmd = Twist()

        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd)

    def odom_callback(self, msg):
        self.current_pose = msg.position

    def is_goal_reached(self):
        if not self.current_pose or not self.target_point:
            return False
        dx = self.target_point.x - self.current_pose.x
        dy = self.target_point.y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        return distance < 0.1

    def navigate_to_goal(self):
        twist = Twist()
        goal_angle = math.atan2(
            self.target_point.y - self.current_pose.y,
            self.target_point.x - self.current_pose.x
        )
        twist.linear.x = 0.5
        twist.angular.z = goal_angle
        self.cmd_vel_publisher.publish(twist)

    def pause_navigation(self):
        self.active = False
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

    def calculate_progress(self):
        if not self.current_pose or not self.target_point:
            return 0
        dx = self.target_point.x - self.current_pose.x
        dy = self.target_point.y - self.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        return max(0, min(100, 100 - distance * 10))


def main(args=None):
    rclpy.init(args=args)

    # Parse arguments
    robot_name = 'tb3_0'
    follow_side = 'right'

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    if len(sys.argv) > 2:
        follow_side = sys.argv[2]

    node = Bug2(robot_name, follow_side)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()