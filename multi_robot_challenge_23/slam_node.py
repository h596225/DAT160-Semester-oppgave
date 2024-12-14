import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import math

class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('simple_slam')

        # Declare parameters for robot names
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value

        # Subscribers for each robot's laser scan and odometry
        self.laser_subscribers = {}
        self.odom_subscribers = {}
        self.robot_states = {}

        for robot_name in robot_names:
            # Initialize state storage
            self.robot_states[robot_name] = {'pose': None, 'scan': None}

            # Subscribe to laser scans
            laser_topic = f'/{robot_name}/scan'
            self.laser_subscribers[robot_name] = self.create_subscription(
                LaserScan,
                laser_topic,
                self.create_laser_callback(robot_name),
                10
            )

            # Subscribe to odometry
            odom_topic = f'/{robot_name}/odom'
            self.odom_subscribers[robot_name] = self.create_subscription(
                Odometry,
                odom_topic,
                self.create_odom_callback(robot_name),
                10
            )

        # Publisher for the combined map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Timer to update the map periodically
        self.timer = self.create_timer(0.1, self.update_map)

        # Initialize the map (simple 2D numpy array for illustration)
        self.map_size = 400  # 400x400 grid
        self.map_resolution = 0.05  # Each cell represents 0.05 meters
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)

        # Precompute the map origin
        self.map_origin_x = - (self.map_size * self.map_resolution) / 2.0
        self.map_origin_y = - (self.map_size * self.map_resolution) / 2.0

    def create_laser_callback(self, robot_name):
        def laser_callback(msg):
            self.robot_states[robot_name]['scan'] = msg
        return laser_callback

    def create_odom_callback(self, robot_name):
        def odom_callback(msg):
            self.robot_states[robot_name]['pose'] = msg.pose.pose
        return odom_callback

    def update_map(self):
        # Simplified SLAM logic
        for robot_name, state in self.robot_states.items():
            if state['pose'] is not None and state['scan'] is not None:
                self.process_scan(robot_name, state['pose'], state['scan'])

        # Publish the updated map
        occupancy_grid = self.convert_map_to_occupancy_grid()
        self.map_publisher.publish(occupancy_grid)

    def process_scan(self, robot_name, pose, scan):
        # Extract robot's position and orientation
        robot_x = pose.position.x
        robot_y = pose.position.y

        # Convert quaternion to yaw angle
        orientation = pose.orientation
        yaw = self.quaternion_to_yaw(orientation)

        # Get laser scan parameters
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        ranges = scan.ranges

        # Iterate over each laser scan measurement
        num_ranges = len(ranges)
        angles = angle_min + np.arange(num_ranges) * angle_increment

        for i in range(num_ranges):
            range_reading = ranges[i]
            angle = angles[i]

            # Check for valid range readings
            if np.isfinite(range_reading) and scan.range_min < range_reading < scan.range_max:
                # Calculate the endpoint of the laser beam in the map frame
                laser_x = robot_x + range_reading * math.cos(yaw + angle)
                laser_y = robot_y + range_reading * math.sin(yaw + angle)

                # Convert world coordinates to map indices
                map_i, map_j = self.world_to_map_indices(laser_x, laser_y)

                # Check if indices are within map bounds
                if 0 <= map_i < self.map_size and 0 <= map_j < self.map_size:
                    # Mark the cell as occupied
                    self.map[map_i, map_j] = 100  # Occupied

                # Also update free space along the ray
                free_x = robot_x
                free_y = robot_y
                steps = int(range_reading / self.map_resolution)
                for step in range(steps):
                    free_x += self.map_resolution * math.cos(yaw + angle)
                    free_y += self.map_resolution * math.sin(yaw + angle)
                    free_i, free_j = self.world_to_map_indices(free_x, free_y)
                    if 0 <= free_i < self.map_size and 0 <= free_j < self.map_size:
                        if self.map[free_i, free_j] == 0:
                            self.map[free_i, free_j] = -1  # Free space
            else:
                continue  # Ignore invalid readings

    def world_to_map_indices(self, x, y):
        map_i = int((y - self.map_origin_y) / self.map_resolution)
        map_j = int((x - self.map_origin_x) / self.map_resolution)
        return map_i, map_j

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def convert_map_to_occupancy_grid(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'map'

        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_size
        occupancy_grid.info.height = self.map_size
        occupancy_grid.info.origin.position.x = self.map_origin_x
        occupancy_grid.info.origin.position.y = self.map_origin_y
        occupancy_grid.info.origin.orientation.w = 1.0

        # Prepare the data: map values should be between -1 and 100
        map_data = self.map.flatten()
        map_data = np.clip(map_data, -1, 100)
        occupancy_grid.data = map_data.tolist()

        return occupancy_grid

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()