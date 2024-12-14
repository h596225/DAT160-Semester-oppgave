import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, Int64
from geometry_msgs.msg import Point, Pose
from multi_robot_challenge_interfaces.action import NavigateToPoint  

class Robot_handler(Node):
    def __init__(self):
        super().__init__('robot_handler')
        
        # Namespace parameter
        self.declare_parameter('namespace', 'tb3_0')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
         # Action Client
        self._action_client = ActionClient(self, NavigateToPoint, 'navigate_to_point')

        # Subscriptions
        self.sub_global_destination = self.create_subscription(Point, '/destination_point', self.clbk_global_destination, 10)
        self.sub_marker_id = self.create_subscription(Int64, f'/{self.namespace}/marker_id', self.clbk_marker_id, 10)
        self.sub_marker_pose = self.create_subscription(Pose, f'/{self.namespace}/marker_map_pose', self.clbk_marker_pose, 10)
        
        # Publishers
        self.pub_request_help = self.create_publisher(Bool, f'/{self.namespace}/need_help', 10)
        
        self.acro_marker_id = None
        self.marker_pose = None

        self.get_logger().info(f'{self.namespace} - Robot_handler startet ...')
        
        # Timer
        timer_period = 0.1  # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def clbk_global_destination(self, msg):
        # Mottar destinasjon fra leder og sender som et mål til Bug2
        self.get_logger().info(f'Mottatt punkt: x={msg.x}, y={msg.y}')
        self.send_goal(msg.x, msg.y)


    # Endret til action-klient server
    def send_goal(self, x, y):
        # Sjekker om serveren er tilgjengelig før den sender målet
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action Server ikke tilgjengelig. Kan ikke sende mål.')
            return

        # Opprett målet og send det
        goal_msg = NavigateToPoint.Goal()
        goal_msg.point = Point(x=x, y=y)

        self.get_logger().info(f'Sender målpunkt: x={x}, y={y}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Fremdrift: {feedback_msg.feedback.progress}%')


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Målpunktet ble avvist!')
            return

        self.get_logger().info('Målpunktet ble akseptert.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Navigasjonen fullført: {result.result_message}')
        else:
            self.get_logger().warning(f'Navigasjonen feilet: {result.result_message}')


    def clbk_marker_id(self, msg):
        # Markør gjennkjenning 
        self.acro_marker_id = msg.data
        self.get_logger().info(f'Markør ID: {self.acro_marker_id} av {self.namespace}')
    

    def clbk_marker_pose(self, msg):
        # Oppdaterer posisjonen for markøren
        self.marker_pose = msg
        self.get_logger().info(f'Markør posisjon: x={msg.position.x}, y={msg.position.y}')
    

    def check_help_needed(self):
        # Sjekker dersom det er stor brann (ID 4), og sender deretter hjelp request til lederen
        help_msg = Bool()
        help_msg.data = (self.acro_marker_id == 4)
        self.pub_request_help.publish(help_msg)
        
        if help_msg.data:
            self.get_logger().info(f'Stor flamme oppdaget! Ønsker hjelp av {self.namespace}')
    

    def timer_callback(self):
        # Sjekker for markører
        if self.acro_marker_id is not None:
            self.check_help_needed()


def main(args=None):
    rclpy.init(args=args)
    robot_handler = Robot_handler()
    rclpy.spin(robot_handler)
    robot_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()