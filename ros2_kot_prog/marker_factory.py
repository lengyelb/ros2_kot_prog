import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros2_kot_prog.interactive_marker import InteractiveMarker
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

class MarkerFactory(Node):
    def __init__(self):
        super().__init__('marker_factory')

        self.DVRK_TCP: PoseStamped = None 

        self.declare_parameter('grab_error', 0.002)
        self.declare_parameter('tcp_offset', 0.008)
        self.declare_parameter('debounce_timeout', 2)

        self.grab_error = self.get_parameter('grab_error').get_parameter_value().double_value
        self.tcp_offset = self.get_parameter('tcp_offset').get_parameter_value().double_value
        self.timeout = self.get_parameter('debounce_timeout').get_parameter_value().integer_value

        self.last_press = self.get_clock().now() - rclpy.time.Duration(seconds=self.timeout)
        self.counter = 0

        self.TCPSubscription = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.TCP_Callback, 10)

        self.publisher_ = self.create_publisher(Marker, 'request_marker_button', 10)
        self.markerspublisher = self.create_publisher(MarkerArray, 'interactice_markers', 10)

        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.executor = MultiThreadedExecutor()

        self.interactiveMarkers = []
    
        self.button_marker = Marker()
        self.button_marker.header.frame_id = 'PSM1_psm_base_link'
        self.button_marker.id = 1
        self.button_marker.ns = "dvrk_viz"
        self.button_marker.type = Marker.CUBE
        self.button_marker.action = Marker.ADD
        self.button_marker.pose.orientation.x = 0.0
        self.button_marker.pose.orientation.y = 0.0
        self.button_marker.pose.orientation.z = 0.0
        self.button_marker.pose.orientation.w = 1.0
        self.button_marker.scale.x = 0.03
        self.button_marker.scale.y = 0.03
        self.button_marker.scale.z = 0.01
        self.button_marker.color.a = 1.0
        self.button_marker.color.r = 1.0
        self.button_marker.color.g = 0.0
        self.button_marker.color.b = 0.0
        self.button_marker.pose.position.x = 0.0
        self.button_marker.pose.position.y = 0.0
        self.button_marker.pose.position.z = -0.2

    def TCP_Callback(self, msg):
        self.DVRK_TCP = msg

    def wait_for_Subscriptions(self):
         # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while (self.DVRK_TCP is None) and rclpy.ok():
            self.get_logger().info('Waiting for subsriptions...')
            rclpy.spin_once(self)

    def add_marker(self):
        self.get_logger().info(f"Creating a new Interactive Marker Node, ID: {self.counter}")


        position=[random.uniform(0.1, -0.1), random.uniform(0.1, -0.1), random.uniform(-0.1, -0.2)]
        interactive_marker = InteractiveMarker(position=position, id=self.counter)
        self.interactiveMarkers.append(interactive_marker)
        self.executor.add_node(interactive_marker)
        self.counter += 1

    def publish_all(self):
        self.publisher_.publish(self.button_marker)

        msg = MarkerArray()
        for interactiveMarker in self.interactiveMarkers:
            msg.markers.append(interactiveMarker.marker)
        self.markerspublisher.publish(msg)
        self.publisher_.publish(self.button_marker)

    def timer_callback(self):
        when = self.last_press + rclpy.time.Duration(seconds=self.timeout)
        if(self.DVRK_TCP is not None and self.get_clock().now() > when):
            dvrk_pos_np = np.array([self.DVRK_TCP.pose.position.x,
                                        self.DVRK_TCP.pose.position.y,
                                        self.DVRK_TCP.pose.position.z-self.tcp_offset])
            marker_pos_np = np.array([self.button_marker.pose.position.x,
                                        self.button_marker.pose.position.y,
                                        self.button_marker.pose.position.z])
                
            distance_to_marker = np.linalg.norm(dvrk_pos_np-marker_pos_np)

            if(distance_to_marker <= self.grab_error):
                self.add_marker()
                self.last_press = self.get_clock().now()

        self.publish_all()

def main(args=None):
    rclpy.init(args=args)
    marker_factory = MarkerFactory()

    rclpy.spin(marker_factory)

    marker_factory.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
