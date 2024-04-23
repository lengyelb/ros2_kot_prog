import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros2_kot_prog.interactive_marker import InteractiveMarker
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

class MarkerFactory(Node):
    """
    This is a node that will create a marker, that acts as a "button" for the dvrk robot.
    Whenever the button is pressed, this class will create a new interactive marker randomly in 3D space
    """
    def __init__(self):
        super().__init__('marker_factory')
        # Initialize default values
        self.DVRK_TCP: PoseStamped = None 
        self.counter = 0
        self.timer_period = 0.1
        self.last_press = None
        self.interactiveMarkers = []
        self.executor = MultiThreadedExecutor()

        # Declare parameters
        self.declare_parameter('grab_error', 0.002)
        self.declare_parameter('tcp_offset', 0.008)
        self.declare_parameter('debounce_timeout', 2)

        # Get parameter values
        self.grab_error = self.get_parameter('grab_error').get_parameter_value().double_value
        self.tcp_offset = self.get_parameter('tcp_offset').get_parameter_value().double_value
        self.timeout = self.get_parameter('debounce_timeout').get_parameter_value().integer_value

        # Initialize last press
        self.last_press = self.get_clock().now() - rclpy.time.Duration(seconds=self.timeout)
        
        # Create Publishers, Subscribers, Timers
        self.TCPSubscription = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.TCP_Callback, 10)
        self.publisher_ = self.create_publisher(Marker, 'request_marker_button', 10)
        self.markerspublisher = self.create_publisher(MarkerArray, 'interactice_markers', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Create the marker that represents the interactive marker requester "button"
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
        """
        This function is a callback for TCP
        """
        self.DVRK_TCP = msg

    def wait_for_Subscriptions(self):
        """
        This function waits for all the subsriptions needed for operation
        """
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while (self.DVRK_TCP is None) and rclpy.ok():
            self.get_logger().info('Waiting for subsriptions...')
            rclpy.spin_once(self)

    def add_marker(self):
        """
        This function adds a new marker to the markers
        """
        self.get_logger().info(f"Creating a new Interactive Marker Node, ID: {self.counter}")

        # Create a random position in space where the marker will be placed
        position=[random.uniform(0.1, -0.1), random.uniform(0.1, -0.1), random.uniform(-0.1, -0.2)]

        # Create new marker, pass the values recieved from launch argumens
        interactive_marker = InteractiveMarker(position=position, id=self.counter, grab_error=self.grab_error, tcp_offset=self.tcp_offset)
        self.interactiveMarkers.append(interactive_marker)
        
        # Add the node to the excetuor so it can calculate its own position
        self.executor.add_node(interactive_marker)

        # Advance id counter
        self.counter += 1

    def publish_all(self):
        """
        This function publishes all relevant topics for this class
        """

        # publish the requester button
        self.publisher_.publish(self.button_marker)

        # Create the marker array, and publish it
        msg = MarkerArray()
        for interactiveMarker in self.interactiveMarkers:
            msg.markers.append(interactiveMarker.marker)
        self.markerspublisher.publish(msg)

    def timer_callback(self):
        """
        This function runs on a timer, acts as a main loop for the marker
        Decides if requester button is pressed, and adds a new marker based on that
        """

        # Check time the button was last pressed to debounce it
        when = self.last_press + rclpy.time.Duration(seconds=self.timeout)
        if(self.DVRK_TCP is not None and self.get_clock().now() > when):
            dvrk_pos_np = np.array([self.DVRK_TCP.pose.position.x,
                                        self.DVRK_TCP.pose.position.y,
                                        self.DVRK_TCP.pose.position.z-self.tcp_offset])
            marker_pos_np = np.array([self.button_marker.pose.position.x,
                                        self.button_marker.pose.position.y,
                                        self.button_marker.pose.position.z])
            
            
            distance_to_marker = np.linalg.norm(dvrk_pos_np-marker_pos_np)

            # If tcp (adjusted with offset) is within grab error, we add a new marker
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
