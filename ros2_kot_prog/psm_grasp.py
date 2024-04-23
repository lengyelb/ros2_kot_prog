import numpy as np
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker,  MarkerArray
from ros2_kot_prog.shape_dict import shapes

class PSM(Node):
    """
    This class controls the dvrk robot, requests new interactive markers, and moves them to their correct place
    """
    def __init__(self):
        super().__init__('psm_controller')

        # Initialize default values
        self.TCP: PoseStamped = None 
        self.Jaw: JointState = None
        self.Markers: list[Marker] = []
        self.MarkerRequester: Marker = None
        self.marker_num = 0
        self.home_coords = [0.0, 0.0, -0.12]

        # Declare parameters
        self.declare_parameter('v', 0.005)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('omega', 0.1)
        self.declare_parameter('tcp_offset', 0.008)
        self.declare_parameter('shape', 'X')

        # Get parameter values
        self.v = self.get_parameter('v').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.omega = self.get_parameter('omega').get_parameter_value().double_value
        self.tcp_offset = self.get_parameter('tcp_offset').get_parameter_value().double_value
        self.shape = self.get_parameter('shape').get_parameter_value().string_value

        # Create Publishers, Subscribers, Timers
        self.TCPSubscription = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.TCP_Callback, 10)
        self.JawSubscription = self.create_subscription(JointState, '/PSM1/jaw/measured_js', self.JAW_Callback,10) 
        self.MarkersSubscription = self.create_subscription(MarkerArray, 'interactice_markers',self.Markers_callback, 10)
        self.MarkerRequesterButtonSubscription = self.create_subscription(Marker, 'request_marker_button',self.MarkerRequester_callback, 10)

        self.TCPPub = self.create_publisher(PoseStamped, '/PSM1/servo_cp', 10)
        self.JawPub = self.create_publisher(JointState, '/PSM1/jaw/servo_jp', 10)

    def TCP_Callback(self, msg):
        """
        This function is a callback for TCP
        """
        self.TCP = msg

    def JAW_Callback(self, msg):
        """
        This function is a callback for JAW
        """
        self.Jaw = msg

    def Markers_callback(self, msg):
        """
        This function is a callback for Markers
        """
        # Parse the markers form the message
        self.Markers = [marker for marker in msg.markers]
        # Sort them based on id, so the newest one is always at the end
        self.Markers.sort(key=lambda marker: marker.id)
    
    def MarkerRequester_callback(self, msg):
        """
        This function is a callback for the marker requester button
        """
        self.MarkerRequester = msg

    def wait_for_marker(self):
        """
        This function waits for a new marker to appear
        """
        while(len(self.Markers) == self.marker_num) and rclpy.ok():
            self.get_logger().info('Waiting for marker...')
            rclpy.spin_once(self)

        self.marker_num = len(self.Markers)

    def wait_for_Subscriptions(self):
        """
        This function waits for all the subsriptions needed for operation
        """
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while (self.TCP is None or self.Jaw is None or self.MarkerRequester is None) and rclpy.ok():
            self.get_logger().info('Waiting for subsriptions...')
            rclpy.spin_once(self)
    
    def move_tcp_to(self, target : list[float]):
        """
        This function moves the tool center point to a target using a linear trajectory

        Args:
            target: The target point [x, y, z]
        """
        msg = self.TCP

        # Calculate the linear trajectory
        pos_curr_np = np.array([self.TCP.pose.position.x*1000,
                                self.TCP.pose.position.y*1000,
                                self.TCP.pose.position.z*1000])
        pos_des_np = np.array(target)

        distance = np.linalg.norm(pos_des_np-pos_curr_np)/1000
        T = distance/self.v
        N = int(math.floor(T / self.dt))
        x = np.linspace(self.TCP.pose.position.x, target[0], N)
        y = np.linspace(self.TCP.pose.position.y, target[1], N)
        z = np.linspace(self.TCP.pose.position.z, target[2], N)

        # Traverse the trajectory
        self.loop_rate = self.create_rate(1.0 / self.dt, self.get_clock()) # Hz
        for i in range(N):
            if(not rclpy.ok()):
                break

            msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x = x[i]
            msg.pose.position.y = y[i]
            msg.pose.position.z = z[i]

            self.TCPPub.publish(msg)
            rclpy.spin_once(self)

    def move_jaw_to(self, target: float):
        """
        This function moves the jaw to a target using a linear trajectory

        Args:
            target: The target opening of the jaw in radians
        """
        msg = self.Jaw

        # Calculate the linear trajectory
        distance = np.linalg.norm(target-self.Jaw.position[0])
        T = distance/self.omega
        N = math.floor(T / self.dt)
        tr_jaw = np.linspace(self.Jaw.position[0], target, N)

        # Traverse the trajectory
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        for i in range(N):
            if(not rclpy.ok()):
                break

            msg.header.stamp=self.get_clock().now().to_msg()
            msg.position = [tr_jaw[i]]
            self.JawPub.publish(msg)
            rclpy.spin_once(self)
    
    def grab_marker(self):
        """
        This function grabs the last marker that appeared in 3D space
        """
        self.wait_for_Subscriptions()

        # Open the jaw
        self.move_jaw_to(target=0.8)

        # Get the position of the latest marker, adjust it with the offset, and move to it
        marker_to_move = self.Markers[-1]

        self.move_tcp_to(target=[ marker_to_move.pose.position.x,
                                  marker_to_move.pose.position.y,
                                  marker_to_move.pose.position.z+self.tcp_offset])
        
        # Close jaw
        self.move_jaw_to(target=0.0)
    
    def move_marker_to(self, target):
        """
        This function moves the latest marker to a target position
        """
        # Grab marker
        self.grab_marker()
        time.sleep(0.2)
        # Move marker
        self.move_tcp_to(target=target)
        time.sleep(0.2)
        # Release marker
        self.move_jaw_to(target=0.8)

    def request_marker(self):
        """
        This funciton requests a marker for grabbing
        """

        # Move to the the requester button, adjusted by offset
        self.move_tcp_to(target=[ self.MarkerRequester.pose.position.x,
                                  self.MarkerRequester.pose.position.y,
                                  self.MarkerRequester.pose.position.z+self.tcp_offset])
        # Wait for the marker to appear
        self.wait_for_marker()

    def draw_shape(self):
        """
        This funciton draws a desired shape
        """

        self.wait_for_Subscriptions()

        # Check if rquested shape can be drawn
        if(self.shape not in shapes.keys()):
            self.get_logger().info('The provided shape is invalid')
            return

        # Get target points form dictionary
        targets = shapes[self.shape]
        for target in targets:
            # Reqeest a marker and move it to target point
            self.request_marker()
            self.move_marker_to(target=target)
        
        # Reset tool to home coordinates
        self.move_tcp_to(self.home_coords)

def main(args=None):
    rclpy.init(args=args)
    psm = PSM()

    psm.draw_shape()

    psm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
