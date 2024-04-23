import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class InteractiveMarker(Node):
    """
    This is an interactive marker node, that will detect if its grabbed by the dvrk robot,
    and adjust its color and position accordingly
    """
    def __init__(self, position, id, grab_error = 0.002, tcp_offset = 0.008):
        super().__init__('interactive_marker')

        # Initialize default values
        self.DVRK_TCP: PoseStamped = None 
        self.DVRK_Jaw: JointState = None
        self.grabbed = False

        # Get values from constructor, passed from marker factory
        self.grab_error = grab_error
        self.tcp_offset = tcp_offset

        # Create Publishers, Subscribers, Timers
        self.TCPSubscription = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.TCP_Callback, 10)
        self.JawSubscription = self.create_subscription(JointState, '/PSM1/jaw/measured_js', self.JAW_Callback,10)
        self.publisher_ = self.create_publisher(Marker, 'markers', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Create marker, whit id passed from constructor
        self.marker = Marker()
        self.marker.header.frame_id = 'PSM1_psm_base_link'
        self.marker.id = id
        self.marker.ns = "dvrk_viz"
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.008
        self.marker.scale.y = 0.008
        self.marker.scale.z = 0.008
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]
    
    def TCP_Callback(self, msg):
        """
        This function is a callback for TCP
        """
        self.DVRK_TCP = msg

    def JAW_Callback(self, msg):
        """
        This function is a callback for JAW
        """
        self.DVRK_Jaw = msg
    
    def decide_grabbed(self):
        """
        This function decided if the marker is currently grabbed by the jaw of the dvrk robot
        """

        # If this instance is currently in grabbed state, we only have to check if the jaw is closed or not
        if(self.grabbed):
            self.grabbed = self.DVRK_Jaw.position[0] == 0.0

        # Else, we have to check the distance from the dvrk robot jaw from the markers position, adjusted with the tcp offset
        else:
            dvrk_pos_np = np.array([self.DVRK_TCP.pose.position.x,
                                    self.DVRK_TCP.pose.position.y,
                                    self.DVRK_TCP.pose.position.z-self.tcp_offset])
            marker_pos_np = np.array([self.marker.pose.position.x,
                                    self.marker.pose.position.y,
                                    self.marker.pose.position.z])
            
            distance_to_marker = np.linalg.norm(dvrk_pos_np-marker_pos_np)

            # If distance is less then the allowed error, we check jaw position
            if(distance_to_marker <= self.grab_error):
                self.grabbed = self.DVRK_Jaw.position[0] == 0.0

    def timer_callback(self):
        """
        This function runs on a timer, acts as a main loop for the marker
        """

        # If dvrk topic is ready, we check if grabbed
        if (self.DVRK_TCP is not None and self.DVRK_Jaw is not None):
            self.decide_grabbed()

            if(self.grabbed):
                #turn grabbed marker blue
                self.marker.color.r = 0.0
                self.marker.color.g = 0.0
                self.marker.color.b = 1.0
                
                #move grabbed marker to the tcp of dvrk robot, adjusted by the tcp offset
                self.marker.pose.position.x = self.DVRK_TCP.pose.position.x
                self.marker.pose.position.y = self.DVRK_TCP.pose.position.y
                self.marker.pose.position.z = self.DVRK_TCP.pose.position.z-self.tcp_offset
            else:
                #turn not grabbed marker back to green
                self.marker.color.r = 0.0
                self.marker.color.g = 1.0
                self.marker.color.b = 0.0
        

def main(args=None):
    # Only for testing porpuses
    rclpy.init(args=args)
    interactive_marker = InteractiveMarker([-0.05, 0.08, -0.14], 1)
    rclpy.spin(interactive_marker)

    interactive_marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
