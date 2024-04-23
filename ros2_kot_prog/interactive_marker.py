import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class InteractiveMarker(Node):
    def __init__(self, position, id):
        super().__init__('interactive_marker')

        self.DVRK_TCP: PoseStamped = None 
        self.DVRK_Jaw: JointState = None
        self.grabbed = False

        self.grab_error = 0.002
        self.tcp_offset = 0.008

        self.TCPSubscription = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.TCP_Callback, 10)
        self.JawSubscription = self.create_subscription(JointState, '/PSM1/jaw/measured_js', self.JAW_Callback,10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher_ = self.create_publisher(Marker, 'markers', 10)

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
        self.DVRK_TCP = msg

    def JAW_Callback(self, msg):
        self.DVRK_Jaw = msg
    
    def decide_grabbed(self):
        if(self.grabbed):
            self.grabbed = self.DVRK_Jaw.position[0] == 0.0

        else:
            dvrk_pos_np = np.array([self.DVRK_TCP.pose.position.x,
                                    self.DVRK_TCP.pose.position.y,
                                    self.DVRK_TCP.pose.position.z-self.tcp_offset])
            marker_pos_np = np.array([self.marker.pose.position.x,
                                    self.marker.pose.position.y,
                                    self.marker.pose.position.z])
            
            distance_to_marker = np.linalg.norm(dvrk_pos_np-marker_pos_np)

            if(distance_to_marker <= self.grab_error):
                self.grabbed = self.DVRK_Jaw.position[0] == 0.0

    def timer_callback(self):
        if (self.DVRK_TCP is not None and self.DVRK_Jaw is not None):
            self.decide_grabbed()

        if(self.grabbed):
            self.marker.color.r = 0.0
            self.marker.color.g = 0.0
            self.marker.color.b = 1.0
            
            self.marker.pose.position.x = self.DVRK_TCP.pose.position.x
            self.marker.pose.position.y = self.DVRK_TCP.pose.position.y
            self.marker.pose.position.z = self.DVRK_TCP.pose.position.z-self.tcp_offset
        else:
            self.marker.color.r = 0.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
        

def main(args=None):
    rclpy.init(args=args)
    interactive_marker = InteractiveMarker([-0.05, 0.08, -0.14], 1)
    rclpy.spin(interactive_marker)

    interactive_marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
