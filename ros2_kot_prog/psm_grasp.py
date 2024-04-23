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

    def __init__(self):
        super().__init__('psm_controller')

        self.TCP: PoseStamped = None 
        self.Jaw: JointState = None
        self.Markers: list[Marker] = []
        self.MarkerRequester: Marker = None
        self.marker_num = 0

        self.tcp_offset = 0.008

        self.TCPSubscription = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.TCP_Callback, 10)
        self.JawSubscription = self.create_subscription(JointState, '/PSM1/jaw/measured_js', self.JAW_Callback,10) 
        self.MarkersSubscription = self.create_subscription(MarkerArray, 'interactice_markers',self.Markers_callback, 10)
        self.MarkerRequesterButtonSubscription = self.create_subscription(Marker, 'request_marker_button',self.MarkerRequester_callback, 10)

        self.TCPPub = self.create_publisher(PoseStamped, '/PSM1/servo_cp', 10)
        self.JawPub = self.create_publisher(JointState, '/PSM1/jaw/servo_jp', 10)

    def TCP_Callback(self, msg):
        self.TCP = msg

    def JAW_Callback(self, msg):
        self.Jaw = msg

    def Markers_callback(self, msg):
        last_marker_num = len(self.Markers)
        self.Markers = [marker for marker in msg.markers]
        self.Markers.sort(key=lambda marker: marker.id)
    
    def MarkerRequester_callback(self, msg):
        self.MarkerRequester = msg

    def wait_for_marker(self):
        while(len(self.Markers) == self.marker_num) and rclpy.ok():
            self.get_logger().info('Waiting for marker...')
            rclpy.spin_once(self)

        self.marker_num = len(self.Markers)

    def wait_for_Subscriptions(self):
         # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while (self.TCP is None or self.Jaw is None or self.MarkerRequester is None) and rclpy.ok():
            print(f"tcp: {self.TCP != None}")
            print(f"jaw: {self.Jaw != None}")
            print(f"marker requester: {self.MarkerRequester != None}")
            self.get_logger().info('Waiting for subsriptions...')
            rclpy.spin_once(self)
    
    def move_tcp_to(self, target, v, dt):
        self.wait_for_Subscriptions()
        msg = self.TCP

        pos_curr_np = np.array([self.TCP.pose.position.x*1000,
                                self.TCP.pose.position.y*1000,
                                self.TCP.pose.position.z*1000])
        pos_des_np = np.array(target)

        distance = np.linalg.norm(pos_des_np-pos_curr_np)/1000
        T = distance/v
        N = int(math.floor(T / dt))
        x = np.linspace(self.TCP.pose.position.x, target[0], N)
        y = np.linspace(self.TCP.pose.position.y, target[1], N)
        z = np.linspace(self.TCP.pose.position.z, target[2], N)

        self.loop_rate = self.create_rate(1.0 / dt, self.get_clock()) # Hz
        for i in range(N):
            if(not rclpy.ok()):
                break

            msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x = x[i]
            msg.pose.position.y = y[i]
            msg.pose.position.z = z[i]

            self.TCPPub.publish(msg)
            rclpy.spin_once(self)

    def move_jaw_to(self, target, omega, dt):
        self.wait_for_Subscriptions()

        msg = self.Jaw

        distance = np.linalg.norm(target-self.Jaw.position[0])
        T = distance/omega
        N = math.floor(T / dt)
        tr_jaw = np.linspace(self.Jaw.position[0], target, N)

        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        for i in range(N):
            if(not rclpy.ok()):
                break

            msg.header.stamp=self.get_clock().now().to_msg()
            msg.position = [tr_jaw[i]]
            self.JawPub.publish(msg)
            rclpy.spin_once(self)
    
    def grab_marker(self, v, omega, dt):
        self.wait_for_Subscriptions()

        self.move_jaw_to(target=0.8, omega=omega, dt=dt)

        marker_to_move = self.Markers[-1]

        self.move_tcp_to(target=[ marker_to_move.pose.position.x,
                                  marker_to_move.pose.position.y,
                                  marker_to_move.pose.position.z+self.tcp_offset], v=v, dt=dt)
        
        self.move_jaw_to(target=0.0, omega=omega, dt=dt)
    
    def move_marker_to(self, target, v, omega, dt):
        self.grab_marker(v=v, omega=omega, dt=dt)
        time.sleep(0.1)
        self.move_tcp_to(target=target, v=v, dt=dt)
        time.sleep(0.1)
        self.move_jaw_to(target=0.8, omega=omega, dt=dt)

    def request_marker(self, v, dt):
        self.wait_for_Subscriptions()
        self.move_tcp_to(target=[ self.MarkerRequester.pose.position.x,
                                  self.MarkerRequester.pose.position.y,
                                  self.MarkerRequester.pose.position.z+self.tcp_offset], v=v, dt=dt)
        time.sleep(0.2)

    def draw_shape(self, shape, v, omega, dt):
        targets = shapes[shape]
        for target in targets:
            self.request_marker(v=v, dt=dt)
            self.wait_for_marker()
            self.move_marker_to(target=target, v=v, omega=omega, dt=dt)

def main(args=None):
    rclpy.init(args=args)
    psm = PSM()

    v = 0.005
    omega = 0.1
    dt = 0.01
    home = [0.0, 0.0, -0.12]
    shape = "+"

    if(shape in shapes.keys()):
        psm.draw_shape(shape="X", v=v, omega=omega, dt=dt)
    
    else:
        psm.get_logger().info('The provided shape is invalid')

    #Home the arm
    psm.move_tcp_to(target=home, v=v, dt=dt)
    psm.move_jaw_to(0.0, omega=omega, dt=dt)

    psm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
