import numpy as np
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

class PSM(Node):

    def __init__(self):
        super().__init__('psm_controller')

        self.TCP: PoseStamped = None 
        self.Jaw: JointState = None
        self.Marker: Marker = None

        self.TCPSubscription = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.TCP_Callback, 10)
        self.JawSubscription = self.create_subscription(JointState, '/PSM1/jaw/measured_js', self.JAW_Callback,10) 
        self.DummyMarkerSubscription = self.create_subscription(Marker, 'dummy_target_marker',self.Marker_callback, 10)

        self.TCPPub = self.create_publisher(PoseStamped, '/PSM1/servo_cp', 10)
        self.JawPub = self.create_publisher(JointState, '/PSM1/jaw/servo_jp', 10)

    def TCP_Callback(self, msg):
        self.TCP = msg

    def JAW_Callback(self, msg):
        self.Jaw = msg

    def Marker_callback(self, msg):
        self.Marker = msg

    def wait_for_Subscriptions(self):
         # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while (self.TCP is None or self.Jaw is None or self.Marker is None) and rclpy.ok():
            print(f"tcp: {self.TCP != None}")
            print(f"jaw: {self.Jaw != None}")
            print(f"marker: {self.Marker != None}")
            self.get_logger().info('Waiting for subsriptions...')
            rclpy.spin_once(self)

    def move_tcp_to(self, target, v, dt):
        self.wait_for_Subscriptions()

        msg = self.TCP

        pos_curr_np = np.array([self.TCP.pose.position.x,
                                self.TCP.pose.position.y,
                                self.TCP.pose.position.z])
        pos_des_np = np.array(target)

        distance = np.linalg.norm(pos_des_np-pos_curr_np)
        T = distance/v
        N = math.floor(T / dt)
        x = np.linspace(self.TCP.pose.position.x, target[0], N)
        y = np.linspace(self.TCP.pose.position.y, target[1], N)
        z = np.linspace(self.TCP.pose.position.z, target[2], N)

        loop_rate = self.create_rate(100, self.get_clock()) # Hz
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

        self.move_tcp_to(target=[ self.Marker.pose.position.x,
                                  self.Marker.pose.position.y,
                                  self.Marker.pose.position.z+0.008], v=v, dt=dt)
        
        self.move_jaw_to(target=0.0, omega=omega, dt=dt)

def main(args=None):
    rclpy.init(args=args)
    psm = PSM()

    #Reset the arm
    psm.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
    psm.move_jaw_to(0.0, 0.1, 0.01)

    # wait one second
    time.sleep(1)

    # grab the marker
    psm.grab_marker(v=0.01, omega=0.05, dt=0.01)

    # wait three seconds
    time.sleep(3)
    # release the marker
    psm.move_jaw_to(0.8, 0.1, 0.01)

    psm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
