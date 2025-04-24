import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import PoseStamped
#from std_msgs.msgs.msg import Header
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np
from std_msgs.msg import Float32MultiArray
import threading

class RobotArm(Node):
    
    def __init__(self):
        super().__init__('arm_node')
        
        self.measured_cp = None
        self.measured_jaw = None
        

        self.servo_jaw_pub = self.create_publisher(
            JointState, '/PSM1/jaw/servo_jp', 10
        )

        self.servo_cp_pub = self.create_publisher(
            PoseStamped,
            '/PSM1/servo_cp',
            10
        )

        self.measured_cp_subscription = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_measured_cp,
            10
        )

        self.measured_jaw_subscription = self.create_subscription(
            JointState,
            '/PSM1/jaw/measured_js',
            self.cb_measured_jaw,
            10
        )


        self.marker = None
        self.marker_subscription = self.create_subscription(
            Marker,
            '/dummy_target_marker',
            self.cb_marker,
            10
        )

        
        self.controller = None
        self.controller_subscription = self.create_subscription(
            Float32MultiArray,
            '/controller_cmd',
            self.controller_cb,
            10
        )
        
        '''
        self.stick = None
        self.stick_subscription = self.create_subscription(
            Float32MultiArray,
            '/stick_cmd',
            self.stick_cb,
            10
        )

        self.trigger = None
        self.trigger_subscription = self.create_subscription(
            Float32MultiArray,
            '/trigger_cmd',
            self.trigger_cb,
            10
        )
        '''

    '''
    def stick_cb(self, msg):
        dx = msg.data[0] * 0.01
        dy = msg.data[1] * 0.01
        dz = msg.data[2] * 0.01

        print(f"dx: {dx}, dy: {dy}, dz: {dz}")

        if self.measured_cp is not None:
            target = np.array([
                self.measured_cp.pose.position.x + dx,
                self.measured_cp.pose.position.y + dy,
                self.measured_cp.pose.position.z + dz
            ])
            print(f"Moving TCP to {target}")
            self.move_tcp_to(target, v=0.02, dt=0.05)


    def trigger_cb(self, msg):
        jaw_target = (msg.data[0] + 1) * 0.4

        if self.measured_jaw is not None:
            print(f"Moving jaws to {jaw_target}")
            self.move_jaw_to(jaw_target, omega=1.0, dt=0.05)
    '''


    def controller_cb(self, msg):
        if len(msg.data) < 4:
            return
    
        dx = msg.data[0] * 0.01
        dy = msg.data[1] * 0.01
        dz = msg.data[2] * 0.01
        jaw_target = (msg.data[3] + 1) * 0.4

        # Ha nincs változás a stick-ben és az állkapocs cél is közel van, semmit se csinálunk
        #if abs(dx) < 1e-3 and abs(dy) < 1e-3 and abs(dz) < 1e-3:
        #    if self.measured_jaw and abs(self.measured_jaw.position[0] - jaw_target) < 1e-3:
        #        self.get_logger().debug("No input change, skipping controller_cb")
        #        return

        print(f"dx: {dx}, dy: {dy}, dz: {dz}, jaw_target: {jaw_target}")
        
        if self.measured_cp is not None:
            target = np.array([
                self.measured_cp.pose.position.x + dx,
                self.measured_cp.pose.position.y + dy,
                self.measured_cp.pose.position.z + dz
            ])
            print(f"Moving TCP to {target}")
            self.move_tcp_to(target, v=0.02, dt=0.05)

        if self.measured_jaw is not None:
            print(f"Moving jaws to {jaw_target}")
            self.move_jaw_to(jaw_target, omega=1.0, dt=0.05)


    def cb_marker(self, msg):
        self.marker = msg
        print(self.marker)

    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg
        print(self.measured_jaw)    
    
    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        print(self.measured_cp)


    def move_jaw_to(self, target, omega, dt):
        #loop_rate = self.create_rate(100, self.get_clock())

        while self.measured_jaw is None and rclpy.ok():
            self.get_logger().info('Waiting for pose')
            print('Asd')
            #rclpy.spin_once(self)
        
        angle_curr = self.measured_jaw.position[0]
        angle_des = target

        # T = d/v = | rc_curr - r_des | / v

        d = abs(angle_des - angle_curr)

        T = d / omega
        #N = int(math.floor(abs(T / dt)))
        N = max(1, int(math.floor(abs(T / dt))))

        tr_jaw = np.linspace(angle_curr, angle_des, N)

        print(f"Moving jaws to {angle_des}...")
        loop_rate = self.create_rate(1.0/dt, self.get_clock())
        for i in range(N):
            msg = self.measured_jaw
            msg.position = [tr_jaw[i]]
            self.servo_jaw_pub.publish(msg)
            #rclpy.spin_once(self)
        print("Moving jaw completed.")
        #self.servo_cp_pub.publish(msg)





    def move_tcp_to(self, target, v=0.02, dt=0.05):
        if self.measured_cp is None:
            self.get_logger().warning('No measured_cp available!')
            return

        r_curr = np.array([self.measured_cp.pose.position.x,
                        self.measured_cp.pose.position.y,
                        self.measured_cp.pose.position.z])
        r_des = np.array(target)

        d = np.linalg.norm(r_des - r_curr)
        if d < 1e-4:
            self.get_logger().info('Target position is too close, skipping motion.')
            return

        T = d / v
        N = int(np.floor(T / dt))

        self.trajectory_tcp = [
            (float(x), float(y), float(z)) for x, y, z in zip(
                np.linspace(r_curr[0], r_des[0], N),
                np.linspace(r_curr[1], r_des[1], N),
                np.linspace(r_curr[2], r_des[2], N)
            )
        ]
        self.get_logger().info(f"TCP trajectory with {N} steps initialized.")

        # Timer alapú mozgás
        self.tcp_timer = self.create_timer(dt, self.tcp_timer_cb)

    def tcp_timer_cb(self):
        if not self.trajectory_tcp:
            self.tcp_timer.cancel()
            return

        pos = self.trajectory_tcp.pop(0)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
        msg.pose.orientation = self.measured_cp.pose.orientation  # Megőrizzük az eredeti orientációt

        self.servo_cp_pub.publish(msg)


    '''
    def grasp_marker(self, v, omega, dt):
        loop_rate = self.create_rate(100, self.get_clock())
        while self.marker is None and rclpy.ok():
            self.get_logger().info('Waiting for marker...')
            rclpy.spin_once(self)

        print(f"Marker position: {self.marker.pose.position}")

        

        # Open jaws
        self.move_jaw_to(0.8, omega, dt)

        # Go to marker
        target_cam = np.array([self.marker.pose.position.x,
        self.marker.pose.position.y,
        self.marker.pose.position.z])

        # Hand-eye transformation
        t_base_cam = np.array([0.18, 0.03, 0.01])
        R_base_cam = R.from_euler('xyz', [155.0, -45.0, -145.0], degrees=True).as_matrix()


        T_base_cam = np.eye(4)
        T_base_cam[:3,:3] = R_base_cam
        T_base_cam[:3,3] = t_base_cam
        print(f"Homogenous transformation matrix T_base_cam: {T_base_cam}")

        target_cam_h = np.append(target_cam, 1)
        print(f"Target_cam homogenous coordinates: {target_cam_h}")

        target_base_h = T_base_cam @ target_cam_h
        print(f"Target_base homogenous coordinates: {target_base_h}")
        
        target_base_h = target_base_h + np.array([0.0, 0.0, 0.008, 0.0])
        target_base = target_base_h[:3]
        self.move_tcp_to(target_base, v, dt)

        # Close jaws
        self.move_jaw_to(0, omega, dt)
'''

def main(args=None):
    rclpy.init(args=args)
    arm = RobotArm()
    rclpy.spin(arm)
    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
