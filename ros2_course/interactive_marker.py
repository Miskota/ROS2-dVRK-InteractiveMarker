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
import tf2_ros
import tf2_geometry_msgs

class InteractiveMarker(Node):

    def __init__(self, initial_pos):
        super().__init__('interactive_marker')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        
        
        #self.position = np.array(initial_pos)
        self.position = None
        
        self.tcp_pos = None
        self.jaw_pos = None
        self.marker_attached = False
        self.marker_offset = None

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.i = 0

        # Get TCP position
        self.tcp_subscription = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_tcp,
            10
        )

        # Get jaws position
        self.joint_subscription = self.create_subscription(
            JointState,
            '/PSM1/jaw/measured_js',
            self.cb_jaw,
            10
        )

        # Send Marker to RViz
        self.publisher_ = self.create_publisher(
            Marker,
            'dummy_target_marker',
            10
        )




    #def cb_tcp(self, msg):
    #    self.tcp_pos = np.array([
    #        msg.pose.position.x,
    #        msg.pose.position.y,
    #        msg.pose.position.z
    #    ])
    #
    #    # Different coordinates
    #    self.tcp_frame = msg.header.frame_id
    #    self.get_logger().info(f'TCP frame: {self.tcp_frame}')
    
    def cb_tcp(self, msg):
        self.tcp_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.tcp_frame = msg.header.frame_id

        # Initialize marker position on first TCP message
        if self.position is None:
            self.position = np.copy(self.tcp_pos)
            self.get_logger().info(f"Marker initialized at TCP position: {self.position}")

        # If marker is attached, move it with the TCP
        if self.marker_attached:
            self.position = self.tcp_pos + self.marker_offset
    
    '''
    def cb_tcp(self, msg):
        

        try:
            # Lookup transform from TCP frame to 'camera'
            transform = self.tf_buffer.lookup_transform(
                'camera',  # target frame
                msg.header.frame_id,  # source frame (likely 'PSM1_psm_base_link')
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Transform the pose
            transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)

            self.tcp_pos = np.array([
                transformed.pose.position.x,
                transformed.pose.position.y,
                transformed.pose.position.z
            ])
            self.tcp_frame = 'camera'  # Now it's in camera frame

            if self.position is None:
                self.position = np.copy(self.tcp_pos)
                self.get_logger().info(f"Marker initialized at TCP position: {self.position}")


        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            self.tcp_pos = None
    '''

    def cb_jaw(self, msg):
        self.jaw_position = msg.position[0]




    def timer_callback(self):
        if self.position is None or self.tcp_pos is None or self.jaw_position is None:
            return

        if self.tcp_pos is None or self.jaw_position is None or self.tcp_frame is None:
            return
        
        if self.tcp_pos is None or self.jaw_position is None:
            return
        
        # Decide if the jaws are open or closed
        jaw_closed = self.jaw_position < 0.05
        

        # Calculate the distance between the TCP and Marker
        dist = np.linalg.norm(self.position - self.tcp_pos)

        # Marker is attached if it's close
        # dist is in meters
        if dist < 0.01 and jaw_closed and not self.marker_attached:

            self.marker_attached = True
            self.marker_offset = self.position - self.tcp_pos
            self.get_logger().info('Marker attached to TCP')

        # Marker is free
        if self.marker_attached and not jaw_closed:
            self.marker_attached = False
            self.get_logger().info('Marker released')
        
        # Update position if attached
        if self.marker_attached:
            self.position = self.tcp_pos + self.marker_offset

        
        marker = Marker()
        #marker.header.frame_id = 'PSM1_base'
        #marker.header.frame_id = 'camera'
        marker.header.frame_id = 'PSM1_psm_base_link'
        #marker.header.frame_id = self.tcp_frame
        #marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dvrk_viz'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY

        marker.pose.position.x = float(self.position[0])
        marker.pose.position.y = float(self.position[1])
        marker.pose.position.z = float(self.position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.008
        marker.scale.y = 0.008
        marker.scale.z = 0.008


        marker.color.a = 1.0

        # Red sphere: Attached
        if self.marker_attached:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        # Green sphere: Free
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        # Send marker
        self.publisher_.publish(marker)
        self.i += 1


def main(args = None):
    rclpy.init(args = args)

    initial_pos = [0.00687728, 0.06412506, 0.27155235]
    marker_node = InteractiveMarker(initial_pos)

    #marker_node = InteractiveMarker()

    rclpy.spin(marker_node)
    
    marker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()