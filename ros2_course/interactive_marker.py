import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
#from std_msgs.msgs.msg import Header
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np


class InteractiveMarker(Node):

    def __init__(self):
        super().__init__('interactive_marker')

        # Create Marker object here
        self.marker = Marker()
        self.marker.header.frame_id = 'PSM1_psm_base_link'
        self.marker.ns = 'dvrk_viz'
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.MODIFY
        self.marker.scale.x = 0.008
        self.marker.scale.y = 0.008
        self.marker.scale.z = 0.008
        self.marker.pose.orientation.w = 1.0
        self.marker.color.a = 1.0




        self.position = None
        
        self.tcp_pos = None
        self.jaw_pos = None
        self.marker_attached = False
        self.marker_offset = None

        self.timer = self.create_timer(0.05, self.timer_callback)
        #self.i = 0

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
    



    def cb_jaw(self, msg):
        self.jaw_position = msg.position[0]



    def timer_callback(self):
        if self.position is None or self.tcp_pos is None or self.jaw_position is None:
            return

        if self.tcp_pos is None or self.jaw_position is None or self.tcp_frame is None:
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
        # Offset: The marker doesn't snap to the TCP
        if self.marker_attached:
            self.position = self.tcp_pos + self.marker_offset

        '''
        marker = Marker()
        marker.header.frame_id = 'PSM1_psm_base_link' # The correct frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dvrk_viz'
        marker.id = 0 # Set the id to a constant value, otherwise it'll clone itself
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
        '''
        self.marker.pose.position.x = self.position[0]
        self.marker.pose.position.y = self.position[1]
        self.marker.pose.position.z = self.position[2]


        # Red sphere: Attached
        if self.marker_attached:
            self.marker.color.r = 1.0
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0
        # Green sphere: Free
        else:
            self.marker.color.r = 0.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0

        # Send marker
        self.publisher_.publish(self.marker)


def main(args = None):
    rclpy.init(args = args)

    marker_node = InteractiveMarker()

    rclpy.spin(marker_node)
    
    marker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()