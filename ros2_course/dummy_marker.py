import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class DummyMarker(Node):
    def __init__(self, position):
        super().__init__('minimal_publisher')
        self.position = position
        self.publisher_ = self.create_publisher(Marker, 'dummy_target_marker', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        i = 0

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = 'camera'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dvrk_viz"
        marker.id = self.i
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.008
        marker.scale.y = 0.008
        marker.scale.z = 0.008
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0;

        self.publisher_.publish(marker)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    marker_publisher = DummyMarker([0.00687728, 0.06412506, 0.27155235])
    rclpy.spin(marker_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()