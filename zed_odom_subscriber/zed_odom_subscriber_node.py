import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry


class ZedOdomSubscriber(Node):
    def __init__(self):
        super().__init__("zed_odom_subscriber")
        self.subscription = self.create_subscription(
            Odometry, "/zed2i/zed_node/odom", self.listener_callback, 10
        )
        self.rover_action_publisher = self.create_publisher(Int32, "/rover_action", 10)

    def listener_callback(self, zed_msg):
        x_pos = zed_msg.pose.pose.position.x
        y_pos = zed_msg.pose.pose.position.y
        z_pos = zed_msg.pose.pose.position.z

        x_ori = zed_msg.pose.pose.orientation.x
        y_ori = zed_msg.pose.pose.orientation.y
        z_ori = zed_msg.pose.pose.orientation.z
        w_ori = zed_msg.pose.pose.orientation.w

        self.get_logger().info(f"X POS: {x_pos}")
        self.get_logger().info(f"Y POS: {y_pos}")
        self.get_logger().info(f"Z POS: {z_pos}\n")
        self.get_logger().info(f"X ORI: {x_ori}")
        self.get_logger().info(f"Y ORI: {y_ori}")
        self.get_logger().info(f"Z ORI: {z_ori}")
        self.get_logger().info(f"W ORI: {w_ori}\n")

        rover_action = random.randint(0, 2)  # ONLY INTEGER
        self.publish_position(rover_action)

    def publish_position(self, rover_action):
        rover_action_msg = Int32()
        rover_action_msg.data = rover_action
        self.rover_action_publisher.publish(rover_action_msg)


def main(args=None):
    rclpy.init(args=args)

    zed_odom_subscriber = ZedOdomSubscriber()

    rclpy.spin(zed_odom_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    zed_odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
