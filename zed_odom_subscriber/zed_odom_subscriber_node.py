import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import serial

# SPEED
LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.30
# CONSTANTS
DIFF_ANGLE_THRESHOLD = 1
DISTANCE_TO_TARGET_THRESHOLD = 0.3
# ACTIONS
LEFT_STEER = 0
RIGHT_STEER = 2
NO_STEER = 1


class ZedOdomSubscriber(Node):
    def __init__(self):
        super().__init__("zed_odom_subscriber")
        self.create_subscription(
            Odometry, "/zed2/zed_node/odom", self.zed_odom_listener_callback, 10
        )

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_all_callback, 10)
        self.rover_action_publisher = self.create_publisher(Int32, "/rover_action", 10)
        self.zed_twist_publisher = self.create_publisher(Twist, "/cmd_vel_zed", 10)
        self.current_waypoint = 0
        self.finished_mission = False
        self.waypoint_objects = [[5, 0], [5, -5], [0, -5], [0, 0]]
        # self.waypoint_objects = [[3, 0], [6, 0], [6, 6], [0, 3]]

        try:
            self.arduino = serial.Serial(
                port="/dev/ttyUSB0", baudrate=115200, timeout=1
            )
        except serial.SerialException as e:
            self.get_logger().info(f"An error occurred: {e}")

    def zed_odom_listener_callback(self, zed_msg):
        x_pos = zed_msg.pose.pose.position.x
        y_pos = zed_msg.pose.pose.position.y
        z_pos = zed_msg.pose.pose.position.z

        x_ori = zed_msg.pose.pose.orientation.x
        y_ori = zed_msg.pose.pose.orientation.y
        z_ori = zed_msg.pose.pose.orientation.z
        w_ori = zed_msg.pose.pose.orientation.w

        # self.get_logger().info(f"X POS: {x_pos}")
        # self.get_logger().info(f"Y POS: {y_pos}")
        # self.get_logger().info(f"Z POS: {z_pos}\n")
        # self.get_logger().info(f"X ORI: {x_ori}")
        # self.get_logger().info(f"Y ORI: {y_ori}")
        # self.get_logger().info(f"Z ORI: {z_ori}")
        # self.get_logger().info(f"W ORI: {w_ori}\n")

        self.get_logger().info(
            f"Current Position: X: {round(x_pos,3)} Y: {round(y_pos,3)}"
        )

        # rover_action = random.randint(0, 2)  # ONLY INTEGER
        rover_action = self.action_policy(
            x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori
        )
        linear, angular = self.action_interpreter(rover_action)
        self.publish_cmd_vel_zed(linear, angular)
        # self.publish_position(rover_action)

    def cmd_vel_all_callback(self, cmd_vel_msg):
        linear = cmd_vel_msg.linear.x
        angular = cmd_vel_msg.angular.z
        self.send_serial_command(linear=linear, angular=angular)

    def action_interpreter(self, rover_action):
        linear = LINEAR_SPEED
        if rover_action == 0:  # LEFT
            angular = -ANGULAR_SPEED
        elif rover_action == 2:  # RIGHT
            angular = ANGULAR_SPEED
        else:
            angular = 0.0
        return linear, angular
        # self.send_serial_command(linear=linear, angular=angular)

    def send_serial_command(self, linear, angular, reset=0.0):
        command_string = f"{round(linear,2)},{round(angular,2)},0,0,0,0,0,{reset}&\n"
        self.get_logger().info(
            f"Command String {command_string}\nCommand String Length: {len(command_string)}"
        )

        try:
            self.arduino.write(command_string.encode())
        except AttributeError as e:
            self.get_logger().info(f"AttributeError: {e}")

    # def publish_position(self, rover_action):
    #     rover_action_msg = Int32()
    #     rover_action_msg.data = rover_action
    #     self.rover_action_publisher.publish(rover_action_msg)

    def publish_cmd_vel_zed(self, linear=0.0, angular=0.0):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear
        cmd_vel_msg.angular.z = angular
        self.zed_twist_publisher.publish(cmd_vel_msg)

    def action_policy(self, x, y, z, qx, qy, qz, qw):
        # assuming the orientation data in the zed odom message are already in quaternions
        yaw = round(
            np.degrees(
                np.arctan2(2 * (qx * qy + qw * qz), 1 - 2 * (qy * qy + qz * qz))
            ),
            1,
        )
        target_position = self.waypoint_objects[self.current_waypoint]

        self.get_logger().info(f"Current Target Waypoint: {target_position}")
        self.get_logger().info(f"Current Waypoint #: {self.current_waypoint}\n")
        relDisX = round(target_position[0] - x, 1)
        relDisY = round(target_position[1] - y, 1)

        if relDisX > 0 and relDisY > 0:
            theta = np.arctan(relDisY / relDisX)
        elif relDisX > 0 and relDisY < 0:
            theta = 2 * np.pi + np.arctan(relDisY / relDisX)
        elif relDisX < 0 and relDisY < 0:
            theta = np.pi + np.arctan(relDisY / relDisX)
        elif relDisX < 0 and relDisY > 0:
            theta = np.pi + np.arctan(relDisY / relDisX)
        elif relDisX == 0 and relDisY > 0:
            theta = 1 / 2 * np.pi
        elif relDisX == 0 and relDisY < 0:
            theta = 3 / 2 * np.pi
        elif relDisY == 0 and relDisX > 0:
            theta = 0
        else:
            theta = np.pi

        relTheta = round(np.degrees(theta), 2)
        diffAngle = relTheta - yaw

        if diffAngle <= 180:
            diffAngle = round(diffAngle, 2)
        else:
            # diffAngle = round(360 - diffAngle, 2)
            diffAngle = round(diffAngle - 360.0, 2)

        if diffAngle >= DIFF_ANGLE_THRESHOLD:
            action = LEFT_STEER
        elif diffAngle < -DIFF_ANGLE_THRESHOLD:
            action = RIGHT_STEER
        else:
            action = NO_STEER

        self.get_logger().info(f"ACTION: {action}\n")
        self.get_logger().info(f"DIFF ANGLE: {diffAngle}\n")

        distance_to_target = np.sqrt(
            (x - target_position[0]) ** 2 + (y - target_position[1]) ** 2
        )

        if (
            self.current_waypoint == 3
            and distance_to_target < DISTANCE_TO_TARGET_THRESHOLD
        ):
            self.finished_mission = True

        if (
            self.current_waypoint <= 2
            and distance_to_target < DISTANCE_TO_TARGET_THRESHOLD
        ):
            self.current_waypoint += 1

        if self.finished_mission == True:
            self.current_waypoint = 0
            self.finished_mission = False

        return action


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
