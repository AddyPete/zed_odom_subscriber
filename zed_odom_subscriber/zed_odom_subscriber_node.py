import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import serial

# SPEED
LINEAR_SPEED = 0.25
ANGULAR_SPEED = 0.20
# CONSTANTS
DIFF_ANGLE_THRESHOLD = 15  # DEGREE
DISTANCE_TO_TARGET_THRESHOLD = 0.2  # METERS
# ACTIONS
# LEFT_STEER = 0
# RIGHT_STEER = 2
# NO_STEER = 1

rover_actions = {
    "Left Steer": 0,
    "Right Steer": 1,
    "No Steer": 2,
    "Soft Left Steer": 3,
    "Soft Right Steer": 4,
}

# SOFT_LEFT = 3
# SOFT_RIGHT = 4


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

        self.waypoint_objects = [[10, 0], [10, -10], [0, -10], [0, 0]]  # METERS
        # self.waypoint_objects = [[5, 0], [5, -5], [0, -5], [0, 0]]

        try:
            self.arduino = serial.Serial(
                port="/dev/ttyUSB0", baudrate=115200, timeout=1
            )
        except serial.SerialException as e:
            self.get_logger().info(f"An error occurred: {e}")

    def zed_odom_listener_callback(self, zed_msg):
        self.get_logger().info(f"Finished Mission? {self.finished_mission}\n ")
        if self.finished_mission == True:
            self.send_serial_command(0.0, 0.0)
            return
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
            f"Current Position: X: {round(x_pos,3)} Y: {round(y_pos,3)}\n"
        )

        # rover_action = random.randint(0, 2)  # ONLY INTEGER
        rover_action = self.action_policy(
            x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori
        )
        linear, angular = self.action_interpreter(rover_action)
        self.publish_cmd_vel_zed(linear, angular)
        # self.publish_position(rover_action)
        # self.get_logger().info("====================================")

    def cmd_vel_all_callback(self, cmd_vel_msg):
        linear = cmd_vel_msg.linear.x
        angular = cmd_vel_msg.angular.z
        self.send_serial_command(linear=linear, angular=angular)

    def action_interpreter(self, rover_action):
        linear = LINEAR_SPEED
        if rover_action == 0:  # LEFT
            angular = ANGULAR_SPEED
        elif rover_action == 2:  # RIGHT
            angular = -ANGULAR_SPEED
        elif rover_action == 3:
            angular = ANGULAR_SPEED / 2  # SOFT LEFT
        elif rover_action == 4:
            angular = -ANGULAR_SPEED / 2  # SOFT RIGHT
        else:
            angular = 0.0
        return linear, angular
        # self.send_serial_command(linear=linear, angular=angular)

    def send_serial_command(self, linear, angular, reset=0.0):
        command_string = f"{round(linear,2)},{round(-angular,2)},0,0,0,0,0,{reset}&\n"
        self.get_logger().info(
            f"Command String {command_string} | Length: {len(command_string)}"
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
        yaw = np.degrees(
            np.arctan2(2 * (qx * qy + qw * qz), 1 - 2 * (qy * qy + qz * qz))
        )

        target_position = self.waypoint_objects[self.current_waypoint]

        self.get_logger().info(
            f"Waypoint # {self.current_waypoint} | Current Target Waypoint: {target_position}\n"
        )
        # self.get_logger().info(f"Current Waypoint #: {self.current_waypoint}\n")
        relDisX = target_position[0] - x
        relDisY = target_position[1] - y

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

        relTheta = np.degrees(theta)
        diffAngle = relTheta - yaw

        if diffAngle <= 180:
            diffAngle = diffAngle
        else:
            # diffAngle = round(360 - diffAngle, 2)
            diffAngle = diffAngle - 360.0

        if diffAngle >= DIFF_ANGLE_THRESHOLD:
            rover_action = "Left Steer"  # LEFT_STEER
        elif diffAngle < -DIFF_ANGLE_THRESHOLD:
            rover_action = "Right Steer"  # RIGHT_STEER

        elif diffAngle >= DIFF_ANGLE_THRESHOLD / 2 and diffAngle < DIFF_ANGLE_THRESHOLD:
            rover_action = "Soft Left Steer"  # SOFT_LEFT
        elif (
            diffAngle <= -DIFF_ANGLE_THRESHOLD / 2 and diffAngle > -DIFF_ANGLE_THRESHOLD
        ):
            rover_action = "Soft Right Steer"  # SOFT_RIGHT
        else:
            rover_action = "No Steer"  # NO_STEER

        action_code = rover_actions[rover_action]

        self.get_logger().info(f"ACTION: {action_code} | {rover_action}\n")
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

        # if self.finished_mission == True:
        #     self.current_waypoint = 0
        #     self.finished_mission = False

        return action_code


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
