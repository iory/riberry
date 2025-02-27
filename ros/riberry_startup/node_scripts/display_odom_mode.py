#!/usr/bin/env python3

import socket

from colorama import Fore
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Float32

from riberry.com.base import PacketType
from riberry.mode import Mode
from riberry.network import get_ip_address
from riberry.network import get_ros_master_ip


class DisplayOdomMode(Mode):
    def __init__(self):
        super().__init__()

        self.min_battery_voltage = rospy.get_param("~min_battery_voltage", 14.0)
        self.max_battery_voltage = rospy.get_param("~max_battery_voltage", 16.0)
        self.battery_pub = rospy.Publisher("~remaining_battery", Float32, queue_size=1)
        rospy.Subscriber(
            "uav/cog/odom", Odometry, callback=self.odometory_cb, queue_size=1
        )
        rospy.Subscriber(
            "~battery_voltage", Float32, callback=self.battery_cb, queue_size=1
        )
        self.odom = Odometry()

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def battery_cb(self, msg):
        rem = max(msg.data - self.min_battery_voltage, 0.0) / (
            self.max_battery_voltage - self.min_battery_voltage)
        self.battery_pub.publish(Float32(data=rem))

    def odometory_cb(self, msg):
        self.odom = msg

    def timer_callback(self, event):
        if self.mode == "DisplayInformationMode":
            self.write(self.display_network_information())
        elif self.mode == "DisplayOdomMode":
            self.write(self.display_odom_information())

    def display_network_information(self):
        ip = get_ip_address()
        if ip is None:
            ip = "no connection"
        ip_str = f"{socket.gethostname()}:\n{Fore.YELLOW}{ip}{Fore.RESET}"
        master_str = "ROS_MASTER:\n" + Fore.RED + f"{get_ros_master_ip()}" + Fore.RESET
        sent_str = chr(PacketType.TEXT)
        sent_str += f"{ip_str}\n{master_str}\n"
        return sent_str

    def display_odom_information(self):
        # Extract position information (3 significant digits)
        position = self.odom.pose.pose.position
        position_str = (
            "Position:\n"
            + f"  x: {position.x:.3f}\n"
            + f"  y: {position.y:.3f}\n"
            + f"  z: {position.z:.3f}"
        )

        # Extract orientation information (3 significant digits)
        orientation = self.odom.pose.pose.orientation
        orientation_str = (
            "Orientation:\n"
            + f"  x: {orientation.x:.3f}\n"
            + f"  y: {orientation.y:.3f}\n"
            + f"  z: {orientation.z:.3f}\n"
            + f"  w: {orientation.w:.3f}"
        )

        # Concatenate and return as a single formatted string
        return chr(PacketType.DISPLAY_ODOM_MODE) + f"{position_str}\n{orientation_str}"


if __name__ == "__main__":
    rospy.init_node("display_battery_mode")
    scm = DisplayOdomMode()
    rospy.spin()
