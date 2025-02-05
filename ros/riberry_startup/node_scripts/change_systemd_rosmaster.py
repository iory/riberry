#!/usr/bin/env python3

from multiprocessing import Manager
from multiprocessing import Process
import os
import re
import socket
import subprocess

import rospy
from std_msgs.msg import String


def get_ros_namespace():
    """
    Get the hostname and convert it to a ROS-compliant namespace format.
    """
    hostname = socket.gethostname().lower().replace('-', '_')
    hostname = re.sub(r'[^a-z0-9_]', '_', hostname)
    return f'robot_{hostname}' if hostname[0].isdigit() else hostname


def run_command(command_list):
    try:
        subprocess.run(command_list, capture_output=True, text=True, check=True, timeout=10)
        return True
    except subprocess.TimeoutExpired:
        rospy.logwarn(f"Command timeout: {' '.join(command_list)}")
        return False
    except subprocess.CalledProcessError:
        rospy.logerr(f"Command failed: {' '.join(command_list)}")
        return False


class SystemdRosMasterChanger:
    def __init__(self, next_ros_master_ip):
        self.services = ["display_information.service", "riberry_startup.service", "user.service"]
        self.next_ros_master_ip = next_ros_master_ip
        self.process_running = True
        self.register_subscriber(get_ros_namespace() if self.next_ros_master_ip.value != "localhost" else None)

    def register_subscriber(self, ros_namespace=None):
        topic_name = f"{ros_namespace}/pairing_information" if ros_namespace else "pairing_information"
        self.sub = rospy.Subscriber(topic_name, String, self.callback, queue_size=1)
        rospy.loginfo(f"Start subscribing to topic /{topic_name}")

    def unregister_subscriber(self):
        self.sub.unregister()

    def set_ros_env(self, ros_master_ip):
        run_command(['systemctl', 'set-environment', f'RIBERRY_ROS_MASTER_IP={ros_master_ip}'])
        if ros_master_ip != "localhost":
            # Avoid topic name conflicts when connecting to other computers
            run_command(['systemctl', 'set-environment', f'ROS_NAMESPACE={get_ros_namespace()}'])

    def unset_ros_env(self):
        run_command(['systemctl', 'unset-environment', 'RIBERRY_ROS_MASTER_IP'])
        run_command(['systemctl', 'unset-environment', 'ROS_NAMESPACE'])

    def restart_services(self):
        for service in self.services:
            if not run_command(['systemctl', 'stop', service]):
                rospy.logwarn(f"{service} could not be stopped, forcing kill")
                run_command(['systemctl', 'kill', service])
            run_command(['systemctl', 'start', service])

    def systemd_rossetmaster(self, ros_master_ip):
        """
        Change ROS_MASTER_URI of specified systemd services.

        Args:
            ros_master_ip (str): ROS master IP address to set (e.g., '192.168.96.0').
        """
        rospy.loginfo(f"Change the following services' ROS_MASTER_URI to http://{ros_master_ip}:11311")
        rospy.loginfo(f'{self.services}')
        self.set_ros_env(ros_master_ip)
        self.restart_services()
        self.unset_ros_env()

    def callback(self, msg):
        if self.next_ros_master_ip.value != msg.data:
            self.systemd_rossetmaster(msg.data)
            # Pass next_ros_master_ip to next process
            self.next_ros_master_ip.value = msg.data
            # Restart this node by another process,
            # because pairing_information topic is published in another roscore
            self.unregister_subscriber()
            self.process_running = False

    def run(self):
        rate = rospy.Rate(2)
        while self.process_running and not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":

    def main_process(next_ros_master_ip):
        os.environ['ROS_MASTER_URI'] = f'http://{next_ros_master_ip.value}:11311'
        rospy.init_node("change_systemd_rosmaster", anonymous=True)
        rospy.loginfo(f"[change_systemd_rosmaster] Start with http://{next_ros_master_ip.value}:11311")
        changer = SystemdRosMasterChanger(next_ros_master_ip)
        changer.run()
        rospy.loginfo("[change_systemd_rosmaster] rospy node finished")

    with Manager() as manager:
        next_ros_master_ip = manager.Value('s', 'localhost')
        while True:
            # Use rospy inside process to completely reset rospy.init_node()
            # after process finished
            process = Process(target=main_process, args=(next_ros_master_ip,))
            process.start()
            process.join()
