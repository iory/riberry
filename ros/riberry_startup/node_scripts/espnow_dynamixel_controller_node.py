#!/usr/bin/env python3

import threading

import rospy
from std_msgs.msg import String

from riberry.com.uart_base import usb_devices
from riberry.esp_now_pairing import ESPNowPairing
from riberry.esp_now_pairing import find_USB_pairing_devices
from riberry.esp_now_pairing import Role


class EspNowControllerNode:
    """
    ROS node to control ESP-NOW devices.
    """
    def __init__(self):
        rospy.init_node('esp_now_controller', anonymous=True)
        # Publish one of the keys from COMMAND_MAP to the /motor_command topic.
        rospy.Subscriber('motor_command', String, self.command_callback)

        self.esp_now_pairing = None
        self.stop_timer = None
        self.lock = threading.Lock()

        # Keep a list of ports from the previous scan.
        self.prev_ports = []

        self.COMMAND_MAP = {
            "Forward": "0.0.0.0",
            "Stop": "127.0.0.0",
            "Reverse": "255.0.0.0"
        }

        self.discovery_thread = threading.Thread(target=self.manage_device_connection, daemon=True)
        self.discovery_thread.start()

    def manage_device_connection(self):
        """
        Manages device connection and disconnection.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                current_ports = usb_devices()
            except Exception as e:
                rospy.logerr(f"Error while scanning for USB devices: {e}")
                current_ports = []

            # --- Check if the connected device has been disconnected ---
            with self.lock:
                if self.esp_now_pairing is not None:
                    connected_port = self.esp_now_pairing.com.serial.port_name
                    if connected_port not in current_ports:
                        rospy.logwarn(f"Detected disconnection of device {connected_port}. Waiting for reconnection.")
                        self.esp_now_pairing = None

            # --- Process only newly connected ports ---
            # Search for new ports only if no device is connected.
            if self.esp_now_pairing is None:
                new_ports = [p for p in current_ports if p not in self.prev_ports]

                if new_ports:
                    rospy.loginfo(f"New USB ports detected: {new_ports}")
                    pairing_devices = find_USB_pairing_devices(new_ports)

                    main_device_info = None
                    for dev in pairing_devices:
                        if dev and dev["role"] == Role.Main:
                            main_device_info = dev
                            break

                    if main_device_info:
                        com_port = main_device_info["com"]
                        rospy.loginfo(f"Controller device found: {com_port}. Attempting to connect.")
                        with self.lock:
                            try:
                                # Confirm it's still None (for thread safety).
                                if self.esp_now_pairing is None:
                                    self.esp_now_pairing = ESPNowPairing(com=com_port, role=Role.Main)
                                    rospy.loginfo(f"Successfully connected to device {com_port}.")
                            except Exception as e:
                                rospy.logerr(f"Failed to connect to device {com_port}: {e}")

            # Finally, save the current list of ports.
            self.prev_ports = current_ports
            rate.sleep()

    def send_command(self, command_ip):
        """
        Sends the actual command to the ESP-NOW device.
        """
        with self.lock:
            if self.esp_now_pairing:
                try:
                    self.esp_now_pairing.set_pairing_info(command_ip)
                    self.esp_now_pairing.pairing()
                    rospy.loginfo(f"Command sent successfully: {command_ip}")
                except Exception as e:
                    rospy.logerr(f"Failed to send command: {e}. The device may have been disconnected.")
                    self.esp_now_pairing = None
                    rospy.logwarn("Connection with the device has been reset. Attempting to reconnect.")
            else:
                rospy.logwarn("Received a command, but no device is connected.")

    def send_stop_command_auto(self):
        rospy.loginfo("5 seconds have passed. Stopping the motor for safety.")
        self.send_command(self.COMMAND_MAP["Stop"])

    def command_callback(self, msg):
        command = msg.data
        rospy.loginfo(f"Command received: '{command}'")

        if self.stop_timer and self.stop_timer.is_alive():
            self.stop_timer.cancel()

        if command in self.COMMAND_MAP:
            command_ip = self.COMMAND_MAP[command]
            self.send_command(command_ip)

            if command in ["Forward", "Reverse"]:
                self.stop_timer = threading.Timer(5.0, self.send_stop_command_auto)
                self.stop_timer.start()
        else:
            rospy.logwarn(f"Undefined command: '{command}'")

    def run(self):
        rospy.loginfo("ESP-NOW controller node has started.")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = EspNowControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
