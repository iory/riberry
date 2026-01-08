#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import rospy
from std_msgs.msg import Bool


def get_key(settings, timeout=0.01):
    """
    Non-blocking key input.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rospy.init_node('key_trigger_node')

    # Generic topic name 'trigger', intended to be remapped in launch file
    pub = rospy.Publisher('trigger', Bool, queue_size=1)

    settings = termios.tcgetattr(sys.stdin)

    print("--------------------------------")
    print("Key Trigger Node Running")
    print(f"Topic: {pub.name}")
    print("HOLD [SPACE] to publish True.")
    print("RELEASE to publish False.")
    print("--------------------------------")

    last_msg = False
    last_press_time = 0
    last_pub_time = 0
    # To prevent chattering
    HOLD_TIMEOUT = 0.5

    try:
        while not rospy.is_shutdown():
            # Check key input with short timeout (Reduced to 0.02 for better responsiveness)
            key = get_key(settings, timeout=0.02)

            current_time = time.time()
            is_pressed = False

            if key == ' ':
                last_press_time = current_time
                is_pressed = True
            elif key == '\x03':  # Ctrl-C
                break
            else:
                # Keep True if within timeout (anti-chattering)
                if current_time - last_press_time < HOLD_TIMEOUT:
                    is_pressed = True
                else:
                    is_pressed = False

            # Logic to prevent flooding the subscriber
            if is_pressed != last_msg:
                # State changed: Log and Publish IMMEDIATELY
                if is_pressed:
                    rospy.loginfo(f"Trigger {pub.name}: Active (True)")
                else:
                    rospy.loginfo(f"Trigger {pub.name}: Inactive (False)")

                pub.publish(is_pressed)
                last_pub_time = current_time
                last_msg = is_pressed
            else:
                # State unchanged: Publish at 10Hz (0.1s interval) as heartbeat
                # This prevents lagging caused by queue flooding
                if current_time - last_pub_time > 0.1:
                    pub.publish(is_pressed)
                    last_pub_time = current_time
    except Exception as e:
        print(e)
    finally:
        # Ensure False is published on exit
        pub.publish(False)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
