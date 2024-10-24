#!/usr/bin/env python3

from enum import Enum
import json
import numpy as np
import os
from pathlib import Path

from kxr_controller.kxr_interface import KXRROSRobotInterface
from riberry.i2c_base import I2CBase
import rospy
from skrobot.model import RobotModel
from std_msgs.msg import Int32
from std_msgs.msg import String


class State(Enum):
    WAIT = 0
    RECORD = 1
    PLAY = 2

class TeachingMode(I2CBase):
    def __init__(self, i2c_addr, lock_path="/tmp/teaching_mode.lock"):
        super().__init__(i2c_addr)
        # Create robot model to control pressure
        robot_model = RobotModel()
        namespace = ""
        robot_model.load_urdf_from_robot_description(namespace + "/robot_description_viz")
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )
        self.joint_names = self.ri.robot.joint_names
        # Teaching motion json
        self.json_filepath = os.path.join(str(Path.cwd()), 'teaching_motion.json')
        self.motion = []
        # Button and mode callback
        self.mode = None
        rospy.Subscriber(
            "/atom_s3_button_state",
            Int32, callback=self.button_cb, queue_size=1)
        rospy.Subscriber(
            "/atom_s3_mode", String, callback=self.mode_cb, queue_size=1)
        self.prev_state = None
        self.state = State.WAIT

    def mode_cb(self, msg):
        self.mode = msg.data

    def button_cb(self, msg):
        """
        Manage state by click
Wait -> (Single-click) -> Record  -> (Single-click) -> Finish recording -> Wait
Wait -> (Double-click) -> Play -> (Double-click) -> Abort -> Wait
"""
        if self.mode != "TeachingMode":
            return
        if msg.data == 3:
            self.ri.servo_off()
        if self.prev_state != self.state:
            sent_str = 'State: {}'.format(self.state.name)
            rospy.loginfo(sent_str)
        self.prev_state = self.state
        # State transition
        if self.state == State.WAIT:
            if msg.data == 1:
                self.state = State.RECORD
            elif msg.data == 2:
                self.state = State.PLAY
        elif self.state == State.RECORD:
            if msg.data == 1:
                self.state = State.WAIT
        elif self.state == State.PLAY:
            if msg.data == 2:
                self.state = State.WAIT

    def add_motion(self):
        """
        Add current pose to self.motion
"""
        if len(self.motion) == 0:
            self.start_time = rospy.Time.now()
        joint_states = {}
        av = self.ri.angle_vector()
        for j, a in zip(self.joint_names, av):
            joint_states[str(j)] = float(a)
        now = rospy.Time.now()
        elapsed_time = (now - self.start_time).to_sec()
        self.motion.append({
            'time': elapsed_time,
            'joint_states': joint_states,
        })
        rospy.loginfo('Add new joint states')
        rospy.loginfo(f'Time: {elapsed_time}, joint_states: {joint_states}')

    def record(self):
        self.motion = []
        with open(self.json_filepath, mode='w') as f:
            rospy.loginfo(f'Start saving motion to {self.json_filepath}')
            while not rospy.is_shutdown():
                # Finish recording
                if self.state == State.WAIT:
                    break
                self.add_motion()
                rospy.sleep(0.1)
            f.write(json.dumps(self.motion, indent=4, separators=(",", ": ")))
            rospy.loginfo(f'Finish saving motion to {self.json_filepath}')

    def play(self):
        # Load motion
        if os.path.exists(self.json_filepath):
            rospy.loginfo(f'Load motion data file {self.json_filepath}.')
            with open(self.json_filepath) as f:
                self.motion = json.load(f)
        # Play motion
        rospy.loginfo('Play motion')
        self.ri.servo_on()
        # To prevent sudden movement, take time to reach the initial motion
        if 'joint_states' not in self.motion[0]:
            print("First element must have 'joint_states' key.")
            return
        first_av = list(self.motion[0]['joint_states'].values())
        self.ri.angle_vector(first_av, 3)
        self.ri.wait_interpolation()
        # Play the actions from the second one onward
        prev_time = self.motion[0]['time']
        avs = []
        tms = []
        for motion in self.motion[1:]:
            current_time = motion['time']
            # Send angle vector
            if 'joint_states' in motion:
                av = np.array(list(motion['joint_states'].values()))
                avs.append(av)
                tms.append(current_time - prev_time)
            prev_time = current_time
        rospy.loginfo('angle vectors')
        rospy.loginfo(f'{avs}')
        rospy.loginfo('times')
        rospy.loginfo(f'{tms}')
        self.ri.angle_vector_sequence(avs, tms)
        # Check interruption by button
        # use self.ri.is_interpolating() after the following PR is merged
        # https://github.com/iory/scikit-robot/pull/396
        def is_interpolating(controller_type=None):
            if controller_type:
                controller_actions = self.ri.controller_table[controller_type]
            else:
                controller_actions = self.ri.controller_table[self.ri.controller_type]
            is_interpolatings = map(
                lambda action: action.is_interpolating(), controller_actions)
            return any(list(is_interpolatings))
        while not rospy.is_shutdown() and is_interpolating():
            if self.state == State.WAIT:
                self.ri.cancel_angle_vector()
                rospy.loginfo('Play interrupted')
                break
            rospy.sleep(0.5)  # Save motion every 0.5s to smooth motion play
        rospy.loginfo('Play finished')

    def main_loop(self):
        rospy.loginfo('start teaching mode')
        try:
            while not rospy.is_shutdown():
                if self.mode != "TeachingMode":
                    rospy.sleep(1)
                    continue
                if self.state == State.WAIT:
                    sent_str = 'Teaching mode\n\n'\
                        + 'single click:\n  record\n\n'\
                        + 'double click:\n  play\n\n'\
                        + 'triple click:\n  servo_off'
                    self.send_string(sent_str)
                    rospy.sleep(1)
                elif self.state == State.RECORD:
                    sent_str = 'Record mode\n\n'\
                        + 'single click:\n  start/stop recording'
                    self.send_string(sent_str)
                    self.record()
                    self.state = State.WAIT
                elif self.state == State.PLAY:
                    sent_str = 'Play mode\n\n'\
                        + 'double click:\n  start/stop playing'
                    self.send_string(sent_str)
                    self.play()
                    self.state = State.WAIT
        except KeyboardInterrupt:
            print('Finish teaching mode by KeyboardInterrupt')
            exit()


if __name__ == '__main__':
    rospy.init_node('teaching_mode', anonymous=True)
    tm = TeachingMode(0x42)
    # To stop program by Ctrl-c
    import signal
    signal.signal(signal.SIGINT, signal.default_int_handler)
    tm.main_loop()
