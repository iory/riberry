import json
import os
from pathlib import Path

from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np
import rospy
from skrobot.model import RobotModel


class MotionManager:
    def __init__(self):
        # skrobot version check
        from packaging import version
        import skrobot
        required_version = "0.0.44"
        current_version = skrobot.__version__
        if version.parse(current_version) < version.parse(required_version):
            raise Exception(f"skrobot version is not greater than {required_version}. (current version: {current_version})\npip install scikit-robot -U")
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
        self.stop = False

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

    def stop(self):
        self.stop = True

    def record(self):
        self.motion = []
        with open(self.json_filepath, mode='w') as f:
            rospy.loginfo(f'Start saving motion to {self.json_filepath}')
            while not rospy.is_shutdown():
                # Finish recording
                if self.stop is True:
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
            is_interpolatings = (action.is_interpolating() for action in controller_actions)
            return any(list(is_interpolatings))
        while not rospy.is_shutdown() and is_interpolating():
            if self.stop is True:
                self.ri.cancel_angle_vector()
                rospy.loginfo('Play interrupted')
                break
            rospy.sleep(0.5)  # Save motion every 0.5s to smooth motion play
        rospy.loginfo('Play finished')
