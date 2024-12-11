import json
import os

from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode


class MotionManager:
    def __init__(self):
        # skrobot version check
        from packaging import version
        import skrobot
        required_version = "0.0.45"
        current_version = skrobot.__version__
        if version.parse(current_version) < version.parse(required_version):
            raise Exception(f"skrobot version is not greater than {required_version}. (current version: {current_version})\npip install scikit-robot -U")
        # Create robot model to control pressure
        robot_model = RobotModel()
        namespace = ""
        with no_mesh_load_mode():
            robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description_viz")
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )
        self.joint_names = self.ri.robot.joint_names
        self.motion = []
        self._stop = False

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
        self._stop = True

    def servo_off(self):
        self.ri.servo_off()

    def record(self, record_filepath):
        self._stop = False
        self.motion = []
        with open(record_filepath, mode='w') as f:
            rospy.loginfo(f'Start saving motion to {record_filepath}')
            while not rospy.is_shutdown():
                # Finish recording
                if self._stop is True:
                    break
                self.add_motion()
                rospy.sleep(0.1)
            f.write(json.dumps(self.motion, indent=4, separators=(",", ": ")))
            rospy.loginfo(f'Finish saving motion to {record_filepath}')

    def play(self, play_filepath):
        self._stop = False
        # Load motion
        if os.path.exists(play_filepath):
            rospy.loginfo(f'Load motion data file {play_filepath}.')
            with open(play_filepath) as f:
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
        while not rospy.is_shutdown() and self.ri.is_interpolating():
            if self._stop is True:
                self.ri.cancel_angle_vector()
                rospy.loginfo('Play interrupted')
                break
            rospy.sleep(0.5)  # Save motion every 0.5s to smooth motion play
        rospy.loginfo('Play finished')
