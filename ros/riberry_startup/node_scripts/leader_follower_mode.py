#!/usr/bin/env python3

from datetime import datetime
from enum import Enum
import json
import os
import threading

from colorama import Fore
from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np
import rosbag
import rospy
import rostopic
from sensor_msgs.msg import JointState
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from std_msgs.msg import Int32

from riberry.com.base import PacketType
from riberry.filecheck_utils import get_cache_dir
from riberry.mode import Mode


class RosbagRecorder:
    def __init__(self):
        self.bag = None
        self.subscribers = []
        self.running = False
        self.lock = threading.Lock()
        self.publishers = {}

    def _callback(self, topic_name):
        def inner(msg):
            with self.lock:
                if self.bag and self.running:
                    self.bag.write(topic_name, msg, rospy.Time.now())
        return inner

    def _get_message_class(self, topic):
        try:
            msg_type, _, _ = rostopic.get_topic_class(topic)
            return msg_type
        except Exception:
            return None

    def is_recording(self):
        return self.running

    def start_recording(self, topics, bag_filename, timeout=1.0):
        self.bag = rosbag.Bag(bag_filename, 'w')
        for topic in topics:
            try:
                msg_class = self._get_message_class(topic)
                if msg_class is None:
                    raise AssertionError(
                        f"Unknown message type {topic}")
                rospy.wait_for_message(topic, msg_class, timeout=timeout)
                rospy.loginfo(f"Subscribing to {topic}")
                sub = rospy.Subscriber(topic, msg_class, self._callback(topic))
                self.subscribers.append(sub)
            except rospy.ROSException:
                raise AssertionError(f"Timeout '{topic}'")
        self.running = True

    def stop_recording(self):
        self.running = False
        for sub in self.subscribers:
            sub.unregister()
        self.subscribers = []
        with self.lock:
            if self.bag:
                self.bag.close()
                self.bag = None

    def show_bag(self, bag_filename):
        if not os.path.exists(bag_filename):
            rospy.logwarn(f"Bag file {bag_filename} not found.")
            return
        rospy.loginfo(f"Saved rosbag: {bag_filename}")
        with rosbag.Bag(bag_filename, 'r') as bag:
            info = {}
            start_time = None
            end_time = None

            for topic, msg, t in bag.read_messages():
                if topic not in info:
                    info[topic] = {
                        "type": type(msg).__name__,
                        "count": 0
                    }
                info[topic]["count"] += 1
                if start_time is None or t < start_time:
                    start_time = t
                if end_time is None or t > end_time:
                    end_time = t

            rospy.loginfo("Recorded topics:")
            for topic, meta in info.items():
                rospy.loginfo(f"  {topic}: {meta['count']} msgs (type: {meta['type']})")

            if start_time and end_time:
                duration = (end_time - start_time).to_sec()
                rospy.loginfo(f"Duration: {duration:.2f} seconds")


class TopicRelay:
    """
    Relay messages from leader to follower topic with specified message class.
    """
    def __init__(self, msg_class, leader_topic, follower_topic, queue_size=1):
        self.last_seq = None
        self.pub = rospy.Publisher(follower_topic, msg_class, queue_size=queue_size)
        self.sub = rospy.Subscriber(leader_topic, msg_class, self.callback, queue_size=queue_size)
        rospy.loginfo(f"Relaying {msg_class.__name__} from '{leader_topic}' to '{follower_topic}' (queue_size={queue_size})")

    def callback(self, msg):
        # Directly republish received message
        if self.last_seq is None:
            self.last_seq = msg.header.seq
        if self.last_seq >= msg.header.seq:
            return
        self.pub.publish(msg)

    def shutdown(self):
        # Clean up subscriber and publisher
        self.sub.unregister()
        self.pub.unregister()
        rospy.loginfo("TopicRelay shutdown complete.")


class State(Enum):
    WAIT = 0
    LEADER_FOLLOWER = 1
    SUSPEND = 2


class LeaderFollowerMode(Mode):
    def __init__(self):
        super().__init__()
        self.state = State.WAIT
        # rosbag recorder
        self.topics = rospy.get_param("~topic_names")
        rospy.loginfo(
            "Topics to be recorded:\n" + "\n".join(
                f"  - {topic}" for topic in self.topics))
        self.cache_dir = get_cache_dir()
        self.recorder = RosbagRecorder()
        # Callbacks
        self.ri = None
        rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        rospy.Subscriber(
            "atom_s3_button_state", Int32,
            callback=self.button_cb, queue_size=1
        )
        # Create robot model
        self.additional_msg = "Creating robot model..."
        self.ri = {}
        for role in ["leader", "follower"]:
            robot_model = RobotModel()
            namespace = role
            with no_mesh_load_mode():
                robot_model.load_urdf_from_robot_description(
                    "/" + namespace + "/robot_description_viz")
            self.ri[role] = KXRROSRobotInterface(
                robot_model, namespace=namespace, controller_timeout=60.0
            )
            if self.ri[role] is None:
                self.additional_msg = f"Failed to Create {role} robot model"
            else:
                self.additional_msg = "Ready for leader/follower control"
        self.original_cfg = self.ri["follower"].update_kxr_parameters()
        self.set_normal_cfg()

    def set_normal_cfg(self):
        self.ri["follower"].update_kxr_parameters(**self.original_cfg)
        self.ri["follower"].send_stretch(127)
        self.ri["follower"].switch_fullbody_controller(start=True)
        self.ri["follower"].select_command_joint_state(
            'command_joint_state_from_robot_hardware')

    def set_leader_follower_cfg(self):
        self.ri["follower"].update_kxr_parameters(frame_count=10)
        self.ri["follower"].switch_fullbody_controller(start=False)
        self.ri["follower"].select_command_joint_state(
            'command_joint_state_from_joint_state')

    def latest_filename(self):
        current_dir = self.current_dataset_dir()
        if current_dir is None:
            return None
        all_entries = os.listdir(current_dir)
        files = [os.path.join(current_dir, entry) for entry in all_entries
                 if os.path.isfile(os.path.join(current_dir, entry)) and entry.endswith("bag")]
        if not files:
            return None
        files_sorted = sorted(files)
        last_file = files_sorted[-1]
        return last_file

    def initial_angle_vector(self, angle_vector=None):
        """Initial angle vector of the task"""
        dir_path = self.current_dataset_dir()
        if dir_path is None:
            return None
        json_path = os.path.join(dir_path, "initial_state.json")
        # Read initial angle vector
        if angle_vector is None:
            with open(json_path) as f:
                angle_vector = json.load(f)["angle_vector"]
                return np.array(angle_vector, dtype=np.float32)
        # Write initial angle vector
        else:
            json_data = {"angle_vector": angle_vector.tolist()}
            with open(json_path, 'w') as f:
                json.dump(json_data, f, indent=4)
                return True

    def current_dataset_dir(self):
        """Return the latest dataset dir"""
        if not os.path.isdir(self.cache_dir):
            return None
        dirs = [
            os.path.join(self.cache_dir, d)
            for d in os.listdir(self.cache_dir)
            if os.path.isdir(os.path.join(self.cache_dir, d)) and d.startswith("dataset_")
        ]
        dirs.sort(key=os.path.getmtime, reverse=True)
        return dirs[0] if dirs else None

    def current_dataset_size(self):
        """Return the size of the latest dataset dir"""
        dir_path = self.current_dataset_dir()
        if dir_path is None:
            return None
        if not os.path.isdir(dir_path):
            return 0
        file_count = 0
        for item in os.listdir(dir_path):
            item_path = os.path.join(dir_path, item)
            if os.path.isfile(item_path) and item_path.endswith("bag"):
                file_count += 1
        return file_count

    def create_dataset_dir(self):
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        dataset_name = f"dataset_{current_time}"
        dataset_dir = os.path.join(self.cache_dir, dataset_name)
        os.mkdir(dataset_dir)
        return dataset_dir

    def leader_follower(self):
        relay = TopicRelay(
            msg_class=JointState,
            leader_topic=f'{self.ri["leader"].namespace}/current_joint_states',
            follower_topic=f'{self.ri["follower"].namespace}/command_joint_state',
            queue_size=1
        )
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and not self.state == State.WAIT:
            rate.sleep()
        relay.shutdown()

    def button_cb(self, msg):
        if self.mode != "LeaderFollowerMode":
            return
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        if msg.data != 0:
            self.additional_msg = ""
        if self.state == State.WAIT:
            # Move to leader/follower control
            if msg.data == 1:
                self.state = State.LEADER_FOLLOWER
                # Move to initial pose
                self.additional_msg = "Follower is moving to initial pose..."
                self.set_normal_cfg()
                self.ri["follower"].servo_on()
                self.ri["follower"].angle_vector(
                    self.ri["leader"].angle_vector(), 3)
                self.ri["follower"].wait_interpolation()
                # Start leader/follower
                self.set_leader_follower_cfg()
                t = threading.Thread(target=self.leader_follower, daemon=True)
                t.start()
                self.additional_msg = "Start leader/follower control"
            # Load initial angle vector
            elif msg.data == 2:
                initial_angle_vector = self.initial_angle_vector()
                if initial_angle_vector is not None:
                    self.additional_msg = "Loading initial pose..."
                    self.ri["leader"].servo_on()
                    self.ri["leader"].angle_vector(initial_angle_vector, 3)
                    self.ri["leader"].wait_interpolation()
                    self.ri["leader"].servo_off()
                    self.additional_msg = "Load initial pose"
                else:
                    self.additional_msg = "Failed to load initial pose"
            # Create new dataset
            elif msg.data == 3:
                self.additional_msg = "Create dataset and record initial pose"
                self.create_dataset_dir()
                self.initial_angle_vector(self.ri["leader"].angle_vector())
        # Leader/Follower control
        elif self.state == State.LEADER_FOLLOWER:
            # Start/Stop recording
            if msg.data == 1:
                current_time = datetime.now().strftime("%m%d_%H%M%S")
                dataset_dir = self.current_dataset_dir()
                if dataset_dir is None:
                    self.additional_msg = "Create dataset first"
                else:
                    # Start recording
                    if not self.recorder.is_recording():
                        try:
                            bag_filename = os.path.join(
                                dataset_dir,
                                f'{current_time}.bag')
                            self.recorder.start_recording(self.topics, bag_filename)
                            self.additional_msg = "Recording rosbag ..."
                        except AssertionError as e:
                            self.additional_msg = f"Recording aborted: {e}"
                            self.recorder.stop_recording()
                            os.remove(bag_filename)
                    # Stop recording
                    else:
                        self.recorder.stop_recording()
                        self.additional_msg = "Successfully finish recording\n"
                        self.additional_msg += f"Size: {self.current_dataset_size()}\n"
                        self.recorder.show_bag(self.latest_filename())
            # Move to SUSPEND
            elif msg.data == 2:
                self.additional_msg = "Suspending leader/follower control"
                self.ri["leader"].servo_on()
                self.state = State.SUSPEND
            # Move to WAIT
            elif msg.data == 3:
                self.additional_msg = "Finish leader/follower control"
                self.ri["follower"].servo_off()
                self.set_normal_cfg()
                self.state = State.WAIT
        # SUSPEND
        elif self.state == State.SUSPEND:
            # Return to leader/follower
            if msg.data == 1:
                self.additional_msg = "Return from suspend"
                self.ri["leader"].servo_off()
                self.state = State.LEADER_FOLLOWER
            # Delete the latest data
            elif msg.data == 2:
                latest_bag_filename = self.latest_filename()
                if latest_bag_filename is not None:
                    os.remove(latest_bag_filename)
                    self.additional_msg = "Remove the latest data"
                    self.additional_msg += f"Size: {self.current_dataset_size()}\n"
                    rospy.loginfo(latest_bag_filename)
        if self.additional_msg != "":
            rospy.loginfo_throttle(60, self.additional_msg)

    def timer_callback(self, event):
        if self.mode != "LeaderFollowerMode":
            return
        self.send_string()

    def send_string(self):
        sent_str = chr(PacketType.LEADER_FOLLOWER_MODE)
        # Create message based on the state
        if self.state == State.WAIT:
            sent_str += "1 Teleop\n"
            sent_str += "2 Load pose\n"
            sent_str += "3 New Dataset\n\n"
            dataset_dir = self.current_dataset_dir()
            if dataset_dir is not None:
                dataset_id = os.path.basename(dataset_dir)[len("dataset_"):]  # Remove prefix
                dataset_day, dataset_time = dataset_id.split("_")
                sent_str += f"ID: {dataset_day}_\n{dataset_time}\n"
            sent_str += f"Size: {self.current_dataset_size()}\n"
            sent_str += "\n" + self.additional_msg
        elif self.state == State.LEADER_FOLLOWER:
            sent_str += "1 Rec motion"
            if self.recorder.is_recording():
                sent_str += "  " + Fore.GREEN + "Start" + Fore.RESET + "/Stop"
                sent_str += Fore.RED + "●" + Fore.RESET + "\n"
            else:
                sent_str += "  Start/" + Fore.GREEN + "Stop\n" + Fore.RESET
            sent_str += "2 Suspend\n"
            sent_str += "3 Exit\n"
            sent_str += "\n" + self.additional_msg
        elif self.state == State.SUSPEND:
            sent_str += "1 Return\n"
            sent_str += "2 Remove last\n"
            sent_str += "\n" + self.additional_msg
        # Send message on AtomS3 LCD
        self.write(sent_str)


if __name__ == "__main__":
    rospy.init_node("leader_follower_mode")
    lfm = LeaderFollowerMode()
    rospy.spin()
