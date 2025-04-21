#!/usr/bin/env python3

from datetime import datetime
import json
import os
import threading

from colorama import Fore
from kxr_controller.kxr_interface import KXRROSRobotInterface
import numpy as np
import rosbag
import rospy
import rostopic
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from std_msgs.msg import Int32

from riberry.com.base import PacketType
from riberry.filecheck_utils import get_cache_dir
from riberry.mode import Mode
from riberry.utils.ros.namespace import get_base_namespace


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


class DataCollectionMode(Mode):
    def __init__(self):
        super().__init__()
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
        robot_model = RobotModel()
        namespace = get_base_namespace()
        with no_mesh_load_mode():
            robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description_viz")
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )
        if self.ri is None:
            self.additional_msg = "Failed to Create robot model"
        else:
            self.additional_msg = "Ready for data collection"

    def latest_filename(self):
        current_dir = self.current_dataset_dir()
        if current_dir is None:
            return None
        all_entries = os.listdir(current_dir)
        files = [os.path.join(current_dir, entry) for entry in all_entries
                 if os.path.isfile(os.path.join(current_dir, entry)) and entry.endswith("bag")]
        if not files:
            return None  # ファイルがない場合
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

    def button_cb(self, msg):
        if self.mode != "DataCollectionMode":
            return
        if self.ri is None:
            rospy.logwarn("KXRROSRobotInterface instance is not created.")
            return
        if msg.data == 1:
            self.additional_msg = ""
            current_time = datetime.now().strftime("%m%d_%H%M%S")
            dataset_dir = self.current_dataset_dir()
            if dataset_dir is None:
                message = "Create dataset first"
                rospy.logerr("Create dataset first")
                self.additional_message = message
                return
            # Start recording
            if not self.recorder.is_recording():
                try:
                    initial_angle_vector = self.initial_angle_vector()
                    if initial_angle_vector is not None:
                        # Move to initial pose
                        self.ri.servo_on()
                        self.ri.angle_vector(initial_angle_vector, 3)
                        self.ri.wait_interpolation()
                        # Users can record task after servo off
                        self.ri.servo_off()
                        bag_filename = os.path.join(
                            dataset_dir,
                            f'{current_time}.bag')
                        self.recorder.start_recording(self.topics, bag_filename)
                        self.additional_msg = "Recording rosbag ..."
                except AssertionError as e:
                    error_msg = f"Recording aborted: {e}"
                    rospy.logerr(error_msg)
                    self.additional_msg = error_msg
                    self.recorder.stop_recording()
                    os.remove(bag_filename)
            # Stop recording
            else:
                self.recorder.stop_recording()
                message = "Successfully finish recording"
                rospy.loginfo(message)
                self.additional_msg = message
                self.recorder.show_bag(self.latest_filename())
        # Delete the latest data
        elif msg.data == 2:
            latest_bag_filename = self.latest_filename()
            if latest_bag_filename is not None:
                os.remove(latest_bag_filename)
                message = "Remove the latest data"
                rospy.loginfo(message)
                rospy.loginfo(latest_bag_filename)
                self.additional_msg = message
        # Create new dataset
        elif msg.data == 3:
            message = "Create dataset"
            rospy.loginfo(message)
            self.additional_msg = message
            self.create_dataset_dir()
            self.initial_angle_vector(self.ri.angle_vector())

    def timer_callback(self, event):
        if self.mode != "DataCollectionMode":
            return
        self.send_string()

    def send_string(self):
        sent_str = chr(PacketType.DATA_COLLECTION_MODE)
        if self.recorder.is_recording():
            sent_str += "1 " + Fore.GREEN + "start" + Fore.RESET + "/stop"
            sent_str += Fore.RED + "●" + Fore.RESET + "\n"
        else:
            sent_str += "1 start/" + Fore.GREEN + "stop\n" + Fore.RESET
        sent_str += "2 Remove last\n"
        sent_str += "3 New Dataset\n\n"
        dataset_dir = self.current_dataset_dir()
        if dataset_dir is not None:
            dataset_id = os.path.basename(dataset_dir)[len("dataset_"):]  # Remove prefix
            dataset_day, dataset_time = dataset_id.split("_")
            sent_str += f"ID: {dataset_day}_\n{dataset_time}\n"
        sent_str += f"Size: {self.current_dataset_size()}\n"
        sent_str += "\n" + self.additional_msg

        # Send message on AtomS3 LCD
        self.write(sent_str)


if __name__ == "__main__":
    rospy.init_node("data_collection_mode")
    rospy.loginfo("Start Data Collection Mode")
    dcm = DataCollectionMode()
    rospy.spin()
