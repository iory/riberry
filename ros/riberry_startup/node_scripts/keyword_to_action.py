#!/usr/bin/env python3

import os

from kxr_controller.kxr_interface import KXRROSRobotInterface
from riberry_startup.msg import Context
from riberry_startup.msg import KeywordCandidates
from riberry_startup.srv import RegisterContexts
from riberry_startup.srv import RegisterContextsRequest
from riberry_startup.srv import SelectMotion
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from std_srvs.srv import SetBool

from riberry.filecheck_utils import get_cache_dir


class KeywordToAction:
    def __init__(self):
        # Create robot model to control pressure
        robot_model = RobotModel()
        namespace = ""
        with no_mesh_load_mode():
            robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description_viz")
        self.ri = KXRROSRobotInterface(
            robot_model, namespace=namespace, controller_timeout=60.0
        )

        # Keyword extraction
        rospy.Subscriber(
            "keyword_extraction/candidates", KeywordCandidates, self.keyword_cb)
        self.threshold = 0.65
        rospy.wait_for_service('keyword_extraction/register_contexts')
        self.register_contexts= rospy.ServiceProxy(
        'keyword_extraction/register_contexts', RegisterContexts)

        # Control robot
        self.req_play = rospy.ServiceProxy("teaching_mode/play", SelectMotion)
        self.req_record = rospy.ServiceProxy("teaching_mode/record", SetBool)
        self.req_special_action = rospy.ServiceProxy("teaching_mode/special_action", SetBool)
        self.req_change_name = rospy.ServiceProxy("teaching_mode/change_name", SetBool)

        # Send motion name
        self.motion_name_pub = rospy.Publisher(
            'teaching_mode/motion_name', String, queue_size=1)
        rospy.Subscriber('speech_to_text',
                         SpeechRecognitionCandidates, self.speech_callback, queue_size=1)

        self.pub_mode = rospy.Publisher(
            "atom_s3_force_mode", String, queue_size=1)
        self.pub_atoms3_info = rospy.Publisher(
            "atom_s3_additional_info", String, queue_size=1)
        self.pub_teaching_info = rospy.Publisher(
            "teaching_mode_additional_info", String, queue_size=1)
        rospy.Subscriber("atom_s3_mode", String, self.mode_cb)
        self.mode = None
        self.named_motions = []
        self.wait_play_motion = False

    def keyword_cb(self, msg):
        max_index = msg.similarities.index(max(msg.similarities))
        highest_similarity_keyword = msg.keywords[max_index]
        highest_similarity_value = msg.similarities[max_index]

        if highest_similarity_value > self.threshold:
            rospy.loginfo(f"Get keyword: {highest_similarity_keyword}")
            self.trigger_action(highest_similarity_keyword)
        else:
            info = f"Unreliable keyword: {highest_similarity_keyword}"
            self.info_on_atoms3(info)
            rospy.logwarn(info)

    def speech_callback(self, msg):
        if not msg.transcript:
            rospy.logwarn("Received empty transcript.")
            return
        num_subscribers = self.motion_name_pub.get_num_connections()
        if num_subscribers > 0:
            text_no_space = "".join(msg.transcript[0].split())
            motion_msg = String(data=text_no_space)
            self.motion_name_pub.publish(motion_msg)

    def mode_cb(self, msg):
        self.mode = msg.data

    def force_mode(self, mode_name, timeout=3):
        self.pub_mode.publish(String(data=mode_name))
        start_time = rospy.Time.now()
        while True:
            if self.mode == mode_name:
                return True
            if timeout > 0:
                elapsed_time = (rospy.Time.now() - start_time).to_sec()
                if elapsed_time > timeout:
                    rospy.logwarn(f"Mode change to {mode_name} timed out after {timeout} seconds")
                    return False
            rospy.sleep(0.01)

    def info_on_atoms3(self, info):
        if self.mode == "TeachingMode":
            self.force_mode("TeachingMode")
            self.pub_teaching_info.publish(info)
        else:
            self.force_mode("DisplayInformationMode")
            self.pub_atoms3_info.publish(info)

    def load_named_motions(self):
        json_dir = get_cache_dir()
        motions = [
            file[len("teaching_"): -len(".json")]
            for file in os.listdir(json_dir)
            if file.startswith("teaching_") and file.endswith(".json")
        ]
        self.named_motions = [
            name for name in motions if not all(c.isdigit() or c == "_" for c in name)
        ]

    def dict_to_context_request(self, dic):
        req = RegisterContextsRequest(contexts=[])
        for keyword in dic.keys():
            context = Context()
            context.keyword = keyword
            context.context = dic[keyword]
            req.contexts.append(context)
        return req

    # User specific function
    def trigger_action(self, keyword):
        if keyword == "サーボオン":
            self.ri.servo_on()
        elif keyword == "サーボオフ":
            self.ri.servo_off()
        elif keyword == "動作名変更":
            self.force_mode("TeachingMode")
            rospy.wait_for_service("teaching_mode/change_name")
            self.req_change_name(True)
        elif keyword == "動作再生開始":
            self.force_mode("TeachingMode")
            self.load_named_motions()
            self.contexts = rospy.get_param("keyword_extraction/contexts", [{}])[0]
            named_motions_dict = {item: [item] for item in self.named_motions}
            merged_dict = {**self.contexts, **named_motions_dict}
            req = self.dict_to_context_request(merged_dict)
            self.register_contexts(req)
            self.wait_play_motion = True
            info = f"motions: {self.named_motions}"
            self.info_on_atoms3(info)
        elif keyword in self.named_motions and self.wait_play_motion:
            self.wait_play_motion = False
            req = self.dict_to_context_request(self.contexts)
            self.register_contexts(req)
            rospy.wait_for_service("teaching_mode/play")
            self.req_play(data=True, name=keyword)
        elif keyword == "動作再生終了":
            self.force_mode("TeachingMode")
            rospy.wait_for_service("teaching_mode/play")
            self.req_play(False)
        elif keyword == "動作教示開始":
            self.force_mode("TeachingMode")
            rospy.wait_for_service("teaching_mode/record")
            self.req_record(True)
        elif keyword == "動作教示終了":
            self.force_mode("TeachingMode")
            rospy.wait_for_service("teaching_mode/record")
            self.req_record(False)
        elif keyword == "掴んで":
            self.force_mode("TeachingMode")
            rospy.wait_for_service("teaching_mode/special_action")
            self.req_special_action(True)  # Start grasp
        elif keyword == "離して":
            self.force_mode("TeachingMode")
            rospy.wait_for_service("teaching_mode/special_action")
            self.req_special_action(False)  # Stop grasp
        elif keyword == "吸着開始":
            self.force_mode("PressureControlMode")
            if hasattr(self.ri, 'pressure_control_client'):
                timeout = rospy.Duration(3)
                self.ri.pressure_control_client.wait_for_server(timeout)
                self.ri.send_pressure_control(
                    board_idx=39,
                    trigger_pressure=-10,
                    target_pressure=-30,
                    release_duration=0
                )
            else:
                rospy.logerr("self.ri does not have pressure_control_client")
        elif keyword == "吸着解除":
            self.force_mode("PressureControlMode")
            if hasattr(self.ri, 'pressure_control_client'):
                timeout = rospy.Duration(3)
                self.ri.pressure_control_client.wait_for_server(timeout)
                self.ri.send_pressure_control(
                    board_idx=39,
                    trigger_pressure=-10,
                    target_pressure=-30,
                    release_duration=2
                )
            else:
                rospy.logerr("self.ri does not have pressure_control_client")
        elif keyword == "DisplayInformationモード":
            self.force_mode("DisplayInformationMode")
        else:
            info = f"Unknown keyword: {keyword}"
            self.info_on_atoms3(info)
            rospy.logwarn(info)


if __name__ == "__main__":
    rospy.init_node("keyword_to_action")
    ka = KeywordToAction()
    rospy.spin()
