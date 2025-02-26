#!/usr/bin/env python3

from kxr_controller.kxr_interface import KXRROSRobotInterface
from riberry_startup.msg import KeywordCandidates
import rospy
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String
from std_srvs.srv import SetBool


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

        # Control robot
        self.req_play = rospy.ServiceProxy("teaching_mode/play", SetBool)
        self.req_record = rospy.ServiceProxy("teaching_mode/record", SetBool)
        self.req_special_action = rospy.ServiceProxy("teaching_mode/special_action", SetBool)
        self.req_change_name = rospy.ServiceProxy("teaching_mode/change_name", SetBool)

        # Send motion name
        self.motion_name_pub = rospy.Publisher(
            'teaching_mode/motion_name', String, queue_size=10)
        rospy.Subscriber('speech_to_text',
                         SpeechRecognitionCandidates, self.speech_callback)

        self.pub_mode = rospy.Publisher(
            "atom_s3_force_mode", String, queue_size=1)
        self.pub_atoms3_info = rospy.Publisher(
            "atom_s3_additional_info", String, queue_size=1)
        self.pub_teaching_info = rospy.Publisher(
            "teaching_mode_additional_info", String, queue_size=1)
        rospy.Subscriber("atom_s3_mode", String, self.mode_cb)
        self.mode = None

    def keyword_cb(self, msg):
        max_index = msg.similarities.index(max(msg.similarities))
        highest_similarity_keyword = msg.keywords[max_index]
        highest_similarity_value = msg.similarities[max_index]

        if highest_similarity_value > self.threshold:
            self.trigger_action(highest_similarity_keyword)
            rospy.loginfo(f"Get keyword: {highest_similarity_keyword}")
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
            motion_msg = String(data=msg.transcript[0])
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

    def info_on_atoms3(self, info):
        if self.mode == "TeachingMode":
            self.force_mode("TeachingMode")
            self.pub_teaching_info.publish(info)
        else:
            self.force_mode("DisplayInformationMode")
            self.pub_atoms3_info.publish(info)

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
            rospy.wait_for_service("teaching_mode/play")
            self.req_play(True)
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
