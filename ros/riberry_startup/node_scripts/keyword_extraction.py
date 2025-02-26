#!/usr/bin/env python3

import json

from riberry_startup.msg import KeywordCandidates
import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from riberry.embedding_cache import EmbeddingCache


class SpeechToKeyword:
    """
    A ROS node that processes speech-to-text input and extracts the most relevant keywords
    based on semantic similarity.

    This class listens to speech recognition results, compares them against predefined
    contextual phrases, and publishes the most semantically similar keywords along with
    their similarity scores.

    Attributes:
        embedding_cache (EmbeddingCache): Handles text embedding operations for similarity calculations.
        contexts (dict): Dictionary of predefined contextual phrases loaded from a JSON file.
        keyword_pub (rospy.Publisher): Publisher for keyword candidates and their similarity scores.
    """

    def __init__(self, embedding_cache):
        self.embedding_cache = embedding_cache
        json_file_path = rospy.get_param("~contexts_json", None)
        if json_file_path is None:
            rospy.logerr("~contexts_json param must be set.")
            return
        with open(json_file_path) as f:
            self.contexts = json.load(f)
            if any(self.contexts) is False:
                rospy.logerr('Context data is empty or invalid')
                return
        rospy.Subscriber('speech_to_text', SpeechRecognitionCandidates, self.speech_to_keyword_callback)
        self.keyword_pub = rospy.Publisher('~candidates', KeywordCandidates, queue_size=10)

    def speech_to_keyword_callback(self, msg):
        """
        Processes speech recognition results and publishes the most relevant keywords.

        This callback calculates similarity scores between the speech input and predefined
        contextual phrases. For keywords with multiple associated phrases, it uses the
        highest similarity score among them.

        Args:
            msg (SpeechRecognitionCandidates): Message containing speech recognition results.

        Notes:
            - Publishes a KeywordCandidates message containing:
                - List of relevant keywords
                - Corresponding similarity scores
            - Skips processing if the transcript is empty
        """
        if not msg.transcript:
            rospy.logwarn("Received empty transcript.")
            return
        user_input = msg.transcript[0]
        candidates = self.embedding_cache.calc_candidates(user_input, self.contexts)
        candidates_msg = KeywordCandidates()
        keywords = []
        similarities = []
        for keyword in candidates.keys():
            similarity_dict = candidates[keyword]
            # Select the highest similarity score among all phrases for this keyword
            best_sentence, max_similarity = max(
                similarity_dict.items(), key=lambda x: x[1])
            keywords += [keyword]
            similarities.append(max_similarity)
        candidates_msg.keywords = keywords
        candidates_msg.similarities = similarities
        self.keyword_pub.publish(candidates_msg)


if __name__ == '__main__':
    rospy.init_node('keyword_extraction')
    embedding_cache = EmbeddingCache()
    speech_to_keyword = SpeechToKeyword(embedding_cache)
    rospy.spin()
