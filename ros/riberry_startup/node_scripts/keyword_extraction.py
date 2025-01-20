#!/usr/bin/env python3

import json
import os

import numpy as np
from openai import AzureOpenAI
import rospy
from sklearn.metrics.pairwise import cosine_similarity
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String

from riberry.filecheck_utils import get_cache_dir


class EmbeddingCache:
    """
    A class to handle embedding calculations and caching.

    Attributes:
        model (str): Name of the embedding model.
        cache_file (str): Path to the cache file.
        cache (dict): In-memory cache for embeddings.
        client (AzureOpenAI): Client for Azure OpenAI API.
    """

    def __init__(self, model="text-embedding-3-large"):
        self.cache_file = os.path.join(get_cache_dir(), 'embedding_cache.json')
        self.model = model
        self.cache = self._load_cache()
        rospy.loginfo(f"cached_words in {self.cache_file}:")
        for key in list(self.cache.keys()):
            rospy.loginfo(f"- {key}")
        rospy.loginfo("")
        self.client = AzureOpenAI(
            api_key='YOUR_API_KEY',
            azure_endpoint="END_POINT",
            api_version="2024-10-01-preview",)

    def _load_cache(self):
        """Load the embedding cache from a file."""
        if os.path.exists(self.cache_file):
            with open(self.cache_file, encoding="utf-8") as f:
                return json.load(f)
        return {}

    def _save_cache(self):
        """Save the current cache to a file."""
        with open(self.cache_file, "w", encoding="utf-8") as f:
            json.dump(self.cache, f, ensure_ascii=False, indent=4)

    def get_embeddings(self, inputs):
        """
        Retrieve embeddings for the given inputs, calculating new ones if needed.
        Maximum array size is 2048.

        Args:
            inputs (list of str): Input text for which embeddings are required.

        Returns:
            list of numpy.ndarray: List of embeddings corresponding to the inputs.
        """
        new_inputs = [inp for inp in inputs if inp not in self.cache]

        if new_inputs:
            response = self.client.embeddings.create(input=new_inputs, model=self.model)
            for input_text, embedding_data in zip(new_inputs, response.data):
                self.cache[input_text] = embedding_data.embedding
            rospy.loginfo(f"Calculated new embeddings for: {new_inputs}")
            self._save_cache()

        # Retrieve embeddings from the cache
        return [np.array(self.cache[inp]).reshape(1, -1) for inp in inputs]


class SpeechToKeyword:
    """
    A class to process speech-to-text input and extract the most relevant keyword based on similarity.

    Attributes:
        embedding_cache (EmbeddingCache): Instance for handling embeddings.
        keywords (list of str): Predefined keywords for comparison.
        keyword_pub (Publisher): Publisher to send the most similar keyword.
    """

    def __init__(self, embedding_cache):
        self.embedding_cache = embedding_cache
        self.keywords = rospy.get_param("~keywords", [])
        if len(self.keywords) == 0:
            rospy.logerr('~keywords param is not set')
        rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.speech_to_keyword_callback)
        self.keyword_pub = rospy.Publisher('~output', String, queue_size=10)

    def speech_to_keyword_callback(self, msg):
        if not msg.transcript:
            rospy.logwarn("Received empty transcript.")
            return
        # Use the first transcript with the highest confidence
        user_input = msg.transcript[0]
        similar_keyword, similarity = self.calc_similarity(user_input)
        self.keyword_pub.publish(String(data=similar_keyword))

    def calc_similarity(self, prompt):
        """
        Calculate the similarity between the input prompt and predefined keywords.

        Args:
            prompt (str): User input text.

        Returns:
            tuple: The most similar keyword and its similarity score.
        """
        embeddings = self.embedding_cache.get_embeddings([prompt]+self.keywords)
        prompt_embedding = embeddings[0]
        keyword_embeddings = embeddings[1:]
        similarities = []
        for keyword, keyword_embedding in zip(self.keywords, keyword_embeddings):
            similarity = cosine_similarity(prompt_embedding, keyword_embedding)[0][0]
            similarities.append((keyword, similarity))

        # Sort by similarity and get the highest-scoring keyword
        similarities.sort(key=lambda x: x[1], reverse=True)
        most_similar = similarities[0]
        rospy.loginfo(f"Prompt: {prompt}")
        rospy.loginfo(
            f"Most similar keyword: {most_similar[0]} (Similarity: {most_similar[1]:.4f})")
        rospy.loginfo("Similarity with all keywords:")
        for keyword, similarity in similarities:
            rospy.loginfo(f"- {keyword}: {similarity:.4f}")
        rospy.loginfo("")

        return most_similar[0], most_similar[1]


if __name__ == '__main__':
    rospy.init_node('keyword_extraction')
    embedding_cache = EmbeddingCache()
    speech_to_keyword = SpeechToKeyword(embedding_cache)
    rospy.spin()
