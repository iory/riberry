import json
import os

import numpy as np
from openai import AzureOpenAI
from sklearn.metrics.pairwise import cosine_similarity

from riberry.filecheck_utils import get_cache_dir


class EmbeddingCache:
    """
    A class that manages text embeddings with local caching capabilities using Azure OpenAI's embedding service.

    This class handles the generation, storage, and retrieval of text embeddings. It maintains a local cache
    to avoid redundant API calls for previously processed text, improving efficiency and reducing API usage.

    Attributes:
        model (str): The name of the Azure OpenAI embedding model to use.
        cache_file (str): File path where embeddings are cached on disk.
        cache (dict): In-memory dictionary storing text-to-embedding mappings.
        client (AzureOpenAI): Azure OpenAI API client instance.
        debug (bool): When True, enables detailed logging of operations.
    """
    def __init__(self, model="text-embedding-3-large", debug=False):
        self.cache_file = os.path.join(get_cache_dir(), 'embedding_cache.json')
        self.model = model
        self.cache = self._load_cache()
        self.debug = debug
        if self.debug is True:
            print(f"cached_words in {self.cache_file}:")
            for key in list(self.cache.keys()):
                print(f"- {key}")
            print("")
        self._load_credentials("/etc/opt/riberry/credentials.json")
        try:
            api_key = os.environ['AZURE_API_KEY']
            endpoint = os.environ['AZURE_ENDPOINT']
        except KeyError as e:
            print(f"[WARNING] Environment variable '{e}' is not set.")
            print("Azure API cannot be used.\n")
            api_key = ""
            endpoint = ""
        self.client = AzureOpenAI(
            api_key=api_key,
            azure_endpoint=endpoint,
            api_version="2024-10-01-preview",)

    def _load_credentials(self, filepath):
        """Load credentials and export them as environment variable"""
        if not os.path.exists(filepath):
            return
        print(f"Load credentials from {filepath}")
        try:
            with open(filepath) as f:
                config = json.load(f)
            api_key = config.get('AZURE_API_KEY')
            endpoint = config.get('AZURE_ENDPOINT')
            if api_key:
                os.environ['AZURE_API_KEY'] = api_key
            if endpoint:
                os.environ['AZURE_ENDPOINT'] = endpoint
        except Exception as e:
            print(f"Error reading config file: {e}")

    def _load_cache(self):
        """
        Load previously cached embeddings from disk.

        Returns:
            dict: Cached embeddings mapping text to their vector representations.
                 Returns empty dict if no cache file exists.
        """
        if os.path.exists(self.cache_file):
            with open(self.cache_file, encoding="utf-8") as f:
                return json.load(f)
        return {}

    def _save_cache(self):
        """
        Save the current embedding cache to disk in JSON format.

        The cache is saved with UTF-8 encoding and proper indentation for readability.
        """
        temp_filename = self.cache_file + '.tmp'
        with open(temp_filename, "w", encoding="utf-8") as f:
            json.dump(self.cache, f, ensure_ascii=False, indent=4)
        # If the save is succeeded, rename the temporary file to the original filename.
        os.replace(temp_filename, self.cache_file)

    def get_embeddings(self, inputs):
        """
        Get embeddings for a list of input texts, using cache when available.

        This method checks the cache first for existing embeddings. For any inputs
        not found in the cache, it generates new embeddings via the Azure OpenAI API
        and caches them for future use.

        Args:
            inputs (list of str): List of text inputs to get embeddings for.

        Returns:
            list of numpy.ndarray: List of embedding vectors, each shaped as (1, embedding_dim).
                                 The embedding dimension is determined by the model.

        Note:
            The Azure OpenAI API has a maximum input size limit of 2048 tokens.
        """
        # If the input is not in cache, the embedding is calculated and cached.
        new_inputs = [inp for inp in inputs if inp not in self.cache]
        if new_inputs:
            response = self.client.embeddings.create(input=new_inputs, model=self.model)
            for input_text, embedding_data in zip(new_inputs, response.data):
                self.cache[input_text] = embedding_data.embedding
            print(f"Calculated new embeddings for: {new_inputs}")
            self._save_cache()

        # Retrieve embeddings from the cache
        return [np.array(self.cache[inp]).reshape(1, -1) for inp in inputs]

    def calc_candidates(self, prompt, contexts, debug=False):
        """
        Calculate semantic similarity between an input prompt and a set of predefined contexts.

        This method computes cosine similarity between the prompt's embedding and the embeddings
        of all sentences in the provided contexts. It organizes the results by keywords and
        their associated sentences.

        Args:
            prompt (str): The input text to compare against contexts.
            contexts (dict): Dictionary mapping keywords to lists of related sentences.
            debug (bool): When True, prints detailed similarity scores. Defaults to False.

        Returns:
            dict: Nested dictionary structure:
                {
                    'keyword1': {
                        'related_sentence1': similarity_score1,
                        'related_sentence2': similarity_score2,
                        ...
                    },
                    'keyword2': {
                        ...
                    }
                }
                where similarity scores are float values between -1 and 1.
        """
        # Get embeddings for prompt and all context sentences at once
        sentences = []
        for keyword in contexts.keys():
            sentences += contexts[keyword]
        embeddings = self.get_embeddings([prompt]+sentences)
        prompt_embedding = embeddings[0]

        # Calculate similarities and organize results by keyword
        candidates = {}
        index = 1
        if self.debug is True:
            print("All similarities:")
        for keyword in contexts.keys():
            if self.debug is True:
                print(f"  Keyword: {keyword}")
            sentence_embeddings = embeddings[index:index+len(contexts[keyword])]
            index += len(contexts[keyword])
            similarity_dict = {}
            for sentence, sentence_embedding in zip(contexts[keyword], sentence_embeddings):
                similarity = cosine_similarity(prompt_embedding, sentence_embedding)[0][0]
                similarity_dict[sentence] = similarity
                if self.debug is True:
                    print(f"    {sentence}: {similarity}")
            candidates[keyword] = similarity_dict
        return candidates
