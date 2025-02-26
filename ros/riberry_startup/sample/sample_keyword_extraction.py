#!/usr/bin/env python3

import json
import os

from riberry.embedding_cache import EmbeddingCache


def load_context_json():
    # Load context
    __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__)))
    json_file_path = os.path.join(__location__, "contexts.json")
    with open(json_file_path) as f:
        contexts = json.load(f)
        if any(contexts) is False:
            print('[Error] Context is not registered')
            return None
    return contexts

def calculate_similarity(embedding_cache, prompt, context):
    candidates = embedding_cache.calc_candidates(prompt, contexts)
    keywords = []
    similarities = []
    for keyword in candidates.keys():
        similarity_dict = candidates[keyword]
        # Use most similar sentence score
        best_sentence, max_similarity = max(
            similarity_dict.items(), key=lambda x: x[1])
        keywords += [keyword]
        similarities.append(max_similarity)
    return keywords, similarities

def extract_keyword(keywords, similarities):
    max_index = similarities.index(max(similarities))
    highest_similarity_keyword = keywords[max_index]
    highest_similarity_value = similarities[max_index]
    print(f"Extracted keyword: {highest_similarity_keyword}")
    print(f"similarity: {highest_similarity_value}")

if __name__ == '__main__':
    embedding_cache = EmbeddingCache(debug=True)
    prompt = input('Enter prompt: ')
    contexts = load_context_json()
    keywords, similarities = calculate_similarity(
        embedding_cache, prompt, contexts)
    extract_keyword(keywords, similarities)
