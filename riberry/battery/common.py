from collections import Counter


def majority_vote(history):
    if not history:
        return 0
    count = Counter(history)
    return count.most_common(1)[0][0]
