import re


class SelectList:
    def __init__(self):
        self.options = []
        self.idx = 0
        self.pattern = None

    def add_option(self, item):
            self.options.insert(0, item)

    def increment_index(self):
        self.idx += 1
        if self.idx >= len(self.options):
            self.idx = 0

    def get_selected(self, extract=False):
        if len(self.options) <= self.idx:
            return None
        selected = self.options[self.idx]
        if extract:
            return self.pattern.search(selected).group(1)
        else:
            return selected

    def get_list_string(self, num):
        sent_str = ''
        if self.idx < num:
            for i, item in enumerate(self.options[:num]):
                item = self.pattern.search(item).group(1)
                if i == self.idx:
                    sent_str += '\x1b[32m' + item
                else:
                    sent_str += '\x1b[37m' + item
                sent_str += '\x1b[0m\n'
        else:
            for i, item in enumerate(self.options[self.idx-num+1:self.idx+1]):
                item = self.pattern.search(item).group(1)
                if i == num-1:
                    sent_str += '\x1b[32m' + item + '\x1b[0m'
                else:
                    sent_str += item
                sent_str += '\n'
        return sent_str

    def add_extract_pattern(self, pattern):
        self.pattern = re.compile(pattern)
