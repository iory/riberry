import re


class SelectList:
    def __init__(self, items=None):
        if isinstance(items, list):
            self.options = items
        else:
            self.options = []
        self.idx = 0
        self.pattern = None

    def add_option(self, item, position="start"):
        if position == "start":
            self.options.insert(0, item)
        elif position == "end":
            self.options.append(item)

    def remove_option(self, item):
        self.options.remove(item)
        if self.idx >= len(self.options):
            self.idx = 0

    def remove_all_options(self):
        self.options = []
        if self.idx >= len(self.options):
            self.idx = 0

    def increment_index(self):
        if len(self.options) == 0:
            return -1
        self.idx = (self.idx + 1) % len(self.options)
        return self.idx

    def reset_index(self):
        self.idx = 0
        return self.idx

    def get_index(self):
        return self.idx

    def set_index(self, idx):
        if idx >= len(self.options):
            self.idx = 0
        else:
            self.idx = idx
        return self.idx

    def set_index_by_keyword(self, key):
        for i, option in enumerate(self.options):
            if self.pattern:
                match = self.pattern.search(option)
                option = match.group(1) if match else option
            if option == key:
                self.idx = i
                return True
        return False

    def selected_option(self, extract=False):
        if len(self.options) <= self.idx:
            return None
        selected = self.options[self.idx]
        if extract:
            return self.pattern.search(selected).group(1)
        else:
            return selected

    def string_options(self, max_num):
        sent_str = ''
        if not self.options:
            return sent_str

        if self.idx < max_num:
            display_options = self.options[:max_num]
            highlight_idx = self.idx
        else:
            display_options = self.options[self.idx-max_num+1:self.idx+1]
            highlight_idx = max_num - 1

        for i, item in enumerate(display_options):
            if self.pattern:
                match = self.pattern.search(item)
                item = match.group(1) if match else item
            color_code = "\x1b[32m" if i == highlight_idx else "\x1b[39m"
            sent_str += f"{color_code}{item}\n"
        sent_str += '\x1b[39m'
        return sent_str

    def set_extract_pattern(self, pattern):
        self.pattern = re.compile(pattern)
