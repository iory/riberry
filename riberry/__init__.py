# flake8: noqa

import importlib.metadata

try:
    __version__ = importlib.metadata.version("riberry-robot")
except importlib.metadata.PackageNotFoundError:
    print("riberry-robot is not installed")
