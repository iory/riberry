# flake8: noqa

import pkg_resources

try:
    __version__ = pkg_resources.get_distribution("riberry-robot").version
except pkg_resources.DistributionNotFound:
    print("riberry-robot is not installed")
