# flake8: noqa

import pkg_resources

try:
    version = pkg_resources.get_distribution('ri-berry').version
except pkg_resources.DistributionNotFound:
    print("ri-berry is not installed")
