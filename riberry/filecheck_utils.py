import os


def get_cache_dir():
    """Return cache dir.
    Returns
    -------
    cache_dir : str
        cache directory.
    """
    ros_home = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))
    pkg_ros_home = os.path.join(ros_home, 'riberry')
    default_cache_dir = os.path.join(pkg_ros_home, 'cache')
    cache_dir = os.environ.get(
        'RIBERRY_CACHE_DIR',
        default_cache_dir)
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    return cache_dir
