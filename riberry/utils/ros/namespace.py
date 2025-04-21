import rospy


def get_base_namespace():
    """Return the clean namespace for the node."""
    full_namespace = rospy.get_namespace()
    last_slash_pos = full_namespace.rfind("/")
    return full_namespace[:last_slash_pos] if last_slash_pos != 0 else ""
