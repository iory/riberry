#!/usr/bin/env python3

import os

import rospkg
import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


def handle_package_list_request(req):
    rospack = rospkg.RosPack()
    packages = rospack.list()
    www_packages = []

    for package in packages:
        if package == "roswww":
            continue
        pkg_path = rospack.get_path(package)
        www_path = os.path.join(pkg_path, "www")
        index_path = os.path.join(pkg_path, "www", "index.html")
        if os.path.isdir(www_path) and os.path.exists(index_path):
            www_packages.append(package)

    return TriggerResponse(success=True, message=",".join(www_packages))


if __name__ == "__main__":
    rospy.init_node("package_list_server")

    full_namespace = rospy.get_namespace()
    last_slash_pos = full_namespace.rfind("/")
    clean_namespace = full_namespace[:last_slash_pos] if last_slash_pos != 0 else ""
    if clean_namespace:
        clean_namespace = "/" + clean_namespace
    print(f"{clean_namespace}/list_www_packages")
    s = rospy.Service(
        f"{clean_namespace}/list_www_packages",
        Trigger,
        handle_package_list_request,
    )
    rospy.spin()
