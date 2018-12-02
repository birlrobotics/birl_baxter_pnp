from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


def list_to_pose(_list):
    _pose = Pose()
    _pose.position.x = _list[0]
    _pose.position.y = _list[1]
    _pose.position.z = _list[2]
    _pose.orientation.x = _list[3]
    _pose.orientation.y = _list[4]
    _pose.orientation.z = _list[5]
    _pose.orientation.w = _list[6]
    return _pose