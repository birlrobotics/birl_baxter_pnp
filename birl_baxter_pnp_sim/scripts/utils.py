from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

def handle_object_in_gazebo_offset(pose_list):
    defalt_offset_z = 0.918
    pose_list[2] = pose_list[2] - defalt_offset_z
    return pose_list

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

def pose_to_list(_pose):
    return [_pose.position.x,_pose.position.y,_pose.position.z,
    _pose.orientation.x,_pose.orientation.y,_pose.orientation.z,_pose.orientation.w]
