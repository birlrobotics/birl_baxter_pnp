import sys,os
import rospy
import copy
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import ipdb
from _constant import gazebo_model_dir,model_name,model_pose
from utils import list_to_pose,pose_to_list,handle_object_in_gazebo_offset
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from _constant import hover_height



def delete_gazebo_models():
# This will be called on ROS Exit, deleting Gazebo models
# Do not wait for the Gazebo Delete Model service, since
# Gazebo should already be running. If the service is not
# available since Gazebo has been killed, it is fine to error out
    for model in model_name:
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model(model)
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))
    else:
        pass


def load_gazebo_models(model_name,model_pose, model_type ="sdf",
                       model_reference_frame="base"):
        model_path = gazebo_model_dir
        if model_type == "sdf":
            with open (os.path.join(model_path,model_name)+ ".sdf" , "r") as _file:
                _xml = _file.read().replace('\n', '')
                rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                resp_sdf = spawn_sdf(model_name, _xml, "/", model_pose, model_reference_frame)
                rospy.loginfo("loading %s succesfully", model_name)
            except rospy.ServiceException, e:
                rospy.logerr("Spawn SDF service call failed: {0}".format(e))
                return False
            return True

        elif model_type == "urdf":
            with open (os.path.join(model_path,model_name)+ ".urdf", "r") as _file:
                _xml = _file.read().replace('\n', '')
                rospy.wait_for_service('/gazebo/spawn_urdf_model')
            try:
                spawn_urdf  = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
                resp_urdf = spawn_urdf(model_name, _xml, "/", model_pose, model_reference_frame)
                rospy.loginfo("loading %s succesfully", model_name)
            except rospy.ServiceException, e:
                rospy.logerr("Spawn URDF service call failed: {0}".format(e))
                return False
            return True
        else:
            rospy.logerr("format is not match")


# def get_model_pose(model_name,hover_flag=None):
#     rospy.wait_for_service('/gazebo/get_model_state')
#     req = GetModelStateRequest()
#     req.model_name = model_name
#     try:
#         client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
#         res = client(req)
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e
#     model_pose = copy.deepcopy(res.pose)
#     pose_list = pose_to_list(model_pose)
#     pose_list = handle_object_in_gazebo_offset(pose_list)
#     if hover_flag != None:
#         pose_list[2] = pose_list[2] + hover_height
#         return pose_list
#     pose_list[2] = pose_list[2] - 0.009
#     return pose_list

def get_model_pose(model_name,hover_flag=None):
    pose_list = copy.deepcopy(model_pose[0])
    if hover_flag != None:
        pose_list[2] = pose_list[2] + hover_height
        return pose_list
    pose_list[2] = pose_list[2] - 0.009
    return pose_list


def run_get_model_pose():
    rospy.init_node("test_get_model_state")
    model_name = "box_male"
    model_pose = get_model_pose(model_name)
    print model_pose


def add_gazebo_models():
    pick_pose = list_to_pose(model_pose[0])

    delete_gazebo_models()

    load_gazebo_models(model_name=model_name[0],
                    model_pose=pick_pose,
                    model_type = "urdf",
                    model_reference_frame="base")   


if __name__ == '__main__':
    rospy.init_node("test_add_models")
    sys.exit(add_gazebo_models())                       
