import sys,os
import rospy
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
from _constant import gazebo_model_dir,models,pick_list,models


def delete_gazebo_models():
# This will be called on ROS Exit, deleting Gazebo models
# Do not wait for the Gazebo Delete Model service, since
# Gazebo should already be running. If the service is not
# available since Gazebo has been killed, it is fine to error out
    for model in models:
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model(model)
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))
    else:
        pass


def load_gazebo_models(model_name,model_pose, model_type ="sdf",
                       model_reference_frame="base"):
        global Count
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
                resp_urdf = spawn_sdf(model_name, _xml, "/", model_pose, model_reference_frame)
                rospy.loginfo("loading %s succesfully", model_name)
            except rospy.ServiceException, e:
                rospy.logerr("Spawn URDF service call failed: {0}".format(e))
                return False
            return True
        else:
            rospy.logerr("format is not match")

def add_gazebo_models():

    pick_pose = list_to_pose(pick_list)
    delete_gazebo_models()
    load_gazebo_models(model_name=models[0],
                        model_pose=pick_pose,
                        model_type = "sdf",
                        model_reference_frame="base")    


if __name__ == '__main__':
    rospy.init_node("test_add_models")
    sys.exit(add_gazebo_models())                       
