import rospy 
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import sys



def get_model_pose(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    req = GetModelStateRequest()
    req.model_name = model_name
    try:
        client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        res = client(req)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return res.pose

def main():
    rospy.init_node("test_get_model_state")
    model_name = "box_male"
    model_pose = get_model_pose(model_name)
    print model_pose


if __name__ == '__main__':
    sys.exit(main())       