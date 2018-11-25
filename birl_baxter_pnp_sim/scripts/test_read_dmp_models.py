import dill
import os
import ipdb
import baxter_interface
import rospy

# dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
# dmp_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'dmp_models')
# ipdb.set_trace()

# home_to_pre_pick = dill.load(open(os.path.join(dmp_model_dir, 'home_to_pre_pick'), 'r'))

rospy.init_node("test_dmp")
limb = 'right'
limb_interface = baxter_interface.limb.Limb(limb)
# pose_index = [pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w]
current_angles = limb_interface.endpoint_pose()

def _get_dmp_plan(robot, group, dmp_model, goal, goal_modification_info=None):
    list_of_postfix = get_eval_postfix(dmp_cmd_fields, 'pose')

    current_pose = group.get_current_pose().pose
    start = numpy.array(cook_array_from_object_using_postfixs(list_of_postfix, current_pose))
    end = numpy.array(cook_array_from_object_using_postfixs(list_of_postfix, goal))

    if goal_modification_info is not None:
        gmst = rospy.Time.now().to_sec()
        new_goal = copy.deepcopy(end)
        if 'translation' in goal_modification_info:
            pxyz_idx = goal_modification_info['translation']['index'] 
            new_goal[pxyz_idx] = end[pxyz_idx]+goal_modification_info['translation']['value']
        if 'quaternion_rotation' in goal_modification_info:
            qxyzw_idx = goal_modification_info['quaternion_rotation']['index']
            rot_q = goal_modification_info['quaternion_rotation']['value']
            new_goal[qxyzw_idx] = quaternion_multiply(rot_q, end[qxyzw_idx])
        end = new_goal
        gmet = rospy.Time.now().to_sec()
        rospy.logdebug("Took %s seconds to modify goal"%(gmet-gmst))

    st = rospy.Time.now().to_sec()
    command_matrix = generalize_via_dmp(start, end, dmp_model)
    rospy.logdebug("Took %s seconds to call generalize_via_dmp"%(rospy.Time.now().to_sec()-st))
    st = rospy.Time.now().to_sec()
    command_matrix = interpolate_pose_using_slerp(command_matrix, dmp_cmd_fields)
    rospy.logdebug("Took %s seconds to call interpolate_pose_using_slerp"%(rospy.Time.now().to_sec()-st))

    if goal_modification_info is None:
        update_goal_vector(numpy.asarray(command_matrix[-1]).reshape(-1).tolist())
    
    st = rospy.Time.now().to_sec()
    plan, fraction = _get_moveit_plan(robot, group, command_matrix, dmp_cmd_fields, 'pose')
    rospy.logdebug("Took %s seconds to call _get_moveit_plan"%(rospy.Time.now().to_sec()-st))

    rospy.loginfo('moveit plan success rate %s'%fraction)
    return plan, fraction

def get_dmp_plan(robot, group, dmp_model, goal, goal_modification_info=None):
    while not rospy.is_shutdown():
        plan, fraction = _get_dmp_plan(robot, group, dmp_model, goal, goal_modification_info)
        if fraction < 1:
            rospy.loginfo("fraction %s, hit 'Enter' to plan again. or enter anything to move on."%fraction)
            #dmp_model['gen_ay'] = numpy.random.randint(20)
            s = raw_input()
            if s == '':
                continue
            else:
                break
        else:
            rospy.loginfo("plan succeeded, press Enter to execute.")
            #s = raw_input()
            break

    if rospy.is_shutdown():
        raise Exception("rospy.is_shutdown")

    return plan