from dmp_util import dmp_plan




def dmp_plan(start, end, demo,limb):
    demo = filter_static_points(demo)
    dmp_model = get_dmp_model(demo)
    command_matrix = generalize_via_dmp(start, end, dmp_model) 
    command_matrix = interpolate_pose_using_slerp(command_matrix)
    plot(command_matrix)
    dmp_angle_plan = convert_pose_to_joint_plan(command_matrix,limb)
    return dmp_angle_plan

    
def plot(demo,fig_idx=0, title_="human_demonstration",saving_fig_name=None ):
    fig = plt.figure(fig_idx)
    ax = fig.gca(projection='3d')
    ax.plot(demo[:,0],demo[:,1],demo[:,2])
    ax.scatter(demo[0,0],demo[0,1],demo[0,2], label="start", color="g",alpha=1)
    ax.scatter(demo[-1,0],demo[-1,1],demo[-1,2], label="end", color="r",alpha=1)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    # ax.set_xlim(-0.5,0.2)
    # ax.set_ylim(min_lim,max_lim)
    # ax.set_zlim(min_lim,max_lim)
    ax.set_title(title_)
    ax.legend()
    if saving_fig_name!= None:
        fig.savefig( os.path.join(fig_saving_path,saving_fig_name), format='png',dpi=300,bbox_inches='tight')
    plt.show()