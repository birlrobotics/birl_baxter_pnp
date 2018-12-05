import numpy
import ipdb
from trac_ik_solver import convert_pose_to_joint_plan
from quaternion_interpolation import interpolate_pose_using_slerp
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def filter_static_points(mat):
    last = mat[0]
    new_mat = [last]
    for idx in range(mat.shape[0]):
        if numpy.linalg.norm(mat[idx]-last) < 0.01 \
            and idx != mat.shape[0]-1:
            pass
        else:
            new_mat.append(mat[idx])
            last = mat[idx]
    return numpy.array(new_mat)
    
def get_dmp_model(mat, model_type='pydmps'):
    if model_type == 'pydmps':
        import pydmps.dmp_discrete
        n_dmps = mat.shape[1]
        dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps, n_bfs=100, ay=numpy.ones(n_dmps)*5)
        dmp.imitate_path(y_des=mat.T)
        return dmp

def generalize_via_dmp(start, end, model,plot=True):
    import pydmps
    import ipdb
    new_dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=model.n_dmps, n_bfs=model.n_bfs, ay=model.ay, w=model.w)  
 
    start = numpy.array(start)
    end = numpy.array(end)
    new_dmp.y0 = start
    new_dmp.goal = end
    y_track, dy_track, ddy_track = new_dmp.rollout(tau=1)

    return y_track
        
def generalize_via_dmp_no_start_end(model,plot=True):
    import pydmps
    import ipdb
    new_dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=model.n_dmps, n_bfs=model.n_bfs, ay=model.ay, w=model.w)  

    y_track, dy_track, ddy_track = new_dmp.rollout(tau=1)

    return y_track

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

def get_dmp_joint_plan(start, end, demo,limb):
    demo = filter_static_points(demo)
    dmp_model = get_dmp_model(demo)
    command_matrix = generalize_via_dmp(start, end, dmp_model) 
    command_matrix = interpolate_pose_using_slerp(command_matrix)
    plot(command_matrix)
    dmp_angle_plan = convert_pose_to_joint_plan(command_matrix,limb)
    return dmp_angle_plan