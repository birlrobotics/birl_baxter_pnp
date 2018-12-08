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

