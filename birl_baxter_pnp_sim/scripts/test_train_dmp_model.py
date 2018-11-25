from smach_based_introspection_framework.offline_part import gen_dmp_model
import os
import logging
import ipdb
# create demonstration direction
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstration_dir  = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations')

# create dmp direction
dmp_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'dmp_model_new')
if not os.path.isdir(dmp_model_dir):
    os.makedirs(dmp_model_dir)

# logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# logger = logging.getLogger()
# logger.setLevel(logging.DEBUG)
ipdb.set_trace()
gen_dmp_model.run(demonstration_dir, dmp_model_dir)
