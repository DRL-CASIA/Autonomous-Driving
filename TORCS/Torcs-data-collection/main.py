import tqdm
import h5py
from environment import Env

env = Env()
_ = env.reset()

'''Setting the variable before running.
'''
step_limit = 3500
track_name = 'e-track6'
# End of setting

all_host_data = {}
all_dc1_data = {}
all_dc2_data = {}
all_dc3_data = {}

for step in  tqdm.tqdm(range(step_limit), ncols=64):
    host_data = env.read_host_data() # dict type
    dc1_data = env.read_dc1_data()
    dc2_data = env.read_dc2_data()
    dc3_data = env.read_dc3_data()
    all_host_data[step] = host_data
    all_dc1_data[step] = dc1_data
    all_dc2_data[step] = dc2_data
    all_dc3_data[step] = dc3_data
    env.step_dc(0.)
env.end()

print("Saving the dataset...")
with h5py.File("./dataset/%s.h5"%track_name, 'w') as f:
    host_group = f.create_group('host_data')
    dc1_group = f.create_group('dc1_data')
    dc2_group = f.create_group('dc2_data')
    dc3_group = f.create_group('dc3_data')

    for i in range(step_limit):
        hgrp_i = host_group.create_group('frame_'+str(i))
        keys = all_host_data[i].keys()
        for key in keys:
            hgrp_i.create_dataset(key, data=all_host_data[i][key])

        keys = all_dc1_data[i].keys()
        dc1_grp_i = dc1_group.create_group('frame_'+str(i))
        dc2_grp_i = dc2_group.create_group('frame_'+str(i))
        dc3_grp_i = dc3_group.create_group('frame_'+str(i))
        for key in keys:
            dc1_grp_i.create_dataset(key, data=all_dc1_data[i][key])
            dc2_grp_i.create_dataset(key, data=all_dc2_data[i][key])
            dc3_grp_i.create_dataset(key, data=all_dc3_data[i][key])

print("Dataset is saved to: ./dataset/%s.h5"%track_name)
