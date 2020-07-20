import h5py
from torcsnet_keras import torcsnet


with h5py.File('ole_1L_01.hdf5', 'r') as f:
    images = f['images'][:]
