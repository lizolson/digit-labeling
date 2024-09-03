import os
import numpy as np
from scipy.spatial import cKDTree
import cv2

path = '/home/liz/Desktop/research/icra24/data/processed/poncho/'
# folders = [
#     'test_scene1',
#     'test_scene2',
#     'test_scene3',
#     'test_scene4',
#     'test_scene5',
#     'train_scene1',
#     'train_scene10',
#     'train_scene11',
#     'train_scene2',
#     'train_scene3',
#     'train_scene4',
#     'train_scene5',
#     'train_scene6',
#     'train_scene7',
#     'train_scene8',
#     'train_scene9']
#folders = ['high', 'low']
folders = ['high', 'low']

rgb_skip_n = 0 # first several frames have weird colors

def nearest_neighbors_kd_tree(x, y, k) :
    x, y = map(np.asarray, (x, y))
    tree = cKDTree(y[:, None])    
    ordered_neighbors = tree.query(x[:, None], k)[1]
    # nearest_neighbor = np.empty((len(x),), dtype=np.intp)
    # nearest_neighbor.fill(-1)
    # used_y = set()
    # for j, neigh_j in enumerate(ordered_neighbors) :
    #     for k in neigh_j:
    #         if k not in used_y:
    #             nearest_neighbor[j] = k
    #             used_y.add(k)
    #             break
    return ordered_neighbors

cut_rgb = True
resize_rgb = False
for f in os.listdir(path):#folders:
    fpath = path + f
    rgb_files = np.array(sorted(os.listdir(fpath + '/rgb'))[rgb_skip_n:])
    joints_files = np.array(sorted(os.listdir(fpath + '/joints')))
    rgb_timestamp = [int(fname[:-4]) for fname in rgb_files]
    joints_timestamp = [int(fname[:-5]) for fname in joints_files]
    nearest_idx = nearest_neighbors_kd_tree(rgb_timestamp, joints_timestamp, 1)
    joints_match_files = joints_files[nearest_idx]
    #os.system('mkdir -p ' + newpath + f + '/rgb')
    os.system('mkdir -p ' + fpath + '/synced_joints')
    for rgbf, jointsf in zip(rgb_files, joints_match_files):
        os.system('cp ' + fpath + '/joints/' + jointsf + ' ' + path + f  + '/synced_joints/' + rgbf[:-4] + '.json')
