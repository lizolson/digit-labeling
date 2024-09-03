import os
import numpy as np
from scipy.spatial import cKDTree
import cv2

path = '/home/liz/Desktop/research/icra24/data/processed/poncho/'
newpath = path + 'aligned/'
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
    depth_files = np.array(sorted(os.listdir(fpath + '/depth')))
    rgb_timestamp = [int(fname[:-4]) for fname in rgb_files]
    depth_timestamp = [int(fname[:-4]) for fname in depth_files]
    nearest_idx = nearest_neighbors_kd_tree(rgb_timestamp, depth_timestamp, 1)
    depth_match_files = depth_files[nearest_idx]
    #os.system('mkdir -p ' + newpath + f + '/rgb')
    os.system('mkdir -p ' + fpath + '/synced_depth')
    for rgbf, depthf in zip(rgb_files, depth_match_files):
        if not cut_rgb:
            os.system('cp ' + fpath + '/rgb/' + rgbf + ' ' + newpath + f + '/rgb/' + rgbf)
            os.system('cp ' + fpath + '/depth/' + depthf + ' ' + newpath + f  + '/depth/' + rgbf)
        else:
            #rgb_img = cv2.imread(fpath + '/rgb/' + rgbf)
            #rgb_img_new = rgb_img[32:-32]
            #if resize_rgb:
            #    rgb_img_new = cv2.resize(rgb_img_new, (640, 480), cv2.INTER_AREA)
            #cv2.imwrite(newpath + f + '/rgb/' + rgbf, rgb_img_new)
            depth_img_new = cv2.imread(fpath + '/depth/' + depthf, -1)
            # depth_img_new = np.zeros((960, 1280), dtype=np.uint16)
            # for i in range(2):
            #     for j in range(2):
            #         depth_img_new[i::2, j::2] = depth_img
            #print(fpath + '/depth/' + depthf)
            cv2.imwrite(fpath  + '/synced_depth/' + rgbf, depth_img_new)
