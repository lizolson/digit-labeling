import time
import pdb
import json
import sys
import os
import numpy as np
import cv2
import shutil

#path = '/media/liz/DATA/icra24/processed/' + sys.argv[1]
head_path = '/home/liz/Desktop/research/icra24/data/processed/' #+ sys.argv[1]
release_path = '/home/liz/Desktop/research/icra24/data/release/' #+ sys.argv[1]


cut_rgb = True
resize_rgb = False
for folder in sorted(os.listdir(head_path)):
    if folder != 'poncho':
        continue
    path = head_path + folder
    if len(sys.argv) > 1:
        if folder != sys.argv[1]:
            continue
    print('\n\n' + folder + '\n\n')
    for subfolder in os.listdir(path):
        if not os.path.exists(os.path.join(release_path, folder, subfolder)):
            os.mkdir(os.path.join(release_path, folder, subfolder))
            os.mkdir(os.path.join(release_path, folder, subfolder, 'rgb'))
            os.mkdir(os.path.join(release_path, folder, subfolder, 'depth'))
            os.mkdir(os.path.join(release_path, folder, subfolder, 'plys'))
            os.mkdir(os.path.join(release_path, folder, subfolder, 'joints'))
        fpath = os.path.join(path, subfolder)
        print(fpath)
        rgb_files = np.array(sorted(os.listdir(fpath + '/rgb')))
        if not os.path.exists(fpath + '/synced_depth'):
            print('no synced')
            print(len(rgb_files))
            continue
        depth_files = np.array(sorted(os.listdir(fpath + '/synced_depth')))
        joints_files = np.array(sorted(os.listdir(fpath + '/synced_joints')))
        print(len(rgb_files), len(depth_files), len(joints_files))
        rgb_timestamp = [int(fname[:-4]) for fname in rgb_files]
        depth_timestamp = [int(fname[:-4]) for fname in depth_files]
        joints_timestamp = [int(fname[:-5]) for fname in joints_files]
        last_rgb = None
        last_depth = None
        for nidx, num in enumerate(rgb_timestamp):
            if num not in depth_timestamp:
                print(num, "NO depth----------------------------------------------------------------------------")
            if num not in joints_timestamp:
                print(num, "NO joints----------------------------------------------------------------------------")
            rgb = cv2.imread(fpath + '/rgb/' + str(num) + '.png') 
            depth = cv2.imread(fpath + '/synced_depth/' + str(num) + '.png') 

            f = open(fpath + '/synced_joints/' + str(num) + '.json')
            #print(num)
            data = json.load(f)
            if  len(data['q_all']) != 30:
                print(num, 'joints len ---------------------------------------------------------------------------')
            if rgb is None:
                print('---------------------------------------------------------------------------')
                print(os.path.join(folder, subfolder) + '/rgb/' + str(num) + '.png') 
            if depth is None:
                print('---------------------------------------------------------------------------')
                print(os.path.join(folder, subfolder) + '/synced_depth/' + str(num) + '.png') 
            if last_rgb != None:
                if last_rgb == rgb:
                    print(num, 'rgb duplicated')
            if last_depth != None:
                if last_depth == depth:
                    print(num, 'depth duplicated')
            rgb = last_rgb
            depth = last_depth
            shutil.copy(fpath + '/rgb/' + str(num) + '.png', os.path.join(release_path, folder, subfolder, 'rgb', str(nidx).zfill(3) + '.png')) 
            shutil.copy(fpath + '/synced_depth/' + str(num) + '.png', os.path.join(release_path, folder, subfolder, 'depth', str(nidx).zfill(3) + '.png')) 
            shutil.copy(fpath + '/synced_joints/' + str(num) + '.json', os.path.join(release_path, folder, subfolder, 'joints', str(nidx).zfill(3) + '.json')) 
            shutil.copy(fpath + '/plys/' + str(num) + '.ply', os.path.join(release_path, folder, subfolder, 'plys', str(nidx).zfill(3) + '.ply')) 
            #time.sleep(.5)       
