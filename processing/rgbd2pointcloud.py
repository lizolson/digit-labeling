#!/usr/bin/env python3

import pdb
import sys 
import numpy as np
from PIL import Image
import imageio
import struct
import os
def get_pointcloud(color_image,depth_image,camera_intrinsics):
    """ creates 3D point cloud of rgb images by taking depth information
        input : color image: numpy array[h,w,c], dtype= uint8
                depth image: numpy array[h,w] values of all channels will be same
        output : camera_points, color_points - both of shape(no. of pixels, 3)
    """

    image_height = depth_image.shape[0]
    image_width = depth_image.shape[1]
    pixel_x,pixel_y = np.meshgrid(np.linspace(0,image_width-1,image_width),
                                  np.linspace(0,image_height-1,image_height))
    camera_points_x = np.multiply(pixel_x-camera_intrinsics[0,2],depth_image/camera_intrinsics[0,0])
    camera_points_y = np.multiply(pixel_y-camera_intrinsics[1,2],depth_image/camera_intrinsics[1,1])
    camera_points_z = depth_image
    camera_points = np.array([camera_points_x,camera_points_y,camera_points_z]).transpose(1,2,0).reshape(-1,3) * 1000

    color_points = color_image.reshape(-1,3)

    # Remove invalid 3D points (where depth == 0)
    flat = depth_image.flatten()

    valid_depth_ind = np.where((flat > 0) & (flat < 3048))[0]
    #close_depth_ind = np.where(flat < 3048)[0]
    
    #goodset = np.array([x for x in valid_depth_ind if x in close_depth_ind])

    #valid_depth_ind = goodset
    camera_points = camera_points[valid_depth_ind,:] * 0.001 * 0.001
    color_points = color_points[valid_depth_ind,:]
    #color_points = color_points[np.where(camera_points[:, 1] < 0.75)]
    #camera_points = camera_points[np.where(camera_points[:, 1] < 0.75)]
    return camera_points,color_points

def write_pointcloud(filename,xyz_points,rgb_points=None):

    """ creates a .pkl file of the point clouds generated
    """

    assert xyz_points.shape[1] == 3,'Input XYZ points should be Nx3 float array'
    if rgb_points is None:
        rgb_points = np.ones(xyz_points.shape).astype(np.uint8)*255
    assert xyz_points.shape == rgb_points.shape,'Input RGB colors should be Nx3 float array and have same size as input XYZ points'

    # Write header of .ply file
    fid = open(filename,'wb')
    fid.write(bytes('ply\n', 'utf-8'))
    fid.write(bytes('format binary_little_endian 1.0\n', 'utf-8'))
    fid.write(bytes('element vertex %d\n'%xyz_points.shape[0], 'utf-8'))
    fid.write(bytes('property float x\n', 'utf-8'))
    fid.write(bytes('property float y\n', 'utf-8'))
    fid.write(bytes('property float z\n', 'utf-8'))
    fid.write(bytes('property uchar red\n', 'utf-8'))
    fid.write(bytes('property uchar green\n', 'utf-8'))
    fid.write(bytes('property uchar blue\n', 'utf-8'))
    fid.write(bytes('end_header\n', 'utf-8'))

    #xyz_points = xyz_points * 0.001#* -1#* -0.001
    #xyz_points = xyz_points * 0.001#* -1#* -0.001
    # Write 3D points to .ply file
    for i in range(xyz_points.shape[0]):
        fid.write(bytearray(struct.pack("fffccc",xyz_points[i,0],xyz_points[i,1],xyz_points[i,2],
                                        rgb_points[i,0].tostring(),rgb_points[i,1].tostring(),
                                        rgb_points[i,2].tostring())))
    fid.close()




############################################################
#  Main
############################################################

if __name__ == '__main__':
    #'''
    dirr_og = sys.argv[1]
    for setup in os.listdir(dirr_og):
        if 'poncho' not in setup:
            continue 
        print(setup)
        next_folders = os.listdir(os.path.join(dirr_og, setup))
        next_folders = sorted(next_folders)
        for seq in next_folders:
            #if seq != 'table_2023-09-03-22-41-49':
            #    continue
            dirr = os.path.join(dirr_og, setup, seq)
            for fl in os.listdir(os.path.join(dirr, "rgb")):
                #if '1696812220169287534' not in fl:
                #    continue
                print(fl)
                rgb_filename=os.path.join(dirr, "rgb", fl)
                depth_filename=os.path.join(dirr, "synced_depth", fl)
                #depth_filename=os.path.join(dirr, "depth", fl)
                output_directory=os.path.join(dirr, "plys")
                fx=611.12934701
                fy=614.44197935
                cx=636.86456617
                cy=365.9642761
                if not os.path.exists(output_directory):
                    os.mkdir(output_directory)
                try:
                    color_data = imageio.imread(rgb_filename)
                except:
                    print(rgb_filename)
                    continue
   
                # color_data = np.asarray(im_color, dtype = "uint8")

                if os.path.splitext(os.path.basename(depth_filename))[1] == '.npy':
                    depth_data = np.load(depth_filename)
                else:
                    try:
                        im_depth = imageio.imread(depth_filename)
                    except:
                        print(depth_filename)
                    depth_data = im_depth[:,:] # values of all channels are equal

                # camera_intrinsics  = [[fx 0 cx],
                #                       [0 fy cy],
                #                       [0 0 1]]
                camera_intrinsics  = np.asarray([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

                output_filename = os.path.join(output_directory, fl[:-3] + 'ply')

                camera_points, color_points = get_pointcloud(color_data, depth_data, camera_intrinsics)

                write_pointcloud(output_filename, camera_points, color_points)
