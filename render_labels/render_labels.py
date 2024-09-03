#import pip
#import pip
#pip.main(['install', 'open3d', '--user'])
#breakpoint()
import imageio
import pickle
import copy
#import open3d as o3d
import math
import pdb
import cv2
import random
import numpy as np
import glob
import os
from math import *
import json
from scipy.spatial.transform import Rotation as R
from typing import Union, Tuple, Optional
import sys
import bpy
import bpycv
import random
import glob
import os
from math import *
import mathutils
from mathutils import Matrix as mat
from mathutils import Quaternion
from mathutils import Vector
from mathutils import Euler
import time
from urdf_parser_py.urdf import URDF
import urdfpy



with open('urdf/digit_model.urdf', ) as f:
    digit = URDF.from_xml_string(f.read())

my_digit = urdfpy.URDF.load('urdf/absolute_digit_model.urdf')

f_x = 611.12934701
f_y = 614.44197935
c_x =  636.86456617
c_y =  365.9642761

#curcam = []

#sys.path.insert(0, '/ /Applications/Blender.app/Contents/Resources/3.5/python/lib/python3.10/site-packages/open3d/')
import open3d as o3d

def pixel_to_3d(u, v, depth_image):
    #print('transformed points:')
    #print(x, y, depth_transformed)
    x_img = 0.001 *(u - c_x) * depth_image/f_x
    y_img = 0.001 * (v - c_y) * depth_image/f_y
    #print('computed points:')
    #print(x_img, y_img, depth_image * 0.001)
    return x_img, y_img, depth_image * 0.001

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #o3d.io.write_point_cloud('/home/liz/temp.ply', source_temp)
    source_temp.transform(transformation)
    #source_temp.paint_uniform_color([1, 0.706, 0])
    #target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])

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
     
    color_points_temp = color_image.reshape(-1,3) / 255.0
    color_points = copy.copy(color_points_temp)
    color_points[: , 0] = color_points_temp[:, 2]
    color_points[: , 2] = color_points_temp[:, 0]

    # Remove invalid 3D points (where depth == 0)
    flat = depth_image.flatten()

    valid_depth_ind = np.where((flat > 0) & (flat < 3048))[0]
    #close_depth_ind = np.where(flat < 3048)[0]
    
    #goodset = np.array([x for x in valid_depth_ind if x in close_depth_ind])

    #valid_depth_ind = goodset
    camera_points = camera_points[valid_depth_ind,:] * 0.001
    color_points = color_points[valid_depth_ind,:]
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


    #xyz_points = xyz_points * 0.001#* -1#* -0.001
    #xyz_points = xyz_points * 0.001#* -1#* -0.001
    #print(xyz_points)
    # Write 3D points to .ply file
    for i in range(xyz_points.shape[0]):
        fid.write(bytearray(struct.pack("fffccc",xyz_points[i,0],xyz_points[i,1],xyz_points[i,2],
                                        rgb_points[i,0].tostring(),rgb_points[i,1].tostring(),
                                        rgb_points[i,2].tostring())))
    fid.close()







obj_name = {
    1: "hip-yaw-housing",
    2: "hip-pitch-housing",
    3: "hip-pitch",
    4: "knee",
    5: "shin",
    6: "tarsus",
    7: "arm-L1",
    8:"arm-L2",
    9:"arm-L3",
    10:"arm-L4",
    11: "hip-yaw-housing" ,
    12: "hip-pitch-housing",
    13: "hip-pitch",
    14:  "knee",
    15: "shin", 
    16: "tarsus",
    17:"arm-L1",
    18:"arm-L2",
    19:"arm-L3",
    20:"arm-L4",
    0: "torso",
    21: "toe-pitch",
    22: "toe-pitch",
    23: "toe-roll",
    24: "toe-roll",
    25: "hip-roll-housing",
    26: "hip-roll-housing",
    27: "shoulder-roll-housing",
    28: "shoulder-roll-housing"
}


part_names_list = ["torso", 
    "left_hip_roll",
    "left_hip_yaw",
    "left_hip_pitch",
    "left_knee",
    "left_shin", #17
    "left_tarsus", #18
    "left_shoulder_roll",#9
    "left_shoulder_pitch",#10
    "left_shoulder_yaw",#11
    "left_elbow",#12
    "right_hip_roll", #5
    "right_hip_yaw", #6
    "right_hip_pitch", #7
    "right_knee",#8
    "right_shin", #19
    "right_tarsus",#20
    "right_shoulder_roll", #13
    "right_shoulder_pitch",#14
    "right_shoulder_yaw",#15
    "right_elbow",
    "left_toe_pitch",
    "right_toe_pitch",
    "left_toe_roll",
    "right_toe_roll",
    "right_waist_cap",
    'left_waist_cap',
    'right_shoulder_cap',
    'left_shoulder_cap'
]




motor_names = ["left-hip-roll","left-hip-yaw","left-hip-pitch","left-knee","left-toe-A","left-toe-B",
    "right-hip-roll","right-hip-yaw","right-hip-pitch","right-knee","right-toe-A","right-toe-B",
    "left-shoulder-roll","left-shoulder-pitch","left-shoulder-yaw","left-elbow",
    "right-shoulder-roll","right-shoulder-pitch","right-shoulder-yaw","right-elbow"]
joint_names = ["left-shin", "left-tarsus", "left-toe-pitch", "left-toe_roll", "left-heel-spring",
    "right-shin", "right-tarsus", "right-toe-pitch", "right-toe_roll", "right-heel-spring"]
q_all = [
    "base-pos-x", "base-pos-y", "base-pos-z",
    "base-yaw","base-pitch","base-roll",
    "hip_roll_joint_left","hip_yaw_joint_left","hip_pitch_joint_left","knee_joint_left","knee_to_shin_left","shin_to_tarsus_left","toe_pitch_joint_left","toe_roll_joint_left",
    "shoulder_roll_joint_left","shoulder_pitch_joint_left","shoulder_yaw_joint_left","elbow_joint_left",
    "hip_roll_joint_right","hip_yaw_joint_right","hip_pitch_joint_right","knee_joint_right","knee_to_shin_right","shin_to_tarsus_right","toe_pitch_joint_right","toe_roll_joint_right",
    "shoulder_roll_joint_right","shoulder_pitch_joint_right","shoulder_yaw_joint_right","elbow_joint_right",
    ]



joint_values = {}
pose = {} 
poses_fk = {}

def project(t_co, intrinsics):
    # return homogeneous coordinate todo: maybe make this generic if time permits
    uv = np.matmul(intrinsics, t_co)
    if uv.shape[0] == 1:
        uv = uv[0]
    uv_h = uv / uv[2]
    return uv_h


def set_intrinsics_from_K_matrix(K: Union[np.ndarray, mat], image_width: int, image_height: int,
                                 clip_start: float = None, clip_end: float = None):
    """ Set the camera intrinsics via a K matrix.

    The K matrix should have the format:
        [[fx, 0, cx],
         [0, fy, cy],
         [0, 0,  1]]

    This method is based on https://blender.stackexchange.com/a/120063.

    :param K: The 3x3 K matrix.
    :param image_width: The image width in pixels.
    :param image_height: The image height in pixels.
    :param clip_start: Clipping start.
    :param clip_end: Clipping end.
    """

    K = mat(K)

    cam = bpy.data.objects["Camera"].data
    #cam = bpy.context.scene.camera.data

    if abs(K[0][1]) > 1e-7:
        raise ValueError(f"Skew is not supported by blender and therefore "
                         f"not by BlenderProc, set this to zero: {K[0][1]} and recalibrate")

    fx, fy = K[0][0], K[1][1]
    cx, cy = K[0][2], K[1][2]

    # If fx!=fy change pixel aspect ratio
    pixel_aspect_x = pixel_aspect_y = 1
    if fx > fy:
        pixel_aspect_y = fx / fy
    elif fx < fy:
        pixel_aspect_x = fy / fx

    # Compute sensor size in mm and view in px
    pixel_aspect_ratio = pixel_aspect_y / pixel_aspect_x
    view_fac_in_px = get_view_fac_in_px(cam, pixel_aspect_x, pixel_aspect_y, image_width, image_height)
    sensor_size_in_mm = get_sensor_size(cam)

    # Convert focal length in px to focal length in mm
    f_in_mm = fx * sensor_size_in_mm / view_fac_in_px

    # Convert principal point in px to blenders internal format
    shift_x = (cx - (image_width - 1) / 2) / -view_fac_in_px
    shift_y = (cy - (image_height - 1) / 2) / view_fac_in_px * pixel_aspect_ratio

    # Finally set all intrinsics
    set_intrinsics_from_blender_params(f_in_mm, image_width, image_height, clip_start, clip_end, pixel_aspect_x,
                                       pixel_aspect_y, shift_x, shift_y, "MILLIMETERS")

def set_intrinsics_from_blender_params(lens: float = None, image_width: int = None, image_height: int = None,
                                       clip_start: float = None, clip_end: float = None,
                                       pixel_aspect_x: float = None, pixel_aspect_y: float = None, shift_x: int = None,
                                       shift_y: int = None, lens_unit: str = None):
    """ Sets the camera intrinsics using blenders represenation.

    :param lens: Either the focal length in millimeters or the FOV in radians, depending on the given lens_unit.
    :param image_width: The image width in pixels.
    :param image_height: The image height in pixels.
    :param clip_start: Clipping start.
    :param clip_end: Clipping end.
    :param pixel_aspect_x: The pixel aspect ratio along x.
    :param pixel_aspect_y: The pixel aspect ratio along y.
    :param shift_x: The shift in x direction.
    :param shift_y: The shift in y direction.
    :param lens_unit: Either FOV or MILLIMETERS depending on whether the lens is defined as focal length in
                      millimeters or as FOV in radians.
    """

    cam_ob = bpy.context.scene.camera
    cam = cam_ob.data


    if lens_unit is not None:
        cam.lens_unit = lens_unit

    if lens is not None:
        # Set focal length
        if cam.lens_unit == 'MILLIMETERS':
            if lens < 1:
                raise Exception("The focal length is smaller than 1mm which is not allowed in blender: " + str(lens))
            cam.lens = lens
        elif cam.lens_unit == "FOV":
            cam.angle = lens
        else:
            raise Exception("No such lens unit: " + lens_unit)

    # Set resolution
    if image_width is not None:
        bpy.context.scene.render.resolution_x = image_width
    if image_height is not None:
        bpy.context.scene.render.resolution_y = image_height

    # Set clipping
    if clip_start is not None:
        cam.clip_start = clip_start
    if clip_end is not None:
        cam.clip_end = clip_end

    # Set aspect ratio
    if pixel_aspect_x is not None:
        bpy.context.scene.render.pixel_aspect_x = pixel_aspect_x
    if pixel_aspect_y is not None:
        bpy.context.scene.render.pixel_aspect_y = pixel_aspect_y

    # Set shift
    if shift_x is not None:
        cam.shift_x = shift_x
    if shift_y is not None:
        cam.shift_y = shift_y


def get_view_fac_in_px(cam: bpy.types.Camera, pixel_aspect_x: float, pixel_aspect_y: float,
                       resolution_x_in_px: int, resolution_y_in_px: int) -> int:
    """ Returns the camera view in pixels.

    :param cam: The camera object.
    :param pixel_aspect_x: The pixel aspect ratio along x.
    :param pixel_aspect_y: The pixel aspect ratio along y.
    :param resolution_x_in_px: The image width in pixels.
    :param resolution_y_in_px: The image height in pixels.
    :return: The camera view in pixels.
    """
    # Determine the sensor fit mode to use
    if cam.sensor_fit == 'AUTO':
        if pixel_aspect_x * resolution_x_in_px >= pixel_aspect_y * resolution_y_in_px:
            sensor_fit = 'HORIZONTAL'
        else:
            sensor_fit = 'VERTICAL'
    else:
        sensor_fit = cam.sensor_fit

    # Based on the sensor fit mode, determine the view in pixels
    pixel_aspect_ratio = pixel_aspect_y / pixel_aspect_x
    if sensor_fit == 'HORIZONTAL':
        view_fac_in_px = resolution_x_in_px
    else:
        view_fac_in_px = pixel_aspect_ratio * resolution_y_in_px

    return view_fac_in_px

def get_sensor_size(cam: bpy.types.Camera) -> float:
    """ Returns the sensor size in millimeters based on the configured sensor_fit.

    :param cam: The camera object.
    :return: The sensor size in millimeters.
    """
    if cam.sensor_fit == 'VERTICAL':
        sensor_size_in_mm = cam.sensor_height
    else:
        sensor_size_in_mm = cam.sensor_width
    return sensor_size_in_mm



def render_scene(pose):
    #for i in range(len(bpy.data.objects)):
    #    print(bpy.data.objects[i])
    for link in digit.links:
        cur_link = link.name
        #print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
        #print(cur_link)
        obj = bpy.data.objects[cur_link]
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        cur_pose = mat(pose[cur_link])
        #print(cur_pose)

        pose_loc = cur_pose.to_translation()
        pose_rot = cur_pose.to_euler()
        bpy.data.objects[cur_link].location = [pose_loc[0], pose_loc[1], pose_loc[2]]#pose_loc
        bpy.data.objects[cur_link].rotation_euler = Euler((pose_rot[0], pose_rot[1], pose_rot[2]), 'XYZ')
        
    bpy.context.view_layer.update()

    
    result = bpycv.render_data()
    seq_idx = 0
    #print("shape",result["image"].shape)
    rgb = cv2.cvtColor(result["image"], cv2.COLOR_RGB2BGR)
    #cv2.imwrite('temp.png', rgb)
    #rgb_copy = cv2.cvtColor(result["image"], cv2.COLOR_RGB2BGR)
    #rgb[result['inst'] == 0] = bckgrnd[result['inst'] == 0]
     
    #cv2.imwrite(os.path.join(dataset_dir, 'rgb', str(img_index).zfill(5) + '.png'), rgb)
    #with open(os.path.join(dataset_dir, 'depth', str(img_index).zfill(5) + '.npy'), 'wb') as f:
    #    np.save(f, result["depth"])
    
    depth = result["depth"]
    #cv2.imwrite(os.path.join(dataset_dir, "depth",  "depth_" + str(img_index).zfill(3) +  ".png"), np.uint16(depth_in_m))  # save as 16bit png
    #cv2.imwrite(os.path.join(dataset_dir, "rgb", str(seq_index).zfill(5), "rgb_" + str(img_index).zfill(3) +  ".png"), 2 * np.uint16(result["image"]))  # save as 16bit png
    #print(os.path.join(dataset_dir, "depth", str(seq_index).zfill(5), "depth_" + str(img_index).zfill(3) +  ".png"), np.uint16(depth_in_m))  # save as 16bit png
    #print(os.path.join(dataset_dir, "rgb", str(seq_index).zfill(5), "rgb_" + str(img_index).zfill(3) +  ".png"), 2 * np.uint16(result["image"]))  # save as 16bit png
    seg = result["inst"]
    ycb_6d = result['ycb_6d_pose']
    return (rgb ,depth, seg, ycb_6d)

example_data_dir = "./urdf/"
models = sorted(glob.glob(os.path.join(example_data_dir, "*.obj")))



proj = np.eye(4)
proj[:3, :3] = R.from_euler('XYZ', [0, 180, 0], degrees=True).as_matrix()
bpycv.clear_all()

#sys is processed/setup
#to render: proj @ sequence_transform-1 @ initial @  pose 

setup_folder = sys.argv[1]
 
camera = bpy.data.objects["Camera"]
    
width = 1280
height = 720

pi = math.pi
camera.location = Vector()
#camera.rotation_euler = Euler(( 1 * pi, 0 * pi, 1 * pi), 'XYZ')
camera.rotation_euler = Euler(( 1 * pi, 0 * pi, 0 * pi), 'XYZ')


dpth_near = 0.5
dpth_far = 3.86

w = 1280
h = 720
A = (dpth_near + dpth_far)/(dpth_near - dpth_far)
B = 2 * dpth_near * dpth_far / (dpth_near - dpth_far)


'''projection_matrix = [
                    [2/w * f_x,  0,          0,  0],
                    [0,          2/h * f_y,  0,  0],
                    [0,          0,          A,              B],
                    [0,          0,          -1,             0]]'''
'''projection_matrix = [
                    [2/w * f_x,  0,          (w - 2*c_x)/w + 0.065,  0],
                    [0,          2/h * f_y,  (2*c_y - h)/h - 0.02,  0],
                    [0,          0,          A,              B],
                    [0,          0,          -1,             0]]'''

#TODO compare this projection matrix against using the f_x, f_y, c_x, c_y hardcoded.

K = np.zeros((3, 3))
K[0, 0] = f_x
K[1, 1] = f_y
K[0, 2] = c_x
K[1, 2] = c_y
K[2, 2] = 1

#update_camera(bpy.data.objects['Camera'], focus_point=mathutils.Vector((-0.4170476496219635, 0.09546088427305222, -2.531710386276245)))
set_intrinsics_from_K_matrix(K, w, h)
#print(camera.location)
#print(camera.rotation_euler)
bpycv.clear_all()

light_data = bpy.data.lights.new(name="my-light-data", type='POINT')
#light_data = bpy.data.lamps.new(name="my-light-data", type='POINT')
light_data.energy = 150

# Create new object, pass the light data 
light_object = bpy.data.objects.new(name="my-light", object_data=light_data)

# Link object to collection in context
#bpy.context.collection.objects.link(light_object)
#bpy.context.view_layer.objects.link(light_object)
bpy.context.collection.objects.link(light_object)
bpy.context.view_layer.objects.active = light_object

#bpy.ops.import_mesh.ply(filepath="/Users/top/Desktop/DigitDataProcessing/points/"+str(index)+"-pointCloud.ply")
#bpy.data.objects[str(index)+"-pointCloud"].scale = (0.001, 0.001, 0.001)

# Change light position
light_object.location = (0, -1, 2)
light_object.location = (0, 0, 0)
bpy.context.view_layer.objects.active = light_object
frame_num = 1

bpy.context.scene.render.engine = "CYCLES"
'''
# Set the device_type

# Set the device and feature set

# get_devices() to let Blender detects GPU device
bpy.context.preferences.addons["cycles"].preferences.get_devices()
for d in bpy.context.preferences.addons["cycles"].preferences.devices:
    print(d["use"])
    #d["use"] = 1 # Using all devices, include GPU and CPU
breakpoint()
'''
bpy.context.preferences.addons["cycles"].preferences.compute_device_type = "CUDA" # or "OPENCL"
bpy.context.scene.cycles.device = "GPU"
def enable_gpus(device_type, use_cpus=False):
    preferences = bpy.context.preferences
    cycles_preferences = preferences.addons["cycles"].preferences
    cycles_preferences.refresh_devices()
    devices = cycles_preferences.devices

    if not devices:
        raise RuntimeError("Unsupported device type")

    activated_gpus = []
    for device in devices:
        if device.type == "CPU":
            device.use = True#use_cpus
        else:
            device.use = True
            activated_gpus.append(device.name)
            print('activated gpu', device.name)

    cycles_preferences.compute_device_type = device_type
    bpy.context.scene.cycles.device = "GPU"

    return activated_gpus
enable_gpus("CUDA")
bpy.context.scene.cycles.samples = 20
bpy.context.scene.render.resolution_y = 720
bpy.context.scene.render.resolution_x = 1280
for idx in range(29):
    model = example_data_dir + obj_name[idx]+".obj"
    #print(model)
    #obj = bpy.ops.wm.obj_import(filepath=model)
    obj = bpycv.object_utils.load_obj(model)
    #pdb.set_trace()
    obj.name = part_names_list[idx]
    obj.rotation_mode = "XYZ"
    obj.matrix_world = np.eye(4)
    if (idx >=11 and idx <=20):
        obj.delta_scale = [1, -1, 1]
    obj["inst_id"] = idx+1;
    #if obj_name[idx] == 'torso':
    #    breakpoint()






seqs = os.listdir(setup_folder)
seqs.sort()
for seq in seqs: 
    #if '42_07' in seq or '44_54' in seq:
    #    continue
    if sys.argv[2] not in seq:
        continue
    #if '0' in seq or '1' in seq:
    #    continue
    #if seq != seqs[int(sys.argv[2])]:
    #    continue
    #print("================================+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++===================================================")
    dataset_dir = os.path.join(setup_folder, seq)
    if not os.path.exists(os.path.join(dataset_dir, "pose_visualized")):
        os.makedirs(os.path.join(dataset_dir, "pose_visualized"))
    if not os.path.exists(os.path.join(dataset_dir, "ycb_6d_pose")):
        os.makedirs(os.path.join(dataset_dir, "ycb_6d_pose"))
    if not os.path.exists(os.path.join(dataset_dir, "seg")):
        os.makedirs(os.path.join(dataset_dir, "seg"))
    filelist = os.listdir(os.path.join(setup_folder, seq, "pose"))
    filelist.sort()
    #read in EKF
    #calc deltas
    #perform ICP
    #write new label
    #render new label
    for fidx, flnm in enumerate(filelist):
        if len(sys.argv) > 3:
            if fidx < int(sys.argv[3]):
                continue
            if fidx > int(sys.argv[4]):
                continue
        #if fidx % 3 != 0:
        #    continue

        bpycv.clear_all()
        bpy.context.scene.frame_set(frame_num)
        light_data = bpy.data.lights.new(name="my-light-data", type='POINT')
        #light_data = bpy.data.lamps.new(name="my-light-data", type='POINT')
        light_data.energy = 150

        # Create new object, pass the light data 
        light_object = bpy.data.objects.new(name="my-light", object_data=light_data)

        # Link object to collection in context
        #bpy.context.collection.objects.link(light_object)
        #bpy.context.view_layer.objects.link(light_object)
        bpy.context.collection.objects.link(light_object)
        bpy.context.view_layer.objects.active = light_object

        #bpy.ops.import_mesh.ply(filepath="/Users/top/Desktop/DigitDataProcessing/points/"+str(index)+"-pointCloud.ply")
        #bpy.data.objects[str(index)+"-pointCloud"].scale = (0.001, 0.001, 0.001)

        # Change light position
        light_object.location = (0, -1, 2)
        light_object.location = (0, 0, 0)
        bpy.context.view_layer.objects.active = light_object
        for idx in range(29):
            model = example_data_dir + obj_name[idx]+".obj"
            #print(model)
            #obj = bpy.ops.wm.obj_import(filepath=model)
            obj = bpycv.object_utils.load_obj(model)
            #pdb.set_trace()
            obj.name = part_names_list[idx]
            obj.rotation_mode = "XYZ"
            obj.matrix_world = np.eye(4)
            if (idx >=11 and idx <=20):
                obj.delta_scale = [1, -1, 1]
            obj["inst_id"] = idx+1;
        frame_num += 1




        label_file = os.path.join(setup_folder, seq, "pose", flnm)
        rgb_file = label_file.replace('pose', 'rgb')
        rgb_file = rgb_file.replace('json', 'png')
        rgb = np.array(imageio.imread(rgb_file))
        depth_file = label_file.replace('pose', 'depth')
        depth_file = depth_file.replace('json', 'png')
        depth = np.array(imageio.imread(depth_file))
        depth = (depth / 256) * 2
        vis_file = rgb_file.replace('rgb', 'pose_visualized')
        labels = json.load(open(label_file))
        pose = {}
        for key in labels['links']:
            #cur_pose[:3, 3] = labels['links'][key]['location']
            #cur_pose[:3, :3] = R.from_quat(labels['links'][key]['orientation']).as_matrix()
            pose[key] = np.array(labels['links'][key])#np.eye(4)
        color_data, depth_data, seg, ycb_6d = render_scene(pose)
        ycb_file = label_file.replace('pose', 'ycb_6d_pose').replace('json', 'pckl')
        seg_file = label_file.replace('pose', 'seg').replace('json', 'npy')
        with open(seg_file, 'wb') as f:
            np.save(f, seg)
        with open(ycb_file, 'wb') as f:
            pickle.dump(ycb_6d, f)
        rgb = cv2.imread(rgb_file)
        gray_image = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

        # Replicate the grayscale values to create a three-channel grayscale image
        three_channel_gray = cv2.merge([gray_image, gray_image, gray_image])
        #three_channel_gray = 0.4 * three_channel_gray + 0.6 * rgb
        three_channel_gray = 0.4 * rgb
        color_data[seg < 1] = three_channel_gray[seg<1]#rgb[seg < 1] * 0.5
        full = np.concatenate((rgb, color_data), axis=1)
        full = cv2.resize(full, (1280, 360))

        text = seq
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2

        # Get the size of the text
        text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)

        # Calculate the position to place the text
        text_x = 50
        text_y = full.shape[0] - 20  # Bottom left corner

        # Define the background box dimensions
        box_padding = 10
        box_x = text_x - box_padding
        box_y = text_y - text_size[1] - box_padding
        box_width = text_size[0] + 2 * box_padding
        box_height = text_size[1] + 2 * box_padding

        # Create a black background box
        black_box = np.zeros_like(full)
        #cv2.rectangle(black_box, (box_x, box_y), (box_x + box_width, box_y + box_height), (0, 0, 0), -1)

        #full[box_y:box_y + box_height, box_x:box_width + box_x, :] = 0

        # Put text on the black box
        #cv2.putText(full, text, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness)

        # Combine the original image and the image with text and black box
        #result = cv2.addWeighted(full, 1, black_box, 0.5, 0)

        cv2.imwrite(vis_file, full)
        

