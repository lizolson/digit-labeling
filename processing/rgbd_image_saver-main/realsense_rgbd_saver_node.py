import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import numpy as np
import os
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats






def createdolder(folder):
    if os.path.isdir(folder):
        pass
    else:
        os.mkdir(folder)
        # print("Successful create folder: " + folder)
    
cam1_folder = '/home/liz/Desktop/research/icra24/data/processed/poncho/'
cam1_rgb_channel = '/rgb/image_raw'
cam1_depth_channel = '/depth_to_rgb/image_raw'

#cam2_folder = '/media/huijie/photos/datasets_bop/YCB_Progresslabeler/different_sensor/realsenseL515/'
#cam2_rgb_channel = '/cam_2/color/image_raw'
#cam2_depth_channel = '/cam_2/aligned_depth_to_color/image_raw'

#scene_name = "test_scene5"
scene_name = sys.argv[1]

cam1_scene_folder = os.path.join(cam1_folder, scene_name)
print(cam1_scene_folder)
#cam2_scene_folder = os.path.join(cam2_folder, scene_name)

createdolder(cam1_scene_folder)
createdolder(os.path.join(cam1_scene_folder, "rgb"))
createdolder(os.path.join(cam1_scene_folder, "depth"))
#createdolder(cam2_scene_folder)
#createdolder(os.path.join(cam2_scene_folder, "rgb"))
#createdolder(os.path.join(cam2_scene_folder, "depth"))

bridge = CvBridge()
seq = 0
def cam1_ImageSaveCallback(data):
    # print(data.header)
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="bgr8")
        timestamp = data.header.stamp
    except CvBridgeError as e:
        print(e)
        return
        
    cv2.imwrite(os.path.join(cam1_scene_folder, 'rgb', str(timestamp) + '.png') , np.asarray(cv_image))
    #print(os.path.join(cam1_scene_folder, 'rgb', str(timestamp) + '.png'))
    # print('rgb saved ', timestamp)

def cam1_DepthSaveCallback(data):
    #print(str(data))
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="passthrough")
        timestamp = data.header.stamp
    except CvBridgeError as e:
        # print(e)
        return
    # print('depth saved ', timestamp)
    global seq
    print('img {} saved'.format(seq))
    seq += 1
    cv2.imwrite(os.path.join(cam1_scene_folder, 'depth', str(timestamp) + '.png'), np.asarray(cv_image))


if __name__ == '__main__':
    rospy.init_node('rgbd_saver_node')
    rospy.Subscriber(cam1_rgb_channel, Image, cam1_ImageSaveCallback)
    rospy.Subscriber(cam1_depth_channel, Image, cam1_DepthSaveCallback)
    rospy.spin()
