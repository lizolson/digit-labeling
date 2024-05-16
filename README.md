# digit-labeling

extracting RGBD images
cd processing/rgbd_image_saver-main/
roscore
python2 realsense_rgbd_saver_node.py video_name
rosbag play bag_filename.bag

python2 convert_rosbag2json.py poncho

python2 align_rgb_depth_timestamp.py 

python2 align_rgb_joints_timestamp.py

python rgbd2pointcloud.py /home/liz/Desktop/research/icra24/data/processed/

python2 dataset_creator.py 
