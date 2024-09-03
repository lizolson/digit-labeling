# Render Labels

TODO: Instructions for installing dependencies for Blender's Python Interpreter

```
cd rendering
/path/to/blender/python/ render_vis_fast.py ../../data/release/ladder-moving/ 5
```


# Processing Bag Files

### Extracting RGBD Images

In one terminal: 

```
roscore
```

In another terminal: 

```
cd processing/
python realsense_rgbd_saver_node.py video_name
```

In a third terminal, play the rosbag you wish to extract images from: 

```
rosbag play bag_filename.bag
```


### Extracting Joint Values

```
python convert_rosbag2json.py sequence_name
```

### Syncing Data

##### Syncing RGB + Depth

```
python align_rgb_depth_timestamp.py 
```

##### Syncing RGB + Joint Values
```
python align_rgb_joints_timestamp.py
```

### Creating Point Clouds

```
python rgbd2pointcloud.py /path/to/data/
```

### Organizing Data

```
python dataset_creator.py 
```
