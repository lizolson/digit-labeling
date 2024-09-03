import pdb
import os
import json
import rosbag
import argparse
import os
import sys

processed_data_root = "/home/liz/Desktop/research/icra24/data/processed/"


# Function to convert digit log file from ros bag to json
def convert_rosbag2json(input_bag_file, output_folder):
    bag = rosbag.Bag(input_bag_file)

    # Iterate over all logger messages in the bag file 
    for topic, msg, t in bag.read_messages(topics=['/digit_observation']):
        output_json_file = os.path.join(output_folder, str(t.to_nsec()) + '.json')
        msg_time = int(str(msg.header.stamp))
        data = {}
        msgdata = msg.q_all
        msglist = [x for x in msgdata]
        data["q_all"] =  msglist
        with open(output_json_file, 'w') as f:
            json.dump(data, f)

    bag.close()
    




if __name__=="__main__":
    base = processed_data_root + sys.argv[1]
    for bagfn in os.listdir(base):
        bagfile = os.path.join(base, bagfn)
        bagfile = bagfile.replace('processed', 'bags') + '.bag'
        print(bagfile)
        obj = sys.argv[1] 
        output_folder = os.path.join(processed_data_root, obj, bagfn, "joints")
        if not os.path.exists(output_folder):
            os.mkdir(output_folder)
        convert_rosbag2json(bagfile, output_folder)




