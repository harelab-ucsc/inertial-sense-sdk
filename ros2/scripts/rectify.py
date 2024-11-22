#!/usr/bin/env python3

import rosbag
import time
import yaml
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import argparse

def rectify_image(raw_image, intrinsics, distortion_coeffs, resolution):
    # Convert lists to numpy arrays
    K = np.array([[intrinsics[0], 0, intrinsics[2]],
                  [0, intrinsics[1], intrinsics[3]],
                  [0, 0, 1]])
    D = np.array(distortion_coeffs)

    # Get image resolution
    width, height = resolution

    # Create rectification and projection maps
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, (width, height), cv2.CV_32FC1)

    # Convert raw image message to OpenCV image using rgb8 encoding
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding='mono8')

    # Rectify the image using the maps
    rectified_image = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Convert the rectified image back to ROS Image message using rgb8 encoding
    rectified_img_msg = bridge.cv2_to_imgmsg(rectified_image, encoding='mono8')
    rectified_img_msg.header = raw_image.header

    return rectified_img_msg

def process_bag(input_bag_path, output_bag_path, raw_image_topic, rectified_image_topic, calibration_yaml_path):
    # Load calibration parameters from YAML file
    with open(calibration_yaml_path, 'r') as file:
        calibration_data = yaml.safe_load(file)

    # Extract calibration parameters for cam0
    cam0 = calibration_data['cam0']
    intrinsics = cam0['intrinsics']
    distortion_coeffs = cam0['distortion_coeffs']
    resolution = cam0['resolution']

    # Create a bridge for converting ROS messages to OpenCV images
    bridge = CvBridge()

    # Open input and output bag files
    num_images = 0
    total_latency = 0
    with rosbag.Bag(input_bag_path, 'r') as in_bag, rosbag.Bag(output_bag_path, 'w') as out_bag:
        # Iterate over messages in the input bag
        for topic, msg, t in in_bag.read_messages():
            if topic == raw_image_topic:
                # If the message is an image, rectify it
                t_prev = time.time_ns()
                rectified_img_msg = rectify_image(msg, intrinsics, distortion_coeffs, resolution)
                total_latency += (time.time_ns() - t_prev)
                num_images += 1
                # Write the rectified image message to the output bag under the rectified image topic
                out_bag.write(rectified_image_topic, rectified_img_msg, t)
            else:
                # Otherwise, copy the message as it is
                out_bag.write(topic, msg, t)

    mean_latency = total_latency / num_images
    print(mean_latency/1e9)

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Rectify raw images in a ROS bag.')
    parser.add_argument('input_bag', type=str, help='Path to input ROS bag file')
    parser.add_argument('output_bag', type=str, help='Path to output ROS bag file')
    parser.add_argument('raw_image_topic', type=str, help='Topic for raw images in input bag')
    parser.add_argument('rectified_image_topic', type=str, help='Topic for rectified images in output bag')
    parser.add_argument('calibration_yaml', type=str, help='Path to calibration YAML file')

    args = parser.parse_args()

    # Call the function to process the bag file with the provided arguments
    process_bag(args.input_bag, args.output_bag, args.raw_image_topic, args.rectified_image_topic, args.calibration_yaml)

if __name__ == '__main__':
    main()

