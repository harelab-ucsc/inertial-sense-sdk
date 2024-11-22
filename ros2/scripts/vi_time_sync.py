#!/usr/bin/env python3
import pdb
import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from inertial_sense_ros2.msg import DIDINS2
import argparse
import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message, serialize_message

import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import cv2
import os
import json
import yaml
from pyproj import Proj, Transformer

#from rectify import rectify_image

class BagProcessor:
    def __init__(self, input_bag_path, output_bag_path, image_topic, ins_topic, intrinsics_path):
        self.input_bag_path = input_bag_path
        self.output_bag_path = output_bag_path
        self.image_topic = image_topic
        self.ins_topic = ins_topic
        self.deltas = []
        self.br = CvBridge()
        self.frames = []
        self.intrinsics = self.load_intrinsics(intrinsics_path)

        # Initialize UTM transformer
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32610", always_xy=True)  # Replace EPSG:32633 with appropriate UTM zone

    def load_intrinsics(self, intrinsics_path):
        """Load camera intrinsics from a YAML file."""
        with open(intrinsics_path, "r") as file:
            return yaml.safe_load(file)

    def process_bag(self):
        # Initialize reader and writer
        reader = SequentialReader()
        storage_options = StorageOptions(uri=self.input_bag_path, storage_id="mcap")
        converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        reader.open(storage_options, converter_options)
        topics_and_types = reader.get_all_topics_and_types()

        writer = SequentialWriter()
        writer.open(StorageOptions(uri=self.output_bag_path, storage_id="mcap"), converter_options)

        topic_type_map = {t.name:t.type for t in topics_and_types}
        # Register all topics with the writer
        for topic in topics_and_types:
            writer.create_topic(topic)

        image_msgs = []
        ins_msgs = []

        # Ensure output directories exist
        os.makedirs("images", exist_ok=True)

        print('reading bag')
        # Read and process messages
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            message_type = get_message(topic_type_map[topic])
            msg = deserialize_message(data, message_type)

            if topic == self.image_topic:
                image_msgs.append(msg)
            elif topic == self.ins_topic:
                ins_msgs.append(msg)
                writer.write(topic, data, timestamp)
            else:
                # Copy all other topics as-is
                writer.write(topic, data, timestamp)
        print(f'image_msgs length: {len(image_msgs)}')
        print(f'ins_msgs length: {len(ins_msgs)} \n')
        print('bag read done \n')

        HDW_STATUS_STROBE_IN_EVENT = 0x00000020

        # Process INS messages and adjust image timestamps
        for ins_msg in ins_msgs:
            if ins_msg.hdw_status & HDW_STATUS_STROBE_IN_EVENT == HDW_STATUS_STROBE_IN_EVENT:
                # print('found strobe triggered INS2')
                ins_timestamp = ins_msg.header.stamp
                ins_timestamp_int = int(ins_timestamp.sec * 1e9 + ins_timestamp.nanosec)
                closest_image = self.find_closest_image(ins_timestamp, image_msgs)
                ins_timestamp_int = int(ins_timestamp.sec * 1e9 + ins_timestamp.nanosec)
                if closest_image:
                    # Compute the time difference
                    old_time = closest_image.header.stamp.sec + closest_image.header.stamp.nanosec * 1e-9
                    new_time = ins_timestamp.sec + ins_timestamp.nanosec * 1e-9
                    delta = old_time - new_time
                    self.deltas.append(delta)

                    # Update image timestamp and save the image
                    updated_image = self.update_image_timestamp(closest_image, ins_timestamp)
                    timestamp_str = f"{ins_timestamp.sec}.{ins_timestamp.nanosec:09d}"
                    self.save_image(updated_image, timestamp_str)

                    # Append pose to JSON
                    self.append_pose_to_json(ins_msg, updated_image, timestamp_str)

                    new_image = serialize_message(updated_image)
                    writer.write(self.image_topic, new_image, ins_timestamp_int)

        mean = np.mean(np.array(self.deltas))
        std = np.std(np.array(self.deltas))
        print(f'time correction mean: {mean} sec, std: {std} sec')

        # Save JSON file
        self.save_json()

        # Close the bag writer
        writer.close()

    def find_closest_image(self, target_timestamp, image_msgs):
        """Find the closest image message to the given timestamp."""
        closest_image = None
        min_diff = float("inf")
        for image in image_msgs:
            old_time = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9
            new_time = target_timestamp.sec + target_timestamp.nanosec * 1e-9
            diff = abs(old_time - new_time)
            if diff < min_diff:
                # print('found closer image timestamp')
                closest_image = image
                min_diff = diff
        print(f'    found image matching timestamp: {target_timestamp}')
        return closest_image

    def update_image_timestamp(self, image_msg, new_timestamp):
        """Create a new Image message with an updated timestamp."""
        new_image = Image()
        new_image.header = image_msg.header
        new_image.header.stamp = new_timestamp
        new_image.data = image_msg.data
        new_image.height = image_msg.height
        new_image.width = image_msg.width
        new_image.encoding = image_msg.encoding
        new_image.is_bigendian = image_msg.is_bigendian
        new_image.step = image_msg.step
        return new_image

    def save_image(self, image_msg, timestamp_str):
        """Save the image message as a PNG file."""
        img_data = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        cv2.imwrite(f"images/{timestamp_str}.png", img_data)

    def append_pose_to_json(self, ins_msg, image_msg, timestamp_str):
        """Append the pose data from INS message to the JSON."""
        # Convert LLA to UTM
        utm_x, utm_y = self.transformer.transform(ins_msg.lla[1], ins_msg.lla[0])  # (longitude, latitude)
        altitude = ins_msg.lla[2]

        transform_matrix = [
            [1, 0, 0, utm_x],
            [0, 1, 0, utm_y],
            [0, 0, 1, altitude],
            [0, 0, 0, 1]
        ]

        pose = {
            "w": image_msg.width,
            "h": image_msg.height,
            "fl_x": self.intrinsics["fx"],
            "fl_y": self.intrinsics["fy"],
            "cx": self.intrinsics["cx"],
            "cy": self.intrinsics["cy"],
            "timestamp": ins_msg.header.stamp.sec + ins_msg.header.stamp.nanosec * 1e-9,
            "file_path": f"images/{timestamp_str}.png",
            "transform_matrix": transform_matrix
        }
        self.frames.append(pose)

    def save_json(self):
        """Save all frames to a JSON file."""
        with open("poses.json", "w") as json_file:
            json.dump({"frames": self.frames}, json_file, indent=4)

    def rectify_image(self, raw_image, intrinsics, distortion_coeffs, resolution):
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
        cv_image = self.br.imgmsg_to_cv2(raw_image, desired_encoding='mono8')

        # Rectify the image using the maps
        rectified_image = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR)

        # Convert the rectified image back to ROS Image message using rgb8 encoding
        rectified_img_msg = self.br.cv2_to_imgmsg(rectified_image, encoding='mono8')
        rectified_img_msg.header = raw_image.header

        return rectified_img_msg

def main():
    parser = argparse.ArgumentParser(description="Fix image timestamps in a ROS2 bag file using INS messages.")
    parser.add_argument("input_bag", help="Path to the input ROS2 bag file")
    parser.add_argument("output_bag", help="Path to the output ROS2 bag file")
    parser.add_argument("image_topic", help="Image topic name (e.g., /camera/image_raw)")
    parser.add_argument("ins_topic", help="INS topic name (e.g., /ins/data)")
    parser.add_argument("intrinsics", help="Path to the YAML file with camera intrinsics")

    args = parser.parse_args()

    rclpy.init()

    processor = BagProcessor(args.input_bag, args.output_bag, args.image_topic, args.ins_topic, args.intrinsics)
    processor.process_bag()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
