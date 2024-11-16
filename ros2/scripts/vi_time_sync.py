#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from inertial_sense_ros2.msg import DIDINS2
import argparse
import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

class BagProcessor:
    def __init__(self, input_bag_path, output_bag_path, image_topic, ins_topic):
        self.input_bag_path = input_bag_path
        self.output_bag_path = output_bag_path
        self.image_topic = image_topic
        self.ins_topic = ins_topic
        self.deltas = []

    def process_bag(self):
        # Initialize reader and writer
        reader = SequentialReader()
        storage_options = StorageOptions(uri=self.input_bag_path, storage_id="mcap")
        converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        reader.open(storage_options, converter_options)
        topics_and_types = reader.get_all_topics_and_types()

        writer = SequentialWriter()
        writer.open(StorageOptions(uri=self.output_bag_path, storage_id="mcap"), converter_options)

        topic_type_map = {t.name: t.type for t in topics_and_types}
        # Register all topics with the writer
        for topic in topics_and_types:
            writer.create_topic(topic)

        image_msgs = []
        ins_msgs = []

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

        # Process INS messages and adjust image timestamps
        for ins_msg in ins_msgs:
            if ins_msg.hdw_status & 2:
                ins_timestamp = ins_msg.header.stamp
                closest_image = self.find_closest_image(ins_timestamp, image_msgs)
                if closest_image:
                    old_time = closest_image.header.stamp.sec + closest_image.header.stamp.nanosec * 1e-9
                    new_time = ins_timestamp.sec + ins_timestamp.nanosec * 1e-9
                    delta = old_time - new_time
                    self.deltas.append(delta)
                    new_image = self.update_image_timestamp(closest_image, ins_timestamp)
                    writer.write(self.image_topic, new_image, ins_timestamp)

        mean = np.mean(np.array(deltas))
        std = np.std(np.array(deltas))
        print('time correction mean:' + mean + 'seconds' + 'std:' + std)
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
                closest_image = image
                min_diff = diff
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


def main():
    parser = argparse.ArgumentParser(description="Fix image timestamps in a ROS2 bag file using INS messages.")
    parser.add_argument("input_bag", help="Path to the input ROS2 bag file")
    parser.add_argument("output_bag", help="Path to the output ROS2 bag file")
    parser.add_argument("image_topic", help="Image topic name (e.g., /camera/image_raw)")
    parser.add_argument("ins_topic", help="INS topic name (e.g., /ins/data)")

    args = parser.parse_args()

    rclpy.init()

    processor = BagProcessor(args.input_bag, args.output_bag, args.image_topic, args.ins_topic)
    processor.process_bag()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
