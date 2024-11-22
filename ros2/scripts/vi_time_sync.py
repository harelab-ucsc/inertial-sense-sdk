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


class BagProcessor:
    def __init__(self, input_bag_path, output_bag_path, image_topic, ins_topic):
        self.input_bag_path = input_bag_path
        self.output_bag_path = output_bag_path
        self.image_topic = image_topic
        self.ins_topic = ins_topic
        self.deltas = []
        self.br = CvBridge()

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
                if closest_image:
                    # compute the time difference
                    old_time = closest_image.header.stamp.sec + closest_image.header.stamp.nanosec * 1e-9
                    new_time = ins_timestamp.sec + ins_timestamp.nanosec * 1e-9
                    delta = old_time - new_time
                    self.deltas.append(delta)

                    # adjust timestamps with strobe-triggered INS2 msgs and write to bag
                    new_image = self.update_image_timestamp(closest_image, ins_timestamp)
                    new_image = serialize_message(new_image)
                    print(f"Writing topic {self.image_topic} with timestamp {ins_timestamp}")
                    writer.write(self.image_topic, new_image, ins_timestamp_int)

                    # save images as *.png's
                    image = self.br.imgmsg_to_cv2(closest_image, desired_encoding='passthrough')
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    cv2.imwrite(f'frame_{ins_timestamp.sec}.{str(ins_timestamp.nanosec).rjust(9, "0")}.png', image)

                    # waste management
                    closest_image = None

        deltas = np.array(self.deltas)
        mean = np.mean(deltas)
        std = np.std(deltas)
        plt.hist(deltas, bins=50)
        plt.savefig("hist.png")
        print(f'time correction mean: {mean} sec, std: {std} sec, {deltas.shape} samples')
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
