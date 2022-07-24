#! /usr/bin/env python

from __future__ import print_function
import os
import cv2
from cv_bridge import CvBridge

import rospy
import rosbag
import message_filters
from tqdm import tqdm

from sensor_msgs.msg import Image, CompressedImage


class ImageExtractor():
    def __init__(self,
                 image_dir,
                 bag_file,
                 stereo,
                 compressed,
                 left_topic,
                 right_topic=None):

        self.stereo = stereo
        self.bag = rosbag.Bag(bag_file, 'r')
        self.img_dir = image_dir
        if stereo:
            self.left_image_dir = os.path.join(self.img_dir, 'left')
            self.right_image_dir = os.path.join(self.img_dir, 'right')
        else:
            self.left_image_dir = os.path.join(self.img_dir, 'images')

        if not os.path.exists(image_dir):
            os.makedirs(image_dir)
        if self.stereo:
            if compressed:
                self.left_topic = left_topic + '/compressed'
                self.right_topic = right_topic + '/compressed'
            else:
                self.right_topic = right_topic
                self.left_topic = left_topic

            if not os.path.exists(self.left_image_dir):
                os.mkdir(self.left_image_dir)
            if not os.path.exists(self.right_image_dir):
                os.mkdir(self.right_image_dir)

        else:
            if compressed:
                self.left_topic = left_topic + '/compressed'
            else:
                self.left_topic = left_topic

        self.cv_bridge = CvBridge()

    def __del__(self):
        self.bag.close()

    def save_image(self, img_msg, base_dir: str):
        stamp = str(img_msg.header.stamp.to_sec())
        filename = os.path.join(base_dir, stamp + '.png')
        if img_msg._type == "sensor_msgs/CompressedImage":
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(img_msg)
        else:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        cv2.imwrite(filename, cv_image)

    def extract_images(self):

        rospy.Time
        image_list = open(os.path.join(self.img_dir, 'images.txt'), 'w')
        image_list.write('#timestamp, filename')
        for topic, msg, t in tqdm(self.bag.read_messages(),
                                  total=self.bag.get_message_count()):
            if topic == self.left_topic:
                stamp = str(msg.header.stamp.to_sec())
                image_list.write('\n' + str(msg.header.stamp.sec) + '.' +
                                 str(msg.header.stamp.nsec) + ',' + stamp + '.png')
                self.save_image(msg, self.left_image_dir)
            elif self.stereo and topic == self.right_topic:
                self.save_image(msg, self.right_image_dir)


if __name__ == "__main__":

    rospy.init_node('extract_images', anonymous=True)

    if (not rospy.has_param('~image_dir')):
        rospy.logfatal("Require the dataset path of asl directory")

    if (not rospy.has_param('~bag')):
        rospy.logfatal("Require the bag file to extract images")

    scale = 1.0
    if (rospy.has_param('~scale')):
        scale = rospy.get_param('~scale')

    image_folder = rospy.get_param('~image_dir')
    bag_file = rospy.get_param('~bag')

    stereo = False
    if (rospy.has_param('~stereo')):
        stereo = rospy.get_param('~stereo')

    left = "/left/image_raw"
    if (rospy.has_param('~left')):
        left = rospy.get_param('~left')

    right = "/right/image_raw"
    if (rospy.has_param('~right')):
        right = rospy.get_param('~right')

    compressed = False
    if (rospy.has_param('~compressed')):
        compressed = rospy.get_param('~compressed')

    rospy.loginfo("bag: {}".format(bag_file))
    rospy.loginfo("image dir: {}".format(image_folder))
    rospy.loginfo("stereo: {}".format(stereo))
    rospy.loginfo("compressed: {}".format(compressed))
    rospy.loginfo("left image topic: {}".format(left))
    rospy.loginfo("right image topic: {}".format(right))

    extractor = ImageExtractor(image_dir=image_folder,
                               bag_file=bag_file,
                               stereo=stereo,
                               left_topic=left,
                               right_topic=right,
                               compressed=compressed)
    extractor.extract_images()

    # while not rospy.is_shutdown():
    #     rospy.spin()
