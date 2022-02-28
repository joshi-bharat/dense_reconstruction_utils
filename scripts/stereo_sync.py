#!/usr/bin/env python

import os
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from message_filters import ApproximateTimeSynchronizer, Subscriber


class StereoSync:

    def __init__(self, root_folder, scale=1.0):

        self.scale = scale
        self.bridge = CvBridge()

        left_image_sub = Subscriber('/camera0', Image)
        right_image_sub = Subscriber('/camera1', Image)

        ats = ApproximateTimeSynchronizer(
            [left_image_sub, right_image_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.stereo_image_callback)

        if not os.path.exists(root_folder):
            os.makedirs(root_folder)
        img_folder = os.path.join(root_folder, 'left')
        if not os.path.exists(img_folder):
            os.mkdir(img_folder)
        self.left_img_data_folder = os.path.join(img_folder, 'data')
        if not os.path.exists(self.left_img_data_folder):
            os.mkdir(self.left_img_data_folder)
        file = os.path.join(img_folder, 'data.csv')
        self.left_img_file = open(file, 'w')
        self.left_img_file.write('#timestamp [ns],filename\n')

        img_folder = os.path.join(root_folder, 'right')
        if not os.path.exists(img_folder):
            os.mkdir(img_folder)
        self.right_img_data_folder = os.path.join(img_folder, 'data')
        if not os.path.exists(self.right_img_data_folder):
            os.mkdir(self.right_img_data_folder)
        file = os.path.join(img_folder, 'data.csv')
        self.right_img_file = open(file, 'w')
        self.right_img_file.write('#timestamp [ns],filename\n')

        self.last_timestamp = -1

    def __del__(self):
        self.left_img_file.close()
        self.right_img_file.close()

    def stereo_image_callback(self, left_img_msg, right_img_msg):
        rospy.logwarn_once(" !!! Inside Stereo Callback !!!")
        left_timestamp = left_img_msg.header.stamp.to_nsec()
        right_timestamp = right_img_msg.header.stamp.to_nsec()
        h = left_img_msg.height
        w = left_img_msg.width

        new_h = int(h * self.scale)
        new_w = int(w * self.scale)
        size = (new_w, new_h)

        if (left_timestamp > self.last_timestamp and right_timestamp > self.last_timestamp):

            stamp = max(left_timestamp, right_timestamp)

            left_image = self.bridge.imgmsg_to_cv2(
                left_img_msg, desired_encoding='bgr8')
            if (self.scale != 1):
                left_image = cv2.resize(left_image, size)
            cv2.imwrite(os.path.join(self.left_img_data_folder,
                                     '{}.png'.format(str(stamp))), left_image)
            self.left_img_file.write('{},{}\n'.format(
                str(stamp), '{}.png'.format(str(stamp))))

            right_image = self.bridge.imgmsg_to_cv2(
                right_img_msg, desired_encoding='bgr8')

            if (self.scale != 1):
                right_image = cv2.resize(right_image, size)
            cv2.imwrite(os.path.join(self.right_img_data_folder,
                                     '{}.png'.format(str(stamp))), right_image)
            self.right_img_file.write('{},{}\n'.format(
                str(stamp), '{}.png'.format(str(stamp))))

            self.last_timestamp = stamp

        else:
            rospy.logwarn(
                'Left Stamp: {} or Right Stamp: {} not greater than the previous stamp: {}'.format(
                    left_timestamp, right_timestamp, self.last_timestamp))


if __name__ == '__main__':

    rospy.init_node('stereo_sync', anonymous=True)

    # all_params = rospy.get_param_names()
    # for param in all_params:
    #     rospy.logwarn(param)

    if (not rospy.has_param('~dataset_path')):
        rospy.logfatal("Require the dataset path of root directory")

    scale = 1.0
    if (rospy.has_param('~scale')):
        scale = rospy.get_param('~scale')

    dataset_folder = rospy.get_param('~dataset_path')

    asl_converter = StereoSync(dataset_folder, scale)

    while not rospy.is_shutdown():
        rospy.spin()
