#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import message_filters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DatasetGen:
    def __init__(self):
        self.last_odom = np.array([0.0, 0.0, 0.0]) 
        self.cur_ind = 0
        self.root_path = '/media/ian/SSD1/tmp_datasets/eth_aerial_mapper/unity_test'
        self.pose_file = open(self.root_path + '/opt_poses.txt', 'w')

        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber('/quadrotor/RGBCamera_Down/image_raw', Image)
        self.pose_sub = message_filters.Subscriber('/orb_slam3/pose', PoseStamped)
        self.sync = message_filters.TimeSynchronizer([self.image_sub, self.pose_sub], 10)
        self.sync.registerCallback(self.data_cb)

    def data_cb(self, img, pose):
        pos = np.array([pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z])
        dist = np.linalg.norm(pos - self.last_odom)
        if dist > 0.5:
            self.last_odom = pos

            #write out img
            cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
            cv2.imwrite(self.root_path + '/image_' + str(self.cur_ind) + '.jpg', cv_img)

            #write out pose
            self.pose_file.write(str(pose.pose.position.x) + ' ' +
                                 str(pose.pose.position.y) + ' ' +
                                 str(pose.pose.position.z) + ' ' +
                                 str(pose.pose.orientation.w) + ' ' +
                                 str(pose.pose.orientation.x) + ' ' +
                                 str(pose.pose.orientation.y) + ' ' +
                                 str(pose.pose.orientation.z) + '\r\n')

            self.cur_ind += 1

if __name__ == '__main__':
    rospy.init_node('dataset_gen')
    dg = DatasetGen()
    rospy.spin()

