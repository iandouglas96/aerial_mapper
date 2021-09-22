#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation
import cv2
import rospy
import message_filters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DatasetGen:
    def __init__(self):
        self.last_pos = np.array([0.0, 0.0, 0.0]) 
        self.cur_ind = 0
        self.root_path = '/media/ian/SSD1/tmp_datasets/eth_aerial_mapper/unity_test'
        self.pose_file = open(self.root_path + '/opt_poses.txt', 'w')

        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber('/quadrotor/RGBCamera_Down/image_raw', Image)
        self.pose_sub = message_filters.Subscriber('/unity_ros/quadrotor/TrueState/pose', PoseStamped)
        self.sync = message_filters.TimeSynchronizer([self.image_sub, self.pose_sub], queue_size=10)
        self.sync.registerCallback(self.data_cb)

    def data_cb(self, img, pose):
        pos = np.array([pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z])
        r = Rotation.from_quat([pose.pose.orientation.x,
                                pose.pose.orientation.y,
                                pose.pose.orientation.z,
                                pose.pose.orientation.w])
        print('----------------')
        print(r.as_quat()) #xyzw
        transform = Rotation.from_dcm([[0., -1., 0.],
                                       [-1., 0., 0.],
                                       [0., 0., -1.]])
        print(transform.as_quat())
        r_rot = r * transform #to camera frame
        print(r_rot.as_quat())
        final_quat = r_rot.as_quat()

        dist = np.linalg.norm(pos - self.last_pos)
        if dist > 0.5:
            self.last_pos = pos

            #write out img
            cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
            cv2.imwrite(self.root_path + '/image_' + str(self.cur_ind) + '.jpg', cv_img)

            #write out pose
            self.pose_file.write(str(pose.pose.position.x) + ' ' +
                                 str(pose.pose.position.y) + ' ' +
                                 str(pose.pose.position.z) + ' ' +
                                 str(final_quat[3]) + ' ' +
                                 str(final_quat[0]) + ' ' +
                                 str(final_quat[1]) + ' ' +
                                 str(final_quat[2]) + '\r\n')

            self.cur_ind += 1

if __name__ == '__main__':
    rospy.init_node('dataset_gen')
    dg = DatasetGen()
    rospy.spin()

