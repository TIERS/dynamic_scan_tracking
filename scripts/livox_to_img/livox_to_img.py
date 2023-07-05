#!/usr/bin/env python

##############################################################################
#
#  @file livox_to_img.py
#  @brief Executable ROS node for livox point cloud to image conversion
#
#  Copyright (C) 2023-07-04 18:06:54.
#  Author: Ha Sier
#  Email: sierha@utu.fi
#  All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
##############################################################################

import rospy
import os

import cv2
import numpy as np

import yolov5

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge

from livox_to_img.livox_functions import *


####################################################################################
#If YOLO does not work well, You can change conf and IoU value 
        # self.model.conf = 0.6
        # self.model.iou = 0.6

# Outdoors : conf = 0.8 ; iou =0.8

# Indoors  : conf = 0.6 ; iou =0.6
####################################################################################



class LidarToImageNode:

    def __init__(self):
        rospy.init_node('lidar_to_image_node')
        self.bridge = CvBridge()
        self.lidar_sub = rospy.Subscriber('/livox/points', PointCloud2, self.point_cloud_callback)
        self.image_pub = rospy.Publisher('/lidar_image', Image, queue_size=1)
        self.init_pub = rospy.Publisher('/init_position', PoseStamped, queue_size=1)


        self.buffer_size = 30
        self.point_cloud_buffer = [None] * self.buffer_size
        self.current_index = 0
        self.counter = 0
        self.model = yolov5.load(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))) + "/model/model.pt")
        self.model.conf = 0.8
        self.model.iou = 0.8
        self.model.agnostic = False
        self.model.multi_label = False
        self.model.max_det = 1000
        self.goalMsg = PoseStamped()

    def point_cloud_callback(self, msg):
        # Extract point cloud data from the message
        self.point_cloud_buffer[self.current_index] = msg
        self.current_index = (self.current_index + 1) % self.buffer_size
        
        if None not in self.point_cloud_buffer:
            combined_cloud_points = []
            # colors = []
            for cloud_msg in self.point_cloud_buffer:
                pc = np.array(list(pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)))
                pc = np.array(list(pc))
                combined_cloud_points.append(pc)

            combined_cloud = np.vstack(combined_cloud_points)
            # print(combined_cloud_points.shape)
            image,valid_img_coords, valid_points = point_cloud_to_image(combined_cloud, 81.7, 25.1, 1024,275,40)
            result = self.model(image)
            predictions_resize = result.pred[0]
            boxes_re = predictions_resize[:, :4] 
            boxes_re = boxes_re.tolist()
            scores_re = predictions_resize[:, 4]
            categories_re = predictions_resize[:, 5].tolist()
            
            if (0.0 in categories_re):
                index = get_index(categories_re)[0]
                x1,y1,x2,y2 = get_box(boxes_re,index)
                print('-------------------------------------------------------------------------------------')
                cv2.rectangle(image, (x1,y2),(x2,y1), (0, 255, 0), 2, 4)
                x1,y1 = get_ori_box(x1,y1)
                x2,y2 = get_ori_box(x2,y2)

                filtered_points = filter_points_by_rectangular_area(valid_img_coords, valid_points, x1, y2, x2, y1)
                filtered_pcd = point_cloud_to_pcd(filtered_points[:, :3])
                init_position = get_init_position(filtered_pcd,0.2,50)
                self.goalMsg.header.stamp = rospy.Time.now()                       
                self.goalMsg.pose.position.z = init_position[2]
                self.goalMsg.pose.position.x = init_position[0]
                self.goalMsg.pose.position.y = init_position[1]
                self.goalMsg.pose.orientation.w = 1.0
                self.init_pub.publish(self.goalMsg)
        
                self.counter = self.counter + 1
                print("image NO.",self.counter)
            # Publish the image
            image_msg = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
            self.image_pub.publish(image_msg)
    

def main():
    try:
        LidarToImageNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass