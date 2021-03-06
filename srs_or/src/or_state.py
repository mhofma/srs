#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: srs
# \note
#   ROS stack name: srs
# \note
#   ROS package name: srs_object_verification
#
# \author
#   Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
#
# \date Date of creation: Jan 2012
#
# \brief
#   Implements generic states which can be used in multiple scenarios.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib
roslib.load_manifest('srs_or')
import rospy
import smach
import smach_ros
import actionlib
import operator
from threading import Semaphore, Condition
from geometry_msgs.msg import Pose, PoseStamped, Point, Point32, PointStamped, Vector3, Vector3Stamped, Quaternion, QuaternionStamped
import tf.transformations

from sensor_msgs.msg import *
from srs_or.srv import *


## Initialize state
#
# This state will initialize all hardware drivers.
class DetectObject(smach.State):
	def kinect_callback(self, data):
		self.pointcloud = data
		self.kinect_sub.unregister()
		self.kinect_cond.acquire()
		self.newPointcloud = True
		self.kinect_cond.notify()
		self.kinect_cond.release()
	
	def __init__(self):
		self.kinect_cond = Condition()
		smach.State.__init__(self,
		outcomes=['succeeded', 'failed', 'not_completed', 'preempted'],
		input_keys=['object_id'],
		output_keys=['target_object_pose'])
		self.or_client =  rospy.ServiceProxy('/or', or_request)
		self.or_publ = rospy.Publisher('/prof_pose', PoseStamped)
		self.newPointcloud = False
	
	##convert a 4x4 matrix stored as vectore to a Pose message 
	def mat_to_pose(self,vecMat):
		pose = PoseStamped()
		print vecMat
		mat = [list(vecMat[0:4]),list(vecMat[4:8]),list(vecMat[8:12]),list(vecMat[12:16])]
		print mat
		pose.pose.position.x = vecMat[3]/1000.0
		pose.pose.position.y = vecMat[7]/1000.0
		pose.pose.position.z = vecMat[11]/1000.0
		quat = tf.transformations.quaternion_from_matrix(mat)
		pose.pose.orientation.x = quat[0]
		pose.pose.orientation.y = quat[1]
		pose.pose.orientation.z = quat[2]
		pose.pose.orientation.w = quat[3]
		return pose
	
	def execute(self, userdata):
		self.kinect_cond.acquire()
		
		self.newPointcloud = False
		max_cnt = 3
		cnt = 0
		while self.newPointcloud == False and cnt < max_cnt:
			#self.kinect_sub = rospy.Subscriber("/cam3d/depth/points", PointCloud2, self.kinect_callback)
			self.kinect_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.kinect_callback)
			self.kinect_cond.wait(10.0)
			if self.newPointcloud == False:
				self.kinect_sub.unregister()
				cnt = cnt + 1
				print 'No Kinect available'
		
		self.kinect_cond.release()
		
		if cnt >= max_cnt:
			return 'failed'
		
		print userdata.object_id
		print self.pointcloud.header
		retVal = self.or_client(self.pointcloud, userdata.object_id)
		#retVal = self.or_client(8, userdata.object_id)
		#sensor_msgs/PointCloud2
		print '4x4 Matrix values:'
			
		pose = self.mat_to_pose(retVal.matrix)
		pose.header = self.pointcloud.header
		userdata.target_object_pose = pose
		self.or_publ.publish(pose)
		return 'succeeded'



