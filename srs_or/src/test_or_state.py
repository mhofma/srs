#!/usr/bin/python

import roslib; roslib.load_manifest('srs_or')
import rospy
import smach
import smach_ros
import sys
import rosbag

from geometry_msgs.msg import Pose

#import unittest

from or_state import *

class TestStates:
	def __init__(self, *args):
		rospy.init_node('test_states')

	
	def test_object_detection(self):
		#bag = rosbag.Bag('/home/mhofma/kinect.bag')
		#lastMsg = 0
		#for topic, msg, t in bag.read_messages(topics=['/camera/depth/points']):
	#		lastMsg = msg
		#bag.close()
		
		
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		SM.userdata.object_id = 3
		#SM.userdata.target_object_pointcloud = lastMsg
	
		# open the container
		with SM:
			smach.StateMachine.add('DETECT', DetectObject(),
			transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed', 'not_completed':'overall_failed', 'preempted':'overall_failed'})
		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			#self.fail(error_message)

# main
if __name__ == '__main__':
	test = TestStates()
	test.test_object_detection()
