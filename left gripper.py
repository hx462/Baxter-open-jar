import rospy
import roslib

import cv;
import cv2;
import cv_bridge

import numpy
import math
import os
import sys
import string
import time
import random
import tf
from sensor_msgs.msg import Image
import baxter_interface
from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import std_srvs.srv
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest




#set the left gripper to a set position,but within the distance in space with respect to the right gripper.


# move a limb
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if self.limb == limb:               # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.pose = [position[0], position[1],                                \
                         self.pose[2], self.pose[3], self.pose[4], self.pose[5]]

# check that the tolerance meet 

    def check_position(s)
	
	pose_right = (self.pose[0] + self.cam_x_offset,
                self.pose[1] + self.cam_y_offset,
                self.pose[2] + (0.112 - self.distance),
                self.pose[3],
                self.pose[4],
                self.pose[5])
				
	pose_left = (self.pose[0] + self.cam_x_offset,
                self.pose[1] + self.cam_y_offset,
                self.pose[2] + (0.112 - self.distance),
                self.pose[3],
                self.pose[4],
                self.pose[5])
				
	i = 0 			
	while right_self.pose[0] - left_self.pose[0] > error0||
	      right_self.pose[1] - left_self.pose[1] > error1||
		  right_self.pose[2] - left_self.pose[2] > error2:
		  
		   move_to_joint_positions = 
		   
# enable camera
# Enable the actuators
        baxter_interface.RobotEnable().enable()

        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.5)
        self.other_limb_interface.set_joint_position_speed(0.5)

        # calibrate the gripper
        self.gripper.calibrate()

        # reset cameras
        self.reset_cameras()

        # close all cameras
        self.close_camera("left")
        self.close_camera("right")
        self.close_camera("head")

        # open required camera
        self.open_camera(self.limb, self.width, self.height)

        # subscribe to required camera
        self.subscribe_to_camera(self.limb)


		
		
# tracking the circle using Hough Algorithm
		  
		   
		
		
				
				