#!/usr/bin/env python

#####################################################################################
#                                                                                   #
# Copyright (c) 2014, Active Robots Ltd.                                            #
# All rights reserved.                                                              #
#                                                                                   #
# Redistribution and use in source and binary forms, with or without                #
# modification, are permitted provided that the following conditions are met:       #
#                                                                                   #
# 1. Redistributions of source code must retain the above copyright notice,         #
#    this list of conditions and the following disclaimer.                          #
# 2. Redistributions in binary form must reproduce the above copyright              #
#    notice, this list of conditions and the following disclaimer in the            #
#    documentation and/or other materials provided with the distribution.           #
# 3. Neither the name of the Active Robots nor the names of its contributors        #
#    may be used to endorse or promote products derived from this software          #
#    without specific prior written permission.                                     #
#                                                                                   #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"       #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE         #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE        #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE          #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR               #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF              #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS          #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN           #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)           #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        #
# POSSIBILITY OF SUCH DAMAGE.                                                       #
#                                                                                   #
#####################################################################################

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

# load the package manifest
roslib.load_manifest("activerobots")

# initialise ros node
rospy.init_node("pick_and_place", anonymous = True)

# directory used to save analysis images
image_directory = os.getenv("HOME") + "/Golf/"

# locate class
class locate():
    def __init__(self, arm, distance):
        # arm ("left" or "right")
        self.limb           = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # required position accuracy in metres
        self.ball_tolerance = 0.005
        self.tray_tolerance = 0.05

        # start positions
        self.ball_tray_x = 0.50                        # x     = front back
        self.ball_tray_y = 0.30                        # y     = left right
        self.ball_tray_z = 0.15                        # z     = up down
        self.golf_ball_x = 0.50                        # x     = front back
        self.golf_ball_y = 0.00                        # y     = left right
        self.golf_ball_z = 0.15                        # z     = up down
        self.roll        = -1.0 * math.pi              # roll  = horizontal
        self.pitch       = 0.0 * math.pi               # pitch = vertical
        self.yaw         = 0.0 * math.pi               # yaw   = rotation

        self.pose = [self.golf_ball_x, self.golf_ball_y, self.golf_ball_z,     \
                     self.roll, self.pitch, self.yaw]

        # camera parameters (NB. other parameters in open_camera)
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.045                      # camera gripper offset
        self.cam_y_offset = -0.01
        self.width        = 960                        # Camera resolution
        self.height       = 600

        # Hough circle accumulator threshold and minimum radius.
        self.hough_accumulator = 35
        self.hough_min_radius  = 15
        self.hough_max_radius  = 35

        # minimum ball tray area
        self.min_area = 20000

        # callback image
        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)

        # colours
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

        # ball tray corners
        self.ball_tray_corner = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]

        # ball tray places
        self.ball_tray_place = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]

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

        # distance of arm to table and ball tray
        self.distance      = distance
        self.tray_distance = distance - 0.075

        # move other arm out of harms way
        if arm == "left":
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))

    # reset all cameras (incase cameras fail to be recognised on boot)
    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()

    # open a camera and set camera parameters
    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # close camera
        cam.close()

        # set camera parameters
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # open camera
        cam.open()

    # close a camera
    def close_camera(self, camera):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - close_camera - Invalid camera")

        # set camera parameters to automatic
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # close camera
        cam.close()

    # convert image pixel to Baxter point
    def pixel_to_baxter(self, px, dist):
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist)                \
          + self.pose[0] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist)                 \
          + self.pose[1] + self.cam_y_offset

        return (x, y)

    # camera call back function
    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, "bgr8")
        except cv_bridge.CvBridgeError, e:
            print e

        # 3ms wait
        cv.WaitKey(3)

    # left camera call back function
    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    # right camera call back function
    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    # head camera call back function
    def head_camera_callback(self, data):
        self.camera_callback(data, "Head Camera")

    # create subscriber to the required camera
    def subscribe_to_camera(self, camera):
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        camera_sub = rospy.Subscriber(camera_str, Image, callback)

    # Convert cv image to a numpy array
    def cv2array(self, im):
        depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                       cv.IPL_DEPTH_8S: 'int8',
                       cv.IPL_DEPTH_16U: 'uint16',
                       cv.IPL_DEPTH_16S: 'int16',
                       cv.IPL_DEPTH_32S: 'int32',
                       cv.IPL_DEPTH_32F: 'float32',
                       cv.IPL_DEPTH_64F: 'float64'}

        arrdtype=im.depth
        a = numpy.fromstring(im.tostring(),
                             dtype = depth2dtype[im.depth],
                             count = im.width * im.height * im.nChannels)
        a.shape = (im.height, im.width, im.nChannels)

        return a

    # find next object of interest
    def find_next_golf_ball(self, ball_data, iteration):
        # if only one object then object found
        if len(ball_data) == 1:
            return ball_data[0]

        # sort objects right to left
        od = []
        for i in range(len(ball_data)):
            od.append(ball_data[i])

        od.sort()

        # if one ball is significantly to the right of the others
        if od[1][0] - od[0][0] > 30:       # if ball significantly to right of the others
            return od[0]                   # return right most ball
        elif od[1][1] < od[0][1]:          # if right most ball below second ball
            return od[0]                   # return lower ball
        else:                              # if second ball below right most ball
            return od[1]                   # return lower ball

    # Use Hough circles to find circle centres (Only works with round objects)
    def hough_it(self, iteration):
        # create gray scale image
        gray_image = cv.CreateImage((self.width, self.height), 8, 1)
        cv.CvtColor(self.cv_image, gray_image, cv.CV_BGR2GRAY)

        # create gray scale array
        gray_array = self.cv2array(gray_image)

        # find Hough circles
        circles = cv2.HoughCircles(gray_array, cv.CV_HOUGH_GRADIENT, 1, 40, param1=50,  \
                  param2=self.hough_accumulator, minRadius=self.hough_min_radius,       \
                  maxRadius=self.hough_max_radius)

        # Check for at least one ball found
        if circles is None:
            # no point in continuing so exit with error message
            sys.exit("ERROR - hough_it - No golf balls found")

        circles = numpy.uint16(numpy.around(circles))

        ball_data = {}
        n_balls   = 0

        circle_array = numpy.asarray(self.cv_image)

        # check if golf ball is in ball tray
        for i in circles[0,:]:
            # convert to baxter coordinates
            ball = self.pixel_to_baxter((i[0], i[1]), self.tray_distance)

            if i[1] > 800:
                # draw the outer circle in red
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
                # draw the center of the circle in red
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 0, 255), 3)
            else:
                # draw the outer circle in green
                cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle in green
                cv2.circle(circle_array, (i[0], i[1]), 2, (0, 255, 0), 3)

                ball_data[n_balls]  = (i[0], i[1], i[2])
                n_balls            += 1

        circle_image = cv.fromarray(circle_array)

        cv.ShowImage("Hough Circle", circle_image)

        # 3ms wait
        cv.WaitKey(3)

        # Check for at least one ball found
        if n_balls == 0:                    # no balls found
            # less than 12 balls found, no point in continuing, exit with error message
            sys.exit("ERROR - hough_it - No golf balls found")

        # select next ball and find it's position
        next_ball = self.find_next_golf_ball(ball_data, iteration)

        # return next golf ball position and pickup angle
        return next_ball

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

    # find distance of limb from nearest line of sight object
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)

    # update pose in x and y direction
    def update_pose(self, dx, dy):
        x = self.pose[0] + dx
        y = self.pose[1] + dy
        pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)

    # used to place camera over golf ball
    def golf_ball_iterate(self, iteration, ball_data):

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.tray_distance)

        x_offset = - pixel_dy * self.cam_calib * self.tray_distance
        y_offset = - pixel_dx * self.cam_calib * self.tray_distance

        # update pose and find new ball data
        self.update_pose(x_offset, y_offset)
        ball_data, angle = self.hough_it(iteration)

        # find displacement of ball from centre of image
        pixel_dx    = (self.width / 2) - ball_data[0]
        pixel_dy    = (self.height / 2) - ball_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.tray_distance)

        return ball_data, error

    # find all the golf balls and place them in the ball tray
    def pick_and_place(self):
        iteration = 0

        # use Hough circles to find balls and select one ball
        next_ball = self.hough_it(iteration)
        error     = 2 * self.ball_tolerance

        # iterate to find next golf ball
        # if hunting to and fro accept error in position
        while error > self.ball_tolerance and iteration < 10:
            iteration += 1
            next_ball, error  = self.golf_ball_iterate(iteration, next_ball)

        # slow down to reduce scattering of neighbouring golf balls
        self.limb_interface.set_joint_position_speed(0.1)

        # move down to pick up ball
        pose = (self.pose[0] + self.cam_x_offset,
                self.pose[1] + self.cam_y_offset,
                self.pose[2] + (0.112 - self.distance),
                self.pose[3],
                self.pose[4],
                self.pose[5])
        self.baxter_ik_move(self.limb, pose)

		
        # close the gripper
        self.gripper.close()

        # UNSCREW IT

def main():
    # create locate class instances
    left_locator = locate('left', 0.0)
    right_locator = locate('right', 0.0)
    left_locator.gripper.open()
    right_locator.gripper.open()

    raw_input("Press Enter to start: ")
    left_locator.gripper.close()
    left_locator.pose = [x, y, z, roll, pitch, yaw]
    left_locator.baxter_ik_move(left_locator.limb, left_locator.pose)

    right_locator.pose = [x, y, z, roll, pitch, yaw]
    right_locator.baxter_ik_move(right_locator.limb, right_locator.pose)
    right_locator.distance = right_locator.get_distance(right_locator.limb)
    right_locator.pick_and_place()

if __name__ == "__main__":
    main()

