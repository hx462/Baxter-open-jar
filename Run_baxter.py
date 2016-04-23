#!/usr/bin/env python
import rospy
import cv
import cv2
import cv_bridge
import numpy
import math
import sys
import baxter_interface
import moveit_commander
import std_srvs.srv

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_commander import conversions

rospy.init_node('unscrew', anonymous=True)

class locate():
    def __init__(self, arm):
        self.limb = arm
        self.limb_interface = baxter_interface.Limb(arm)
        self.gripper = baxter_interface.Gripper(arm)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.tolerance = 0.005

        self.pos_x = 0.50
        self.pos_y = 0.00
        self.pos_z = 0.05
        self.or_x = 0.0
        self.or_y = -1.0
        self.or_z = 0.0
        self.or_w = 0.0

        self.pose = [self.pos_x, self.pos_y, self.pos_z, self.or_x, self.or_y, self.or_z, self.or_w]

        self.hough_accumulator = 30
        self.hough_min_radius = 35
        self.hough_max_radius = 70

        self.cam_calib    = 0.0004
        self.cam_x_offset = 0.045
        self.cam_y_offset = -0.01
        self.width        = 960
        self.height       = 600

        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)

        baxter_interface.RobotEnable().enable()

        self.limb_interface.set_joint_position_speed(0.5)

        self.gripper.calibrate()

        self.reset_camera(self.limb, self.width, self.height)

        self.subscribe_to_camera(self.limb)

        self.distance = 0

    def reset_camera(self, limb, width, height):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()
        cam = baxter_interface.camera.CameraController(limb + '_hand_camera')
        cam.close()
        cam.resolution = (int(width), int(height))
        cam.exposure = -1
        cam.gain = -1
        cam.white_balance_blue = -1
        cam.white_balance_green = -1
        cam.white_balance_red = -1
        cam.open()

    def camera_callback(self, data, camera_name):
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, 'bgr8')
        except cv_bridge.CvBridgeError, e:
            print e
        cv.WaitKey(3)

    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    def subscribe_to_camera(self, limb):
        if limb == 'left':
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        else:
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        camera_sub = rospy.Subscriber(camera_str, Image, callback)

    def cv2array(self, im):
        depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                       cv.IPL_DEPTH_8S: 'int8',
                       cv.IPL_DEPTH_16U: 'uint16',
                       cv.IPL_DEPTH_16S: 'int16',
                       cv.IPL_DEPTH_32S: 'int32',
                       cv.IPL_DEPTH_32F: 'float32',
                       cv.IPL_DEPTH_64F: 'float64'}
        a = numpy.fromstring(im.tostring(), dtype=depth2dtype[im.depth], count=im.width*im.height*im.nChannels)
        a.shape = (im.height, im.width, im.nChannels)
        return a

    def find_lid(self, circle_data, iteration):
        print('Finding the lid...')
        if len(circle_data) == 1:
            return circle_data[0]
        else:
            lid = None
            max_area = -1
            for i in range(len(circle_data)):
                r = circle_data[i][2]
                area = numpy.pi * r * r
                if area > max_area:
                    lid = circle_data[i]
                    max_area = area
            print('Found lid at ' + str(lid[0]) + ', ' + str(lid[1]) + ' with radius ' + str(lid[2]))
            return lid

    def hough(self, iteration):
        print('Applying Hough...')
        gray_image = cv.CreateImage((self.width, self.height), 8 , 1)
        cv.CvtColor(self.cv_image, gray_image, cv.CV_BGR2GRAY)
        gray_array = self.cv2array(gray_image)
        cv2.GaussianBlur(gray_array, (5, 5), 0)

        circles = cv2.HoughCircles(gray_array, cv.CV_HOUGH_GRADIENT, 1, 40, param1=50, param2=self.hough_accumulator, minRadius=self.hough_min_radius, maxRadius=self.hough_max_radius)

        if circles is None:
            sys.exit('Error in hough(), no lid found')

        circles = numpy.uint16(numpy.around(circles))

        circle_data = {}
        n_circles = 0

        circle_array = numpy.asarray(self.cv_image)

        for i in circles[0,:]:
            cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(circle_array, (i[0], i[1]), 2, (0, 255, 0), 3)
            circle_data[n_circles] = (i[0], i[1], i[2])
            n_circles += 1

        circle_image = cv.fromarray(circle_array)
        cv.ShowImage("Hough Circle", circle_image)
        cv.WaitKey(3)

        if n_circles == 0:
            sys.exit('Error in hough(), no lid found')

        lid = self.find_lid(circle_data, iteration)

        return lid

    def baxter_ik_move(self, limb, pose):
        print('Moving arm...')
        arm = moveit_commander.MoveGroupCommander(limb + '_arm')

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base'

        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = pose[2]

        goal_pose.pose.orientation.x = pose[3]
        goal_pose.pose.orientation.y = pose[4]
        goal_pose.pose.orientation.z = pose[5]
        goal_pose.pose.orientation.w = pose[6]

		
        arm.set_pose_target(goal_pose)
        arm.set_start_state_to_current_state()
        plan = arm.plan()

        arm.execute(plan)

        if self.limb == limb:
            quaternion_pose = self.limb_interface.endpoint_pose()
            position = quaternion_pose['position']
            self.pose = [position[0], position[1], self.pose[2], self.pose[3], self.pose[4], self.pose[5], self.pose[6]]

        rospy.sleep(5.0)

    def get_distance(self, limb):
        print('Getting distance...')
        dist = baxter_interface.analog_io.AnalogIO(limb + '_hand_range').state()
        print ('Distance = ' + str(float(dist / 1000.0)))
        return float(dist / 1000.0)

		
		
		
		
	def gripper_pose(self, deviation, x, y):
	
## adjust the gripper based on the deviation
			
		self.pose = [position[0], position[1], position[2], or_x, or_y, or_z, self.or_w]
		
		deviation =  
		









	
    def update_pose(self, dx, dy):
        x = self.pose[0] - dx
        y = self.pose[1] - dy
        print('Updating pose from ' + str(self.pose[0]) + ', ' + str(self.pose[1]) + ' to ' + str(x) + ', ' + str(y) + '...')
        pose = [x, y, self.pose[2], self.pose[3], self.pose[4], self.pose[5], self.pose[6]]
		
##		self.distance_0 = self.get_distance(self.limb)
		
        self.baxter_ik_move(self.limb, pose)

## update the distance of the gripper to the table		
##		self.distance = self.get_distance(self.limb)

## check if it change, cause the vertical distance should not change much when we are finding the lid
		while deviation = self.distance - self.distance_0 > tolerance_distance and error > self.tolerance and iteration < 10:
		
            pose = [self.pose[0],, self.pose[1], self.pose[2], or_x, or_y, or_z, self.pose[6]]
			self.baxter_ik_move(self.limb, pose)
			self.pose = self.gripper_pose(self,deviation,x,y)
			
			


			
			
	    
	    pose = baxter_ik_move

    def iterate(self, iteration, lid_data):
        print('Iterating...')
        pixel_dx = (self.width / 2) - lid_data[0]
        pixel_dy = (self.height / 2) - lid_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error = float(pixel_error * self.cam_calib)

        print('Pixel offset by ' + str(pixel_dx) + ', ' + str(pixel_dy) + '...')

        x_offset = - pixel_dy * self.cam_calib
        y_offset = - pixel_dx * self.cam_calib

        print('World offset by ' + str(x_offset) + ', ' + str(y_offset) + '...')

        self.update_pose(x_offset, y_offset)
        lid_data = self.hough(iteration)

        pixel_dx = (self.width / 2) - lid_data[0]
        pixel_dy = (self.height / 2) - lid_data[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error = float(pixel_error * self.cam_calib)
        return lid_data, error

    def unscrew(self):
        iteration = 0

        next_lid = self.hough(iteration)
        error = 2 * self.tolerance

        while error > self.tolerance and iteration < 10:
            iteration += 1
            next_lid, error = self.iterate(iteration, next_lid)

        self.distance = self.get_distance(self.limb)

        self.limb_interface.set_joint_position_speed(0.1)

        pose = [self.pose[0] + self.cam_x_offset,
                self.pose[1] + self.cam_y_offset,
                self.pose[2] + (0.112 - self.distance),
                self.pose[3],
                self.pose[4],
                self.pose[5],
                self.pose[6]]

        self.baxter_ik_move(self.limb, pose)

        self.gripper.close()

        # DO THE UNSCREWING

def main():
    locator = locate('right')
    locator.gripper.open()

    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()
    left_gripper.open()
    raw_input('Press Enter to start: ')
    left_gripper.close()
    #hold_pose = []
    #locator.baxter_ik_move('left', hold_pose)

    locator.baxter_ik_move(locator.limb, locator.pose)
    locator.unscrew()

if __name__ == "__main__":
    main()
