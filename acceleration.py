# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

# Header header

# geometry_msgs/Quaternion orientation
# float64[9] orientation_covariance # Row major about x, y, z axes

#geometry_msgs/Vector3 angular_velocity
#float64[9] angular_velocity_covariance # Row major about x, y, z axes

#geometry_msgs/Vector3 linear_acceleration
#float64[9] linear_acceleration_covariance # Row major x, y z 
#Publisher


from sensor_msgs.msgs import Imu


		


def callback(message):

    for joint in message.linear_acceleration():
	    difference = abs(message.linear_acceleration.z - current_positon.z)
		if difference> 0.1
		   rospy.signal_shutdown("Moving.")
		   return True
	return False	 
	
	
	
def check_joints(joint_command):
        for joint in joint_command.keys():
            error=abs(joint_command[joint] - left.joint_angle(joint))
            print "Error: %s, Goal: %s, Actual: %s"%(str(error), str(joint_command[joint]), str(left.joint_angle(joint)))
            print error < 0.1 
            if error < 0.1:
                rospy.signal_shutdown("Example finished.")
                return True 
        return False  
     
    


def Pull_up_Sensing():


    rospy.init_node('Position', anonymous=True)


    rospy.Subscriber("/robot/accelerometer/left_accelerometer/state", Left_State, callback)

	
    rospy.spin()

