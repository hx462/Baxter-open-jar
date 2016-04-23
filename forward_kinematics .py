#!/usr/bin/env python


import rospy
from sensor_msgs.msg import JointState

import numpy as np
import scipy as sp
import math

np.set_printoptions(precision=4,suppress=True)

theta = 0;
xi = np.zeros((6,7));

fixed_base = np.array([[0,0,0]]);
moving_tool = np.array([[.7957,.9965,.3058]]);


gst_0 = np.array([[1,0,0,.7957],[0,1,0,.9965],[0,0,1,.3058],[0,0,0,1]]);
#print(gst_0)


q = np.array([[.0635,.2598,.1188],[.1106,.3116,.3885],[.1827,.3838,.3881],[.3682,.5684,.3181],[.4417,.6420,.3177],[.6332,.8337,.3067],[.7152,.9158,.3063]]).T

omega = np.array([[-.0059,.0113,.9999],[-.7077,.7065,-.0122],[.7065,.7077,-.0038],[-.7077,.7065,-.0122],[.7065,.7077,-.0038],[-.7077,.7065,-.0122],[.7065,.7077,-.0038]]).T;

#print(omega)
#print(q)
def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix

    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    omega_hat = np.array([[0,-omega[2],omega[1]],
                          [omega[2],0,-omega[0]],
                          [-omega[1],omega[0],0]]);


    return omega_hat

def rotation_3d(omega, theta):
    """
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    rot=np.eye(3) + skew_3d(omega)*math.sin(np.linalg.norm(omega)*theta)/np.linalg.norm(omega) + np.dot(skew_3d(omega),skew_3d(omega))*(1-math.cos(np.linalg.norm(omega)*theta))/np.square(np.linalg.norm(omega))
    return rot

def homog_3d(xi, theta):
    """    
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    wV = np.split(xi,2)


    w = wV[1]
    VV = wV[0]

    R = rotation_3d(w, theta)

    g1 = R
    g2 = (np.dot((np.eye(3)-R),np.dot(skew_3d(w),VV)))/np.square(np.linalg.norm(w)) + (np.dot(w,np.dot(w,np.dot(VV,theta))))/np.square(np.linalg.norm(w))
    g12 = np.insert(g1,3,g2,axis=1)

    g3 = np.array([[0,0,0,1]])

    g = np.concatenate((g12,g3),axis=0)

    return g


def prod_exp(xi, theta):
    """

    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)

    homog_3d(xi, theta)
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')

    g = np.eye(4)
    for i in range(0,len(theta)) :
        ex = homog_3d(xi[0:6,i], theta[i])
        g = np.dot(g,ex)
    return g

def v(omega,q):

    if not omega.shape[0] == 3:
       raise TypeError('omega must be a 3xN')
    
    
    v = np.zeros((3,len(q.T)))
    #print(v)
    for i in range(0,len(q.T)) :
        
        v[0:3,i] = -np.dot(skew_3d(omega[0:3,i]),q[0:3,i])
     
        #v = np.insert(v,3,v,axis=1)

    return v


def xi(omega,v):

    if not omega.shape[0] == 3:
       raise TypeError('omega must be a 3xN')

    xi = np.zeros((6,7))
    for i in range(0,len(q.T)) :

        xi[0:6,i] = np.concatenate((v[0:3,i],omega[0:3,i]),axis=0)

    return xi


#Subcriber function
def callback(message):

    theta1 = round(message.position[2],10)
    theta2 = round(message.position[3],10)
    theta3 = round(message.position[0],10)
    theta4 = round(message.position[1],10)
    theta5 = message.position[4]
    theta6 = message.position[5]
    theta7 = message.position[6]
    theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6, theta7])
    print 'joint_angles: \n', theta

    #v = v(omega,q)
    #print(v)
    #print(v)
    #print 'joint_angles: ', v
    #print '\n'
    v = np.zeros((3,len(q.T)))
    for i in range(0,len(q.T)) :       
        v[0:3,i] = -np.dot(skew_3d(omega[0:3,i]),q[0:3,i])     
    print 'vectors: \n', v


    xi = np.zeros((6,7))
    #print xi
    for i in range(0,len(q.T)) :
        xi[0:6,i] = np.concatenate((v[0:3,i],omega[0:3,i]),axis=0)
    #xi = xi(omega,v)
    print 'xi: \n', xi

    g = prod_exp(xi, theta);
    print 'map: \n', g

    new_tranformation = np.dot(g,gst_0)
    print 'new_tranformation: \n', new_tranformation

    

#Define the method which contains the node's main functionality
def forward_kinematics():


    rospy.init_node('Position', anonymous=True)


    rospy.Subscriber("/robot/assembly/left/joint_states", JointState, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()






#-----------------------------Testing code--------------------------------------
#-------------------------------------------------------------------------------



if __name__ == "__main__":
    print('Testing...')


#---Test value of theta---
#theta = np.array([0, 0, 0, 0, 0, 0, 0])
#theta = np.array([1,1,1,1,1,1,1])
#print(theta)
#for i in range (7):
#    joint_angles = raw_input("Enter joint_angles: ")
#    theta[i] = joint_angles
#print 'joint_angles: ', theta


#Get Message Here
forward_kinematics()

#new_tranformation = np.dot(g,gst_0)
#print(new_tranformation)
