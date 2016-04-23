import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

def forward_kinematics_map(theta):
	gst0 = np.array([[1, 0, 0, 0.7957],
									 [0, 1, 0, 0.9965],
									 [0, 0, 1, 0.3058],
									 [0, 0, 0, 1]])

	w1 = np.array([-0.0059,0.0113,0.9999]).T
	q1 = np.array([0.0635,0.2598,0.1188]).T

	w2 = np.array([-0.7077,0.7065,-0.0122]).T
	q2 = np.array([0.1106,0.3116,0.3885]).T

	w3 = np.array([0.7065,0.7077,-0.0038]).T
	q3 = np.array([0.1827,0.3838,0.3881]).T

	w4 = np.array([-0.7077,0.7065,-0.0122]).T
	q4 = np.array([0.3682,0.5684,0.3181]).T

	w5 = np.array([0.7065,0.7077,-0.0038]).T
	q5 = np.array([0.4417,0.6420,0.3177]).T

	w6 = np.array([-0.7077,0.7065,-0.0122]).T
	q6 = np.array([0.6332,0.8337,0.3067]).T

	w7 = np.array([0.7065,0.7077,-0.0038]).T
	q7 = np.array([0.7152,0.9158,0.3063]).T

	t1 = np.zeros((6, 1))
	temp = np.cross(w1, q1)
	for i in range(3):
		t1[i,0] = -temp[i]
		t1[i+3,0] = w1[i]

	t2 = np.zeros((6, 1))
	temp = np.cross(w2, q2)
	for i in range(3):
		t2[i,0] = -temp[i]
		t2[i+3,0] = w2[i]

	t3 = np.zeros((6, 1))
	temp = np.cross(w3, q3)
	for i in range(3):
		t3[i,0] = -temp[i]
		t3[i+3,0] = w3[i]

	t4 = np.zeros((6, 1))
	temp = np.cross(w4, q4)
	for i in range(3):
		t4[i,0] = -temp[i]
		t4[i+3,0] = w4[i]

	t5 = np.zeros((6, 1))
	temp = np.cross(w5, q5)
	for i in range(3):
		t5[i,0] = -temp[i]
		t5[i+3,0] = w5[i]

	t6 = np.zeros((6, 1))
	temp = np.cross(w6, q6)
	for i in range(3):
		t6[i,0] = -temp[i]
		t6[i+3,0] = w6[i]

	t7 = np.zeros((6, 1))
	temp = np.cross(w7, q7)
	for i in range(3):
		t7[i,0] = -temp[i]
		t7[i+3,0] = w7[i]

	twists = np.array([t1.T[0], t2.T[0], t3.T[0], t4.T[0], t5.T[0], t6.T[0], t7.T[0]]).T

	return np.dot(kfs.prod_exp(twists, theta), gst0)

print('Expect 0.2, 0.3, 0.4')
print forward_kinematics_map(np.array([1.7017, -0.7907, -2.772, 2.412, -2.8835, -1.4779, 2.7314]))

print('Expect 0.1, 0.5, 0.4')
print forward_kinematics_map(np.array([-1.0422, -0.779, 2.4161, 2.229, -0.4918, 1.5622, 2.3321]))

print('Expect 0.3, 0.1, 0.6')
print forward_kinematics_map(np.array([-1.4347, -2.1284, 0.0362, 1.7358, 0.0207, 1.9630, -0.6106]))



joint_recorder.py

