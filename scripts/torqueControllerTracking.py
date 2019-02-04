#!/usr/bin/env python
import rospy
import csv
import time
import struct
import numpy as np
from fb5_torque_ctrl.msg import encoderData
from fb5_torque_ctrl.msg import PwmInput
import math
from geometry_msgs.msg import TransformStamped

#from tf.transformations import euler_from_quaternion
#Defining as global variable for loging
pwmInput=PwmInput()
codeStartTime=0
#The experimental data is added to the data.csv file
head=['time','interval','encR','encL','wR','wL','wdotR','wdotL','wddotR','wddotL','pwmR','pwmL']
with open('data.csv','w') as myfile:
	writer=csv.writer(myfile)
	writer.writerow(head)
encRPrev=0
encLPrev=0
wRprev=0
wLprev=0
wdotRprev=0
wdotLprev=0


####################################################################
####################################################################

#Defining the bot properties the best we know
m=1.72 #mass in kg
d=0.065 #distance from CG to axis
R=0.085	#Semi-Distance between wheels
r=0.025 #Radius of wheels

#Controller gains
k1=1
k2=1
k3=1
k4=1

#Defining the nonlinear system properties.
def S(q):
	global d
	s=np.matrix([[math.cos(q[2]), -d*math.sin(q[2])],[math.sin(q[2]), d*math.cos(q[2])],[0, 1]])
	return s

def S_dot(q,q_dot):
	global d

	s_dot=np.matrix([[-math.sin(q[2])*(q_dot[2][0]), -d*math.cos(q[2])*(q_dot[2][0])],[math.cos(q[2])*(q_dot[2][0]), -d*math.sin(q[2])*(q_dot[2][0])],[0, 0]])
	#print "s_dot:", s_dot
	return s_dot

def M(q):
	global d
	global m
	inertia=np.matrix([[m, 0, m*d*math.sin(q[2])],[0, m, -m*d*math.cos(q[2])],[m*d*math.sin(q[2]), -m*d*math.cos(q[2]), 1]])
	return inertia

def V(q,q_dot):
	global d
	global m
	coriolis=np.matrix([[0, 0, m*d*(q_dot[2])*math.cos(q[2])],[0, 0, m*d*(q_dot[2])*math.sin(q[2])],[0, 0, 0]])
	return coriolis

def B(q):
	global R
	global r
	cont_transform=1/r*np.matrix([[math.cos(q[2]), math.cos(q[2])],[math.sin(q[2]), math.sin(q[2])],[R, -R]])
	return cont_transform

#def M_bar, B_bar, V_bar and F_bar.

#By calculation we obtain B_bar as [[1/r,1/r],[R/r -R/r]]
def B_bar(q):
	b_bar=S(q).transpose()*B(q)
	return b_bar

#By calculation we obtain M_bar as [[m 0],[0 1-md^2]].
def M_bar(q):
	return S(q).transpose()*M(q)*S(q)


#By calculation we get V_bar as a 2x2 null matrix.
def V_bar(q,q_dot):
	#print "MSdot:", M(q)*S_dot(q,q_dot)
	return S(q).transpose()*(M(q)*S_dot(q,q_dot)+V(q,q_dot)*S(q))

#Assume constant rolling friction acting opposite to the direction of motion.
F_bar = np.matrix([[0.01],[0.01]])

#Create reference trajectory. This code only accomadates constant vr and wr. The one below makkes it go in a circle.
def traj_req(t,dt,qr_prev):
	vr=0.05		#in m/s
	wr=0.1		#in rad/s
	xr_dot=vr*math.cos(qr_prev[2])
	yr_dot=vr*math.sin(qr_prev[2])
	thetar=qr_prev[2]+wr*dt
	xr=qr_prev[0][0]+xr_dot*dt
	yr=qr_prev[1]+yr_dot*dt
	return np.array([[xr],[yr],[thetar],[vr],[wr]])

#Defining the required torque at each instant.
def torque(q,q_dot,v,u):
	global F_bar
	return B_bar(q).I*(M_bar(q)*np.matrix(u)+V_bar(q,q_dot)*np.matrix(v)+F_bar) #Assuming zero disturbance Tau_d

#Rotation matrix from body frame to inertial frame
def rot2body(q):
	rotmat=np.matrix([[math.cos(q[2]), math.sin(q[2]), 0],[-math.sin(q[2]), math.cos(q[2]), 0],[0, 0, 1]])
	return rotmat

#The control velocity required
def vc(e,vr,wr):
	global k1
	global k2
	global k3
	vc1=vr*math.cos(e[2])+k1*e[0]
	vc2=wr+k2*vr*e[1]+k3*vr*math.sin(e[2])
	return np.array([vc1,vc2])

#The rate of change of control velocity
def vc_dot(e,v,vr,wr):
	global k1
	global k2
	global k3
	edot=np.matrix(e_dot(e,v,vr,wr))
	return np.matrix([[k1, 0, -vr*math.sin(e[2])],[0, k2*vr, k3*vr*math.cos(e[2])]])*edot

#Error dynamics
def e_dot(e,v,vr,wr):
	e1=v[1]*e[1]-v[0]+vr*math.cos(e[2])
	e2=-v[1]*e[0]+vr*math.sin(e[2])
	e3=wr-v[1]
	return np.array([e1,e2,e3])

#Torque measurements
#tau_array=np.array([[0],[0]]) #For storage maybe


#Since tf package can't be directly installed on RasPi the function below has be written. 
#Note that this function cannot handle singularities. It gives ZYX rotation Euler angles in radian.
def quat2eul(q):
	#Computing Phi
	phi = math.atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2))
	
	#Computing Theta
	#Introducing a check to avoid numerical errors
	sinTheta=2*(q[0]*q[2]-q[1]*q[3])
	if sinTheta>=1:
		theta=math.pi/2
	else:
		theta=math.asin(sinTheta)

	#Computing Psi
	psi=math.atan2(2*(q[0]*q[3]+q[2]*q[1]),1-2*(q[2]**2+q[3]**2))
	return np.array([[phi],[theta],[psi]])


####################################################################
####################################################################
def callback(data):
	global encRPrev
	global encLPrev
	global wRprev
	global wLprev
	global wdotRprev
	global wdotLprev
	global pwmInput
	global codeStartTime

	#rospy.loginfo(rospy.get_caller_id''() + "I heard %s", data)
	#['interval','encR','encL','wR','wL','wdotR','wdotL','wddotR','wddotL']
	#30 counts is one rotation of the wheel i.e. 2*pi radian
	duration=0.04 #25 Hz
	wR=(data.encoderR-encRPrev)*2*math.pi/30/duration
    	wL=(data.encoderL-encLPrev)*2*math.pi/30/duration
   	wdotR=(wR-wRprev)/duration
    	wdotL=(wL-wLprev)/duration
   	wddotR=(wdotR-wdotRprev)/duration
    	wddotL=(wdotL-wdotLprev)/duration
	encRPrev=data.encoderR
	encLPrev=data.encoderL
	wRprev=wR
	wLprev=wL
	wdotRprev=wdotR
	wdotLprev=wdotL
    	logTime=rospy.get_time()-codeStartTime
    	row=[logTime,data.interval,data.encoderR,data.encoderL,wR,wL,wdotR,wdotL,wddotR,wddotL,pwmInput.rightInput,pwmInput.leftInput]
    	with open('data.csv','ab') as myfile:
        	writer=csv.writer(myfile)
        	writer.writerow(row)
	#print(rospy.get_time()-codeStartTime-logTime) #Just to see how long this logging takes

def callbackVICON(data):
	global q
	q[0][0]=data.transform.translation.x
	q[0][1]=data.transform.translation.y
	eulerAng=euler_from_quaternion([data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w])
	q[0][2]=eulerAng[2]
	print "Euler Angle", eulerAng
	print "[x_pos,y_pos,w_quat]",q

def torqueController():
	global pwmInput
	global codeStartTime
	#############################################################
	#global q
	#global encRPrev
	#global encLPrev
	#global wRprev
	#global wLprev
	#global wdotRprev
	#global wdotLprev
	#############################################################
	rospy.init_node('torqueController',anonymous=True)
    	rospy.Subscriber('encoderData', encoderData, callback)
	pub_PWM=rospy.Publisher('pwmCmd',PwmInput,queue_size=10)

	#VICON data subscriber. Change the name to the required name here. 
    	rospy.Subscriber("/vicon/vijeth_0/vijeth_0", TransformStamped, callbackVICON)

    	#The torque controller outputs commands at only 10Hz.
	#The encoder data is still at 25 Hz as determined by the publisher in the other file
	#############################################################
	#Defning the initial point for bot the reference trajectory and the bot.
	#qr0=np.array([[0.0],[0.0],[0.0]])
	#qr_prev=qr0
	#qr_array=qr0 #For storage maybe

	#q0=np.array([[2],[2],[0.5]]) #This has to be changed to checking convergence while tracking.
	#q=q0 		#This has been randomly initialised will be overwritten by the VICON data right away.
	#qprev=q0
	#q_array=q0 #For storage maybe

	#Time parameters and initialisation
	#dt=1.0
	#timeLoopEnd=rospy.get_time()

	#Defining the initial velocity in cotinuatuion of defining the initial state of the bot.
	#vel0 = np.array([[0],[0]])
	#vel = vel0
	#vel_array = vel0 #For storage maybe

	#Defining error vectors. All propagation will happen with this.
	#e0 = np.array(rot2body(q0)*np.matrix(qr0-q0))
	#e=e0
	#e_array=e0 #For storage maybe
	#############################################################

	codeStartTime=rospy.get_time()
	rate = rospy.Rate(10)
	timeSinceInput=rospy.get_time()
   	timeBetInputs=5
    	while not rospy.is_shutdown():
		#############################################################
		#Defning the initial point for bot the reference trajectory and the bot.
		#qr0=np.array([[0.0],[0.0],[0.0]])
		#qr_prev=qr0
		#qr_array=qr0 #For storage maybe

		#q0=np.array([[2],[2],[0.5]]) #This has to be changed to checking convergence while tracking.
		#q=q0 		#This has been randomly initialised will be overwritten by the VICON data right away.
		#q_array=q0 #For storage maybe


		#dt=timeLoopEnd-rospy.get_time()
		#print "dt:", dt
		#q_dot = (q-q_prev)/dt

		#Velocity of states
		#vel0 = np.array([[0],[0]])
		#vel = np.array((S(q).transpose()*S(q)).I*S(q).transpose()*np.matrix(q_dot))
		#print "vel:", vel

		#Error in states
		#e=np.array(rot2body(q)*np.matrix(qr0-q0))

		#Generating the reference trajectory
		#ref = traj_req(t,0.2,qr_prev)
		#qr=ref[:3]
		#vel_ref=ref[3]
		#w_ref=ref[4]
		#qr_array=np.concatenate((qr_array,qr),axis=1)
		#qr_prev=qr

		#Controller
		#u=np.array(vc_dot(e,vel,vel_ref,w_ref)) + k4*(vc(e,vel_ref,w_ref)-vel)

		#Torque to be sent at each instant
		#tau=np.array(torque(q,q_dot,vel,u))
		#tau_array=np.concatenate((tau_array,np.array(torque(q,q_dot,vel,u))),axis=1) 
		#print tau_array

		#K_tau=100
		#Tau2PWM=K_tau*tau
		#print Tau2PWM

		#pwmInput.rightInput=K_tau*tau[0][0]
		#pwmInput.leftInput=K_tau*tau[0][1]
		#############################################################
		i=0
		while i<2:
			pwmInput.rightInput=150*i
			pwmInput.leftInput=-150*i
            		#rospy.loginfo(pwmInput)
	        	pub_PWM.publish(pwmInput)
			if((rospy.get_time()-timeSinceInput)>=timeBetInputs):
				timeSinceInput=rospy.get_time()
                                i=i+1
			rate.sleep()
		#############################################################
		#q_prev=q
		#timeLoopEnd=rospy.get_time()
		#rate.sleep()
		#############################################################

if __name__ == '__main__':
    try:
        torqueController()
    except rospy.ROSInterruptException:
        pass
