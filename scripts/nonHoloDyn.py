import numpy as np
import math
import matplotlib.pyplot as plt

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
	vr=0.5		#in m/s
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
#the rate of change of control velocity
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

#Defning the initial point for bot the reference trajectory and the bot.
qr0=np.array([[0.0],[0.0],[0.0]])
qr_prev=qr0
qr_array=qr0

q0=np.array([[2],[2],[0.5]]) #This has to be changed to checking convergence while tracking.
q=q0
q_array=q0

#Defining the initial velocity in cotinuatuion of defining the initial state of the bot.
vel0 = np.array([[0],[0]])
vel = vel0
vel_array = vel0

#Defining error vectors. All propagation will happen with this.
e0 = np.array(rot2body(q0)*np.matrix(qr0-q0))
e=e0
e_array=e0

#Torque measurements
tau_array=np.array([[0],[0]])

#Defining the simulation parameters
start=0.0
stop=100.0
steps=501
dt=(stop-start)/(steps-1)

for t in np.linspace(start,stop,steps):
	#Generating the reference trajectory
	ref = traj_req(t,0.2,qr_prev)
	qr=ref[:3]
	vel_ref=ref[3]
	w_ref=ref[4]
	qr_array=np.concatenate((qr_array,qr),axis=1)
	qr_prev=qr

	#Controller
	u=np.array(vc_dot(e,vel,vel_ref,w_ref)) + k4*(vc(e,vel_ref,w_ref)-vel)

	#Propagating the velocity
	v_dot=u
	vel=vel+v_dot*dt
	vel_array=np.concatenate((vel_array,vel),axis=1)

	#Propagating the states
	q_dot=np.array(S(q)*np.matrix(vel))
	q=q+q_dot*dt
	q_array=np.concatenate((q_array,q),axis=1)

	#Propagating the error
	e=np.array(rot2body(q)*np.matrix(qr-q))
	e_array=np.concatenate((e_array,e),axis=1)

	#Torque to be sent at each instant
	tau_array=np.concatenate((tau_array,np.array(torque(q,q_dot,vel,u))),axis=1) 


#plt.plot(qr_array[0],qr_array[1])
#plt.plot(q_array[0],q_array[1])
#plt.plot(np.linspace(start,stop,steps+1),tau_array[1])
#plt.plot(np.linspace(start,stop,steps+1),tau_array[0])
#plt.plot(np.linspace(start,stop,steps+1),vel_array[0])
#plt.plot(np.linspace(start,stop,steps+1),vel[0])

#plt.show()
