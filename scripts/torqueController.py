#!/usr/bin/env python
import rospy
import csv
import time
import struct
from fb5_torque_ctrl.msg import encoderData
from fb5_torque_ctrl.msg import PwmInput
import math
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
	wR=(data.encoderR-encRPrev)*2*math.pi/512/duration
        wL=(data.encoderL-encLPrev)*2*math.pi/512/duration
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

def torqueController():
	global pwmInput
	global codeStartTime
	pub_PWM=rospy.Publisher('pwmCmd',PwmInput,queue_size=10)
        rospy.init_node('torqueController',anonymous=True)
        rospy.Subscriber('encoderData', encoderData, callback)
        #The torque controller outputs commands at only 10Hz.
	#The encoder data is still at 25 Hz as determined by the publisher in the other file
	codeStartTime=rospy.get_time()
	rate = rospy.Rate(10)
	timeSinceInput=rospy.get_time()
        timeBetInputs=0.1
        while not rospy.is_shutdown():
		i=0
		while i<360:
			pwmInput.rightInput=175+75*math.sin(8*i*math.pi/180)
			pwmInput.leftInput=175+75*math.sin(8*i*math.pi/180)
                #while i<2:
		#	pwmInput.rightInput=120*i;
		#	pwmInput.leftInput=0*i;
		        #rospy.loginfo(pwmInput)
                        pub_PWM.publish(pwmInput)
			if((rospy.get_time()-timeSinceInput)>=timeBetInputs):
				timeSinceInput=rospy.get_time()
                                i=i+1
			rate.sleep()
if __name__ == '__main__':
    try:
        torqueController()
    except rospy.ROSInterruptException:
        pass

