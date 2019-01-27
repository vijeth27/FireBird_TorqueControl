#!/usr/bin/env python
import rospy
import csv
import time
import struct
from fb5_torque_ctrl.msg import encoderData
from fb5_torque_ctrl.msg import PwmInput
from geometry_msgs.msg import TransformStamped
import math

#Defining as global variable for logging
codeStartTime=0
N_bots=4
bot_loc=np.empty((2,N_bots));
BotNumber=0

#The experimental data is added to the data.csv file
headEnc=['time','interval','encR','encL','wR','wL','wdotR','wdotL','wddotR','wddotL']
headPWM=['time','pwmR','pwmL']
headVICON=['time','vijeth_0_x','vijeth_0_y','vijeth_1_x','vijeth_1_y','vijeth_2_x','vijeth_2_y','vijeth_3_x','vijeth_3_y']

with open('dataEnc.csv','w') as myfile:
	writer=csv.writer(myfile)
	writer.writerow(headEnc)

with open('dataPWM.csv','w') as myfile:
	writer=csv.writer(myfile)
	writer.writerow(headPWM)

with open('dataVICON.csv','w') as myfile:
	writer=csv.writer(myfile)
	writer.writerow(headVICON)
encRPrev=0
encLPrev=0
wRprev=0
wLprev=0
wdotRprev=0
wdotLprev=0

def callbackEnc(data):
	global encRPrev
	global encLPrev
	global wRprev
	global wLprev
	global wdotRprev
	global wdotLprev
	global codeStartTime
	#rospy.loginfo(rospy.get_caller_id''() + "I heard %s", data)
	#['interval','encR','encL','wR','wL','wdotR','wdotL','wddotR','wddotL']
	#30 counts is one rotation of the wheel i.e. 2*pi radian
	duration=0.04 #25 Hz
	wR=(data.encoderR-encRPrev)*2*math.pi/512/duration
    wL=(data.encoderL-encLPrev)*2*math.pi/512/duration
	#wR=(data.encoderR-encRPrev)*2*math.pi/30/duration
    #wL=(data.encoderL-encLPrev)*2*math.pi/30/duration
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
    with open('dataEnc.csv','ab') as myfile:
            writer=csv.writer(myfile)
            writer.writerow(row)
	#print(rospy.get_time()-codeStartTime-logTime) #Just to see how long this logging takes

def callbackPWM(data):
	global codeStartTime
	logTime=rospy.get_time()-codeStartTime
    row=[logTime,data.rightInput,data.leftInput]
    with open('dataPWM.csv','ab') as myfile:
            writer=csv.writer(myfile)
            writer.writerow(row)
	#print(rospy.get_time()-codeStartTime-logTime) #Just to see how long this logging takes

def callbackVICON(data,arg):
	global bot_loc
	global BotNumber
	global codeStartTime
	bot_index=arg
	#Temporary shiz
	bot_loc[:,bot_index]=(data.transform.translation.x,data.transform.translation.y);
	
	if arg==BotNumber:
		logTime=rospy.get_time()-codeStartTime
		row=[logTime,bot_loc[0,0],bot_loc[1,0],bot_loc[0,1],bot_loc[1,1],bot_loc[0,2],bot_loc[1,2],bot_loc[0,3],bot_loc[1,3]]
		with open('dataPWM.csv','ab') as myfile:
            writer=csv.writer(myfile)
            writer.writerow(row)
            #print(rospy.get_time()-codeStartTime-logTime) #Just to see how long this logging takes

def loggerNode():
	global codeStartTime
	rospy.init_node('loggingNode',anonymous=True)
	codeStartTime=rospy.get_time()
    #The torque controller outputs commands at only 10Hz.
    rospy.Subscriber('pwmCmd',PwmInput,callbackPWM)
    rospy.Subscriber('encoderData', encoderData, callbackEnc)

    #VICON data subscriber. Change the name to the required name here. The bot this is running on is to be placed last.
	rospy.Subscriber("/vicon/vijeth_1/vijeth_1", TransformStamped, callbackVICON, 1)
	rospy.Subscriber("/vicon/vijeth_2/vijeth_2", TransformStamped, callbackVICON, 2)
	rospy.Subscriber("/vicon/vijeth_3/vijeth_3", TransformStamped, callbackVICON, 3)
	rospy.Subscriber("/vicon/vijeth_0/vijeth_0", TransformStamped, callbackVICON, 0)

	rospy.spin()
if __name__ == '__main__':
    try:
        loggerNode()
    except rospy.ROSInterruptException:
        pass
