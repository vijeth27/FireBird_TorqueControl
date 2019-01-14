#!/usr/bin/env python
import rospy
import serial
import time
import struct
from fb5_torque_ctrl.msg import PwmInput
from fb5_torque_ctrl.msg import encoderData
ser=serial.Serial('/dev/ttyUSB0',9600)

ser.flushInput()
ser.flushOutput()
ser.write('8')
def callback(pwmInput):
	ser.write('A') # ~ is \x7E
        ser.write('8')
        rightPWM=pwmInput.rightInput
        leftPWM=pwmInput.leftInput
        ser.write(struct.pack('>B',rightPWM))
        ser.write(struct.pack('>B',leftPWM))

def encoderOut():
	rospy.Subscriber('pwmCmd', PwmInput, callback)
	pub_encoder=rospy.Publisher('encoderData',encoderData,queue_size=10)
	rospy.init_node('encoderOut',anonymous=True)
	rate = rospy.Rate(30) #Since bot sends data at 25hz the publisher will be forced to slow down
	while not rospy.is_shutdown():
		i=0
		while i<10:
			encoder=encoderData()
			encoder.interval=0
			encoder.encoderR=0
			encoder.encoderL=0
			encoder.chksum=0
			startTime=rospy.get_time()
			byte1=ser.read()
                	if byte1 is 'A':
                        	encoderRightH=ser.read()
                        	encoderRightL=ser.read()
                        	encoderLeftH=ser.read()
                        	encoderLeftL=ser.read()
                        	chksum=ser.read()
                        	encoder.encoderR=ord(encoderRightH)*256+ord(encoderRightL)
                        	encoder.encoderL=ord(encoderLeftH)*256+ord(encoderLeftL)
                        	sum=ord(encoderRightH)+ord(encoderRightL)+ord(encoderLeftH)+ord(encoderLeftL)
                        	#Byte2 and byte3 are the higher significant and lower significant bits respectively
                        	encoder.chksum=ord(chksum)==sum%256
        		encoder.interval = rospy.get_time()-startTime
        		#rospy.loginfo(encoder)
        		pub_encoder.publish(encoder)
			rate.sleep()

if __name__ == '__main__':
    try:
        encoderOut()
    except rospy.ROSInterruptException:
        pass
