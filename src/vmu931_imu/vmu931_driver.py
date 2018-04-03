#!/usr/bin/env python

import dato
import serial
from time import sleep
import sys
import rospy

ACCELEROMETERS = "a"
GYROSCOPES = "g"
MAGNETOMETERS = "c"
QUATERNIONS = "q"
EULER_ANGLES = "e"
HEADING = "h"
SELF_TEST = "t"
CALIBRATION = "l"
STATUS = "s"

GYROSCOPES_250SPS = "0"
GYROSCOPES_500SPS = "1"
GYROSCOPES_250SPS = "2"
GYROSCOPES_500SPS = "3"
ACCELEROMETERS_2G = "4"
ACCELEROMETERS_4G = "5"
ACCELEROMETERS_8G = "6"
ACCELEROMETERS_16G = "7"

class vmu931():
	def __init__ (self, port, baudrate = 9600):
		self._port = port
		self._baudrate = baudrate
		self.connected = False
		self.status = dato.Status()
		self.value = {"Accelerometers":dato.Dato(), "Gyroscopes":dato.Dato(),
                "Magnetometers":dato.Dato(), "Quaternions":dato.Dato(), 
                "Euler":dato.Dato(), "Heading":dato.Dato(), "Text": dato.Dato()}
		
		self.serial_dev = serial.Serial()
		self.serial_dev.port = self._port
		self.serial_dev.baudrate = self._baudrate
		self.serial_dev.bytesize=serial.EIGHTBITS
	
	def isConnected (self):
		
		return self.connected
	
	def setup(self):
		
		if not self.serial_dev.isOpen():
			try:
				self.serial_dev.open()
				self.serial_dev.flushInput()
			except serial.serialutil.SerialException, e:
				rospy.logerr('vmu931:setup: %s',e)
				self.connected = False
				return False
		
		rospy.loginfo('vmu931:setup: port %s:%d opened successfully',self.serial_dev.port, self.serial_dev.baudrate)
		self.connected = True
		return True
	
	def close(self):
		if self.serial_dev.isOpen():
			self.serial_dev.close()
		self.connected = False
		
	def readByte(self, num):
		bytesReader = []
		for i in range(1,num):
			bytesReader.append(self.serial_dev.read(1))
		return bytesReader
		
	
	'''
		Reads and process the message once
		@returns code, processed message
			0 -> OK
			-1 -> Not Connected
			-2 -> Nothing to read
			-3 -> Unknown data type
			-4 -> Error in message
			-5 -> I/O error
	'''
	def readOneTime(self):
		if not self.connected:
			return -1,''
		
		try:
			input_bytes = self.serial_dev.inWaiting()
			#rospy.loginfo('readOneTime 1')
			if input_bytes<=4:
				#rospy.logwarn('readOneTime bytes = %d', input_bytes)
				return -2,''
			buffer_input = self.serial_dev.read(input_bytes)
		except IOError, e:
			rospy.logwarn('vmu931_driver::readOneTime: %s', e)
			return -5,''
		dataTypeMessage = False
		if (ord(buffer_input[0])==2 and ord(buffer_input[len(buffer_input)-1])== 3): #string message
			dataTypeMessage = False
		elif (ord(buffer_input[0])==1 and ord(buffer_input[len(buffer_input)-1])==4): #data message
			dataTypeMessage = True
		else:
			rospy.logwarn('vmu931_driver::readOneTime: unknown data type')
			'''sizeMessage = ord(buffer_input[1])-4
			typeMessage = buffer_input[2]
			dataMessage = buffer_input[3:3+sizeMessage]
			rospy.logwarn('vmu931_driver::readOneTime: msg =(%s) %s', type(dataMessage),dataMessage)'''
			return -3,''
		sizeMessage = ord(buffer_input[1])-4
		typeMessage = buffer_input[2]
		dataMessage = buffer_input[3:3+sizeMessage]
		#rospy.loginfo('readOneTime: type %s, size = %d', typeMessage, sizeMessage)
		#print len(dataMessage)
		#print len(dataMessage[0:4])
		if (dataTypeMessage):
			if (typeMessage==ACCELEROMETERS):
				if (sizeMessage != 16):
					return -4,''
				self.value["Accelerometers"].setTimestam(dataMessage[0:4])
				self.value["Accelerometers"].setX(dataMessage[4:8])
				self.value["Accelerometers"].setY(dataMessage[8:12])
				self.value["Accelerometers"].setZ(dataMessage[12:16])
				#rospy.loginfo("readOneTime: accelerometers received!")
			elif (typeMessage==GYROSCOPES):
				if (sizeMessage != 16):
					return -4,''
				self.value["Gyroscopes"].setTimestam(dataMessage[0:4])
				self.value["Gyroscopes"].setX(dataMessage[4:8])
				self.value["Gyroscopes"].setY(dataMessage[8:12])
				self.value["Gyroscopes"].setZ(dataMessage[12:16])
				#rospy.loginfo("readOneTime: gyro received!")
			elif (typeMessage==MAGNETOMETERS):
				if (sizeMessage != 16):
					return -4,''
				self.value["Magnetometers"].setTimestam(dataMessage[0:4])
				self.value["Magnetometers"].setX(dataMessage[4:8])
				self.value["Magnetometers"].setY(dataMessage[8:12])
				self.value["Magnetometers"].setZ(dataMessage[12:16])
				#rospy.loginfo("readOneTime: magnetometer received!")
			elif (typeMessage==EULER_ANGLES):
				if (sizeMessage != 16):
					return -4,''
				self.value["Euler"].setTimestam(dataMessage[0:4])
				self.value["Euler"].setX(dataMessage[4:8])
				self.value["Euler"].setY(dataMessage[8:12])
				self.value["Euler"].setZ(dataMessage[12:16])
				#rospy.loginfo("readOneTime: euler received!")
			elif (typeMessage==QUATERNIONS):
				if (sizeMessage != 20):
					return -4,''
				self.value["Quaternions"].setTimestam(dataMessage[0:4])
				self.value["Quaternions"].setW(dataMessage[4:8])
				self.value["Quaternions"].setX(dataMessage[8:12])
				self.value["Quaternions"].setY(dataMessage[12:16])
				self.value["Quaternions"].setZ(dataMessage[16:20])
				#rospy.loginfo("readOneTime: quaternions received!")
			elif (typeMessage==HEADING):
				if (sizeMessage != 8):
					return -4,''
				self.value["Heading"].setTimestam(dataMessage[0:4])
				self.value["Heading"].setHeading(dataMessage[4:8])
				#rospy.loginfo("readOneTime: heading received!")
			elif (typeMessage==STATUS):
				if (sizeMessage != 7):
					return -4,''
				#rospy.loginfo("readOneTime: status received!")
				self.status.sensorStatus(dataMessage[0])
				self.status.sensorResolution(dataMessage[1])
				self.status.outputRate(dataMessage[2])
				self.status.streaming(dataMessage[3:7])
			else:
				rospy.logerr("vmu931_driver::readOneTime: unknown type %s received!", typeMessage)
				return -4,''
			return 0,typeMessage
		else:
			rospy.loginfo("vmu931_driver::readOneTime: Text from device: {}".format(dataMessage))
			self.value["Text"].setMsg(dataMessage)
			return 0,''
			
	
	def streamingAccelerometers (self, value):
		if (value and not self.status.streamingAcc) or (not value and self.status.streamingAcc):
			self.sendCommand (ACCELEROMETERS)
	
	def streamingGyroscopes (self, value):
		if (value and not self.status.streamingGyro) or (not value and self.status.streamingGyro):
			self.sendCommand (GYROSCOPES)
	
	def streamingMagnetometers (self, value):
		if (value and not self.status.streamingMag) or (not value and self.status.streamingMag):
			self.sendCommand (MAGNETOMETERS)
	
	def streamingQuaternions (self, value):
		if (value and not self.status.streamingQuart) or (not value and self.status.streamingQuart):
			self.sendCommand (QUATERNIONS)

	def streamingEulerAngles (self, value):
		if (value and not self.status.streamingEuler) or (not value and self.status.streamingEuler):
			self.sendCommand (EULER_ANGLES)
	
	def streamingHeading (self, value):
		if (value and not self.status.streamingHead) or (not value and self.status.streamingHead):
			self.sendCommand (HEADING)
	
	def calibrate (self):
		self.sendCommand(CALIBRATION)

	def selfTest (self):
		self.sendCommand('t')
	
	def sendCommand(self, mod):
		#self.serial_dev.flushInput()
		if (self.connected):
			mod = "var" + mod
			byte_message = mod.encode("ascii")
			cont=0
			if (mod == "var"+STATUS):
				self.status.notUpdate()
				try:
					self.serial_dev.write(byte_message)
				except serial.SerialException, e:
					rospy.logerr('vmu931_driver::sendCommand: %s',e)
					return False
				
			elif (mod == "var"+CALIBRATION):
				self.value["Text"].notUpdate()
				try:
					self.serial_dev.flushInput()
					self.serial_dev.write(byte_message)
				except serial.SerialException, e:
					rospy.logerr('vmu931_driver::sendCommand: %s',e)
					return False
				
			elif (mod == "var"+SELF_TEST):
				'''print "Test start"
				self.value["Text"] = dato.Dato()
				self.serial_dev.write(byte_message)
				cont = 0
				while (not self.value["Text"].update):
					self.readOneTime()
					cont = cont +1
					print cont
				print self.value["Text"].msg
				cont = 0
				self.value["Text"].notUpdate()
				while (not self.value["Text"].update):
					self.readOneTime()
					cont = cont +1
					print cont
				print self.value["Text"].msg
				'''
				
			else:
				try:
					self.serial_dev.write(byte_message)
				except serial.SerialException, e:
					rospy.logerr('vmu931_driver::sendCommand: %s',e)
					return False
				
		sleep(0.02)
		return True
	
	def getValue (self):
		copyValue = self.value
		for key, val in self.value.iteritems():
			val.notUpdate()
		return copyValue
	
	def printAllValue (self):
		for key, val in self.value.iteritems():
			print (key)
			print "setTimestam = {} w = {} x = {} y = {} z = {} heading ={} msg ={}".format(val.timestamp, val.w, val.x, val.y, val.z, val.heading, val.msg)
		return
	
	def printValue (self, val):
		print "setTimestam = {} w = {} x = {} y = {} z = {} heading ={} msg ={}".format(val.timestamp, val.w, val.x, val.y, val.z, val.heading, val.msg)
		return
	
	def getStatus (self):
		copyValue = self.status
		self.status.notUpdate()
		return copyValue
	
	def printStatus (self):
         print "Status devices: \n mag={} gyro={} acc={} \nResolution: \n gyro={} acc={} \nLow output rate status: {}\nStreaming: \n Head={} Uler={} Mag={} Quart={} Gyro={} Acc={}".format(self.status.magStatus, self.status.gyroStatus, self.status.accStatus, self.status.gyroResolution, self.status.accResolution, self.status.ouptputRate, self.status.streamingHead, self.status.streamingEuler, self.status.streamingMag, self.status.streamingQuart, self.status.streamingGyro, self.status.streamingAcc)
         return
	
	'''
		Reads and updates the current status of the sensor (command s)
	'''
	def readStatus(self):
		if (self.connected):
			#rospy.loginfo('readStatus')
			self.sendCommand(STATUS)
		
	def __del__(self):
		if self.serial_dev.isOpen():
			self.serial_dev.close()
			self.connected = False
