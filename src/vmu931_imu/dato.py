#Modulo dato.py
#!/usr/bin/env python

import struct

class Dato():
	def __init__(self):
		self.timestamp = 0
		self.w = -1.
		self.x = -1.
		self.y = -1.
		self.z = -1.
		self.heading = -1.
		self.msg = ""
		self.update = False
		
	def setTimestam (self, listByte):
		self.timestamp = struct.unpack('>I', listByte)[0]
		self.update = True
	
	def setX (self, listByte):
		self.x = struct.unpack('>f', listByte)[0]
		self.update = True
		
	def setY (self, listByte):
		self.y = struct.unpack('>f', listByte)[0]
		self.update = True
		
	def setZ (self, listByte):
		self.z = struct.unpack('>f', listByte)[0]
		self.update = True
		
	def setW (self, listByte):
		self.w = struct.unpack('>f', listByte)[0]
		self.update = True
	
	def setHeading (self, listByte):
		self.heading = struct.unpack('>f', listByte)[0]
		self.update = True
	
	def setMsg (self, listByte):
		self.msg = listByte
		self.update = True
		
	def notUpdate (self):
		self.update = False
	
	
class Status():
	def __init__ (self):
		self.magStatus = False
		self.gyroStatus = False
		self.accStatus = False
		self.gyroResolution = 0
		self.accResolution = 0
		self.ouptputRate = 0
		self.streamingHead = False
		self.streamingEuler = False
		self.streamingMag = False
		self.streamingQuart = False
		self.streamingGyro = False
		self.streamingAcc = False
		self.update = False
	
	def sensorStatus (self, byte):
		value = ord(byte)
		self.accStatus = ((value & 1)!= 0)
		self.gyroStatus = ((value & 2)!= 0)
		self.magStatus = ((value&4)!=0)
		self.update = True
		
	def sensorResolution (self, byte):
		value = ord(byte)
		if (value&1)!=0:
			self.accResolution = 2
		elif(value&2)!=0:
			self.accResolution = 4
		elif(value&4)!=0:
			self.accResolution = 8
		elif(value&8)!=0:
			self.accResolution = 16
		else:
			self.accResolution = 0
			
		if (value&16)!=0:
			self.gyroResolution = 250
		elif(value&32)!=0:
			self.gyroResolution = 500
		elif(value&64)!=0:
			self.gyroResolution = 1000
		elif(value&128)!=0:
			self.gyroResolution = 2000
		else:
			self.gyroResolution = 0
		self.update = True
	
	def outputRate (self, byte):
		value = ord(byte)
		if (value&1)!=0:
			self.ouptputRate = 200
		else:
			self.ouptputRate = 1000
		self.update = True
			
	def streaming (self, listByte):
		value =  struct.unpack('>I', listByte)[0]
		self.streamingHead = (value & 64)!= 0
		self.streamingEuler = (value & 16)!= 0
		self.streamingMag = (value & 8)!= 0
		self.streamingQuart = (value & 4)!= 0
		self.streamingGyro = (value & 2)!= 0
		self.streamingAcc = (value & 1)!= 0
		self.update = True
		
	def notUpdate (self):
		self.update = False