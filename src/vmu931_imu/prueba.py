#!/usr/bin/env python

import vmu931_driver as vmu
import dato

def printValue (val):
	print "setTimestam = {} w = {} x = {} y = {} z = {} heading ={} msg ={}".format(val.timestamp, val.w, val.x, val.y, val.z, val.heading, val.msg)
	
d = vmu.vmu931("/dev/ttyUSB_VMU931")
d.connectToVMU931()
d.isConnected()
d.enableStreamingAccelerometers()
d.enableStreamingGyroscopes()
d.enableStreamingMagnetometers()
d.enableStreamingQuaternions()
d.enableStreamingEulerAngles()
d.enableStreamingHeading()
for x in range(0,1):
	print "-------------------------------"
	d.readOneTime()
	d.printAllValue()
	
