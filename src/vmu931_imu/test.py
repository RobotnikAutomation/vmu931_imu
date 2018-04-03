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
d.enableStreamingQuaternions()
d.enableStreamingHeading()
d.enableStreamingEulerAngles()
d.enableStreamingMagnetometers()
d.selfTest()
for x in range(0,20):
	print "-------------------------------"
	d.readOneTime()
	d.printAllValue()
