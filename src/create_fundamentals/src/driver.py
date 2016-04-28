#!/usr/bin/python
import roslib
import rospy
from time import sleep
from irobot import Create
from threading import Thread
from math import sin,cos,pi
from datetime import datetime

from create_fundamentals.msg import SensorPacket
from create_fundamentals.srv import *

class CreateDriver:
	def __init__(self):
		port = rospy.get_param('/create_fundamentals/port', "/dev/ttyUSB0")
		self.create = Create(port)
		self.packetPub = rospy.Publisher('sensor_packet', SensorPacket, queue_size=1)
		self.fields = ['wheeldropCaster','wheeldropLeft','wheeldropRight','bumpLeft','bumpRight','wall','cliffLeft','cliffFronLeft','cliffFrontRight','cliffRight','virtualWall','infraredByte','advance','play','encoderLeft','encoderRight','chargingState','voltage','current','batteryTemperature','batteryCharge','batteryCapacity','wallSignal','cliffLeftSignal','cliffFrontLeftSignal','cliffFrontRightSignal','cliffRightSignal','homeBase','internalCharger','songNumber','songPlaying']
		self.create.update = self.sense

	def start(self):
		self.create.start()
		self.then = datetime.now() 

	def stop(self):
		self.create.stop()

	def sense(self):
		packet = SensorPacket()
		for field in self.fields:
			packet.__setattr__(field,self.create.__getattr__(field))
		self.packetPub.publish(packet)

	def leds(self,req):
		print 'leds'
		self.create.leds(req.advance,req.play,req.color,req.intensity)
		return LedsResponse(True)

	# diff drive in rad/s
	def diff_drive(self,req):
		print 'differential drive', req.left, req.right, 'requested'
		self.create.diffDrive(req.left,req.right)
		return DiffDriveResponse(True)

	def reset_encoders(self,req):
		print 'encoder reset'
		self.create.clear()
		return ResetEncodersResponse(True)

	def store_song(self,req):
		print 'request to store', req.song, 'as song', req.number
		print type(req.song)
		self.create.storeSong(req.number, *req.song)
		return StoreSongResponse(True)

	def play_song(self,req):
		print 'request to play song', req.number
		self.create.playSong(req.number)
		return PlaySongResponse(True)
		
if __name__ == '__main__':
	node = rospy.init_node('create')
	driver = CreateDriver()
	
	rospy.Service('reset_encoders', ResetEncoders, driver.reset_encoders)
	rospy.Service('leds', Leds, driver.leds)
	rospy.Service('store_song', StoreSong, driver.store_song)
	rospy.Service('play_song', PlaySong, driver.play_song)
	rospy.Service('diff_drive', DiffDrive, driver.diff_drive)

	sleep(1)
	driver.start()
	sleep(1)

	rospy.spin()
	driver.stop()
