"""This module provides a class wrapping an iRobot Create."""

from serial import Serial
from struct import pack
from struct import unpack
from time import sleep
from threading import Thread
from threading import Lock
from datetime import datetime
from math import pi
class Monitor(Thread):
	def __init__(self, watchdog, packetRef, create, read, sendAll, update):
		Thread.__init__(self)
		self.watchdog = watchdog
		self.packetRef = packetRef
		self.create = create
		self.read = read
		self.sendAll = sendAll
		self.update = update

		def mask(num):
			return (
				num >> 7 & 1,
				num >> 6 & 1,
				num >> 5 & 1,
				num >> 4 & 1,
				num >> 3 & 1,
				num >> 2 & 1,
				num >> 1 & 1,
				num & 1
				)

		def unsigned(num):
			return (num,)

		def signed(num):
			if (num >> 15):
				return (-1*(2**16-num),)
			return (num,)

		def signed8(num):
			if (num >> 7):
				return (-1*(2**8-num),)
			return (num,)

		self.packets = (
			(7,1,mask,("na","na","na","wheeldropCaster","wheeldropLeft","wheeldropRight","bumpLeft","bumpRight")),
			(8,1,unsigned,("wall",)),
			(9,1,unsigned,("cliffLeft",)),
			(10,1,unsigned,("cliffFronLeft",)),
			(11,1,unsigned,("cliffFrontRight",)),
			(12,1,unsigned,("cliffRight",)),
			(13,1,unsigned,("virtualWall",)),
			(17,1,unsigned,("infraredByte",)),
			(18,1,mask,("na","na","na","na","na","advance","na","play")),
			(19,2,signed,("distance",)),
			(20,2,signed,("angle",)),
			(21,1,unsigned,("chargingState",)),
			(22,2,unsigned,("voltage",)),
			(23,2,signed,("current",)),
			(24,1,signed8,("batteryTemperature",)),
			(25,2,unsigned,("batteryCharge",)),
			(26,2,unsigned,("batteryCapacity",)),
			(27,2,unsigned,("wallSignal",)),
			(28,2,unsigned,("cliffLeftSignal",)),
			(29,2,unsigned,("cliffFrontLeftSignal",)),
			(30,2,unsigned,("cliffFrontRightSignal",)),
			(31,2,unsigned,("cliffRightSignal",)),
			(34,1,mask,("na","na","na","na","na","na","homeBase","internalCharger")),
			(36,1,unsigned,("songNumber",)),
			(37,1,unsigned,("songPlaying",)),
			)

	def run(self):
		while(len(self.watchdog) == 0):
			then = datetime.now()
			self.sendAll() #send queued commands

			self.create.send(149,len(self.packets),*[i[0] for i in self.packets]) #read sensor packets
			self.sendAll() 

			bytes = self.read(sum([i[1] for i in self.packets]))

			try:
				bytes = unpack('B'*sum([i[1] for i in self.packets]),bytes)
				data = {}

				offset = 0
				d = None
				theta = None
				for packet in self.packets:
					if (packet[1] == 1):
						results = packet[2](bytes[offset])
					else:
						results = packet[2]((bytes[offset] << 8) | (bytes[offset+1]))

					index = 0
					for result in results:
						data[packet[3][index]] = result
						if (packet[3][index] == "distance"):
							d = result
						elif (packet[3][index] == "angle"):
							theta = result
						index = index + 1

					offset = offset + packet[1]

				if d is not None and theta is not None:
					self.create.updateEncoders(d, theta)
				data['encoderLeft'], data['encoderRight'] = self.create.getEncoders()

				if (len(self.packetRef)):
					self.packetRef[0] = data #atomic
				else:
					self.packetRef.append(data) #atomic
			except Exception as e:
				print "bad data"
				print e

			self.update()

			now = datetime.now()
			elapsed = now - then
			elapsed = elapsed.seconds*1000. + elapsed.microseconds/1000.
			if (elapsed < self.create.period*1000.):
				sleep((self.create.period*1000. - elapsed)/1000.)

class Create:
	"""Wrapper class for the iRobot Create"""

	def __init__(self, tty="/dev/ttyUSB0"):
		"""constructor for the Create, takes in a single argument: the serial port"""

		self.timeout = 5
		self.period = .07
		self.runRef = []
		self.packetRef = []
		self.queueLock = Lock()
		self.queue = []
		self.encoderLock = Lock()
		self.__encoder_left = 0
		self.__encoder_right = 0
		self.port = Serial(tty, 57600, timeout= self.timeout)
		self.portLock = Lock()
		self.update = lambda : ''
		self.reset()

	def start(self):
		"""Start the iCreate after initialization or reset."""
		
		self.__sendNow(128,128,132,140,1,5,64,16,69,16,74,16,72,40,69,60,141,1)
		sleep(1)
		self.send(139,10,0,255)

		monitor = Monitor(self.runRef, self.packetRef, self, self.__read, self.__sendAll, self.update)
		monitor.start()
		sleep(1.5)

	def stop(self):
		"""Stop the iCreate. Must be called before deletion of the iCreate object."""
		self.runRef.append('quit')
		sleep(1)
		rh,rl = self.__convert(0)
		lh,ll = self.__convert(0)
		self.__sendNow(145,rh,rl,lh,ll) #emergency brake
		self.__sendNow(139,0,0,255)

	def reset(self):
		"""Reset the iCreate."""
		self.runRef.append('quit')
		self.runRef = []
		sleep(1)

		self.port.flushOutput()
		self.port.flushInput()
		self.__sendNow(128,7)
		self.__read(128,True)
		#should have processed initialization
		self.port.flushInput() #ignore rest of input

	def __convert(self, num):
		return self.__highLow(self.__twos(num))

	def __highLow(self, num):
		return num >> 8, num & 0xFF

	def __twos(self, num, bits=16):
		if (num >=0):
			return num
		return 2**bits+num

	def send(self, *opcodes):
		self.queueLock.acquire()

		def lmbda():
			self.__sendNow(*opcodes)

		self.queue.append(lmbda)

		self.queueLock.release()

	def __sendNow(self, *opcodes):
		self.portLock.acquire()
		format = "B"*len(opcodes)	
		data = pack(format, *opcodes)
		self.port.write(data)
		self.portLock.release()

	def __sendAll(self):
		self.queueLock.acquire()
		
		self.__read(self.port.inWaiting())
		for send in self.queue:
			send()
		self.queue = []

		self.queueLock.release()
	
	def __read(self,num,block=False):
		self.portLock.acquire()
		if (block):
			self.port.timeout = None
		bytes = self.port.read(num)
		if (block):
			self.port.timeout = self.timeout
		self.portLock.release()
		return bytes

	def __getattr__(self,name):
		if (len(self.packetRef)):
			if (name in self.packetRef[0]):
				return self.packetRef[0][name]
		raise AttributeError, "no %s attribute" % (name)

	def updateEncoders(self,d,theta):
		theta = theta/180.0*pi
		b = 258.0
		self.encoderLock.acquire()
		if theta == 0:
			self.__encoder_left += d / 32.0
			self.__encoder_right += d / 32.0
		else:
			r = d / theta - b / 2
			self.__encoder_left += (r * theta) / 32.0
			self.__encoder_right += ((r + b) * theta) / 32.0
		self.encoderLock.release()
	
	def getEncoders(self):
		#print('get encoders')
		self.encoderLock.acquire()
		l = self.__encoder_left
		r = self.__encoder_right
		self.encoderLock.release()
		return (l, r)

	def clear(self):
		self.encoderLock.acquire()
		self.__encoder_left = 0
		self.__encoder_right = 0
		self.encoderLock.release()

	def __del__(self):
		self.port.close()


	def leds(self,play,advance,color,intensity):
		"""Controls the LEDs. Parameters are play: boolean (play on/off), advance: boolean (advance on/off), color: 0-255 (how much red in the power light), and intensity: 0-255 (how bright should the power light be."""
		if (play):
			play = 1
		else:
			play = 0
		if (advance):
			advance = 1
		else:
			advance = 0
		bits = play << 1 | advance << 3
		self.send(139,bits,color,intensity)

	def storeSong(self, num, *song):
		"""Store a song. First parameter is the song number, the remaming arguments are taken to be of the form: note, duration, note, duration, etc. See page 12 of the iRobot Create Open Interface Manual for numerical note definitions. Duration is interpreted as duration*1/64th of a second."""
		if (len(song) > 32):
			song = song[:31]
		self.send(140,num,len(song)/2,*song)

	def playSong(self,num):
		"""Plays a song. Takes one parameter, the song number."""
		#if (not  self.packetRef[0]['songPlaying']):
		self.send(141,num)

	def diffDrive(self,left,right):
		"""Drive the iCreate like a tank (i.e. left throttle, right throttle). Takes two parameters: left and right throttle. Each can be between -15.625 and 15.625 representing rad/s."""
		left *= 32.0
		right *= 32.0
		if (left < -500): 
			left = -500
		if (right < -500):
			right = -500
		if (left > 500):
			left = 500
		if (right > 500):
			right = 500

		lh,ll = self.__convert(int(left))
		rh,rl = self.__convert(int(right))
		self.send(145,rh,rl,lh,ll)
