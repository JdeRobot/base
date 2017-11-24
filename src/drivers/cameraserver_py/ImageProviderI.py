import jderobot
import threading
import os
from threadImage import ThreadImage #camera isn a threadImage
import numpy as np

class WorkQueue(threading.Thread):
		def __init__(self):
				self.callbacks = []
				threading.Thread.__init__(self)
				

		def run(self):
				if not len(self.callbacks) == 0:
						self.callbacks[0].execute()
						del self.callbacks[0]
			
		def add(self, job):
				self.callbacks.append(job)
				self.run()
				

class Job(object):
	
	def __init__(self,cb,format, camera):
		self.cb = cb
		self.format = format
		self.camera = camera
		self.imageData = jderobot.ImageData()
	
	def execute(self):
		if not self.getData():
			print "No data"
			return
		self.cb.ice_response(self.imageData)

	def getData(self):
		img = self.camera.get_image()
		self.imageData = jderobot.ImageData()
		self.imageData.description = jderobot.ImageDescription()
		self.imageData.description.width = img.shape[1]
		self.imageData.description.height = img.shape[0]
		self.imageData.description.format = 'RGB8'
		self.pixelData = np.getbuffer(img)

		self.imageData.pixelData = self.pixelData

		return True

class ImageProviderI(jderobot.Camera):

	def __init__(self,workQueue, camera):
		self.workQueue = workQueue
		self.camera = camera

	def getCameraDescription(self, current=None):
		return jderobot.CameraDescription()

	def setCameraDescription(self, description):
		return 0

	def startCameraStreaming(self, current=None):
		return ''
	
	def stopCameraStreaming(self,current=None):
		print('')

	def reset(self):
		print('---')

	def getImageDescription(self,current=None):

		self.imageData = jderobot.ImageDescription() 
		img = self.camera.get_image()
		self.imageData.width = img.shape[1]
		self.imageData.height = img.shape[0]
		self.format = 'RGB'    
		return self.imageData

	def getImageData_async(self,cb,formato,curren=None):
		job = Job(cb,formato, self.camera)
		return self.workQueue.add(job)