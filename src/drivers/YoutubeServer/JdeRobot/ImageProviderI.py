import jderobot
import threading
from PIL import Image
import os,sys

class WorkQueue(threading.Thread):
		def __init__(self,lock):
				self.callbacks = []
				self.lock = lock
				threading.Thread.__init__(self)
				

		def run(self):
				if not len(self.callbacks) == 0:
						self.callbacks[0].execute()
						del self.callbacks[0]
			
		def add(self, job):
				self.callbacks.append(job)
				self.run()
				

class Job(object):
	
	def __init__(self,cb,formato,lock, outdir):
		self.cb = cb
		self.format = formato
		self.imageDescription = jderobot.ImageData()
		self.lock = lock
		self.outdir = outdir
	
	def execute(self):
		if not self.getData():
			print("No data")
			#self.cb.ice_exception(jderobot.Image.DataNotExistException())
			return
		self.cb.ice_response(self.imageDescription)

	def getData(self):

		self.imageDescription = jderobot.ImageData()
		self.lock.acquire()
		self.imageP = ImageProviderI(self, self.outdir)
		self.imageDescription.description = self.imageP.getImageDescription()
		if os.path.isfile(self.outdir + 'image.jpg'):
			try:
				self.im = Image.open(self.outdir + 'image.jpg','r')
				self.lock.release()
				self.im = self.im.convert('RGB')
				self.imRGB = list(self.im.getdata())
				self.pixelData = []
				self.im.close()
				for pixeList in self.imRGB:
					for pixel in pixeList:
						self.pixelData.append(pixel)
				self.imageDescription.pixelData = self.pixelData
				return True
			except:
				self.getData()
		else:
			self.lock.release()
			return False
			
			
			

class ImageProviderI(jderobot.Camera):

	def __init__(self,workQueue, outdir):
		self.workQueue = workQueue
		self.outdir = outdir

	def getCameraDescription(self):
		return 0

	def setCameraDescription(description):
		return 0

	def startCameraStreaming(self):
		return ''
	
	def stopCameraStreaming(self,current=None):
		print('')

	def reset(self):
		print('---')

	def getImageDescription(self,current=None):

		self.imageData = jderobot.ImageDescription() 
		if os.path.isfile(self.outdir + 'image.jpg'):
			try:
				self.image= Image.open(self.outdir + 'image.jpg','r')
				self.imageData.width = self.image.width
				self.imageData.height = self.image.height
				self.image.close()
			except:
				self.imageData.width = 640
				self.imageData.height = 360

			self.format = 'RGB'
			return self.imageData

	def getImageData_async(self,cb,formato,curren=None):
		job = Job(cb,formato,self.workQueue.lock, self.outdir)
		return self.workQueue.add(job)
