import jderobot
import threading
from PIL import Image
import os

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
	
	def __init__(self,cb,formato):
		self.cb = cb
		self.format = formato
		self.imageDescription = jderobot.ImageData()
	
	def execute(self):
		if not self.getData():
			print "No data"
			return
		self.cb.ice_response(self.imageDescription)

	def getData(self):
		if os.path.isfile('./image.jpg'):
			self.imageDescription = jderobot.ImageData()
			self.imageDescription.description = ImageProviderI.getImageDescription(self)
			self.im = Image.open('./image.jpg','r')
			self.im = self.im.convert('RGB')
			self.imRGB = list(self.im.getdata())
			self.pixelData = []
			for pixeList in self.imRGB:
				for pixel in pixeList:
					self.pixelData.append(pixel)
			self.imageDescription.pixelData = self.pixelData
			return True
		else:
			return False

class ImageProviderI(jderobot.Camera):

  def __init__(self,workQueue):
    self.workQueue = workQueue

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
    if os.path.isfile('./image.jpg'):
      self.image= Image.open('./image.jpg')
      self.imageData.width = self.image.width
      self.imageData.height = self.image.height
      self.format = 'RGB'
      return self.imageData

  def getImageData_async(self,cb,formato,curren=None):
    job = Job(cb,formato)
    return self.workQueue.add(job)
