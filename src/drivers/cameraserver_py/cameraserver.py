import Ice
import sys
import config
import cv2
from ImageProviderI import ImageProviderI,WorkQueue
from threadImage import ThreadImage



if __name__== "__main__":

	try:
		cfg = config.load(sys.argv[1])
		id = Ice.InitializationData()
		ic = Ice.initialize(None, id)

		endpoint = cfg.getProperty("cameraServer.Proxy")
		fps = cfg.getPropertyWithDefault("cameraServer.FrameRate",12)
		uri = cfg.getProperty("cameraServer.Uri")

		camera = cv2.VideoCapture(uri)
		
		workQueue = WorkQueue()
		workQueue.setDaemon(True)

		imageThread = ThreadImage(camera, fps)

		imageThread.start()
		workQueue.start()

		adapter = ic.createObjectAdapterWithEndpoints("youtubeServer",endpoint) #Properties
		object = ImageProviderI(workQueue, imageThread)
		adapter.add(object, Ice.stringToIdentity("cameraA"))
		adapter.activate()
		workQueue.join()
		print endpoint
		ic.waitForShutdown()
	except KeyboardInterrupt:
		imageThread.stop()
		del(camera)
		del(ic)
		sys.exit() 