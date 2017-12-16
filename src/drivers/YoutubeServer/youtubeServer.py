import sys
import JdeRobot
import config
import comm
import Ice
import threading
from PIL import Image
from easyiceconfig import easyiceconfig as EasyIce
from videoTools.processVideo import processVideo
from videoTools.threadFlow import ThreadImage,ThreadDownload,ThreadChangeName
from JdeRobot.ImageProviderI import ImageProviderI,WorkQueue



if __name__== "__main__":

	try:
		cfg = config.load(sys.argv[1])
		jdrc = comm.init(cfg,"youtubeServer")
		ic = jdrc.getIc()
		print(cfg.getProperty("youtubeServer.ImageSrv.URL"))
		endpoint = cfg.getProperty("youtubeServer.ImageSrv.Proxy")
		URL = cfg.getProperty("youtubeServer.ImageSrv.URL")
		liveBroadcast = cfg.getProperty("youtubeServer.ImageSrv.LiveBroadcast")
		print(endpoint)
		lock = threading.Lock()
		workQueue = WorkQueue(lock)
		workQueue.setDaemon(True)

		dataFlow = processVideo(URL,liveBroadcast)
		
		downloadThread = ThreadDownload(dataFlow)
		imageThread = ThreadImage(dataFlow,liveBroadcast)
		nameThread = ThreadChangeName(dataFlow,lock)

		downloadThread.start()
		imageThread.start()
		nameThread.start()
		workQueue.start()

		adapter = ic.createObjectAdapterWithEndpoints("youtubeServer",endpoint) #Properties
		object = ImageProviderI(workQueue)
		adapter.add(object, Ice.stringToIdentity("youtubeServer"))
		adapter.activate()
		workQueue.join()
		print('esperar a cierre')
		ic.waitForShutdown()
	except KeyboardInterrupt:
		sys.exit()        
