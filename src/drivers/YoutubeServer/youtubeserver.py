import sys
import JdeRobot
import config
import comm
import Ice
import threading
import os
from PIL import Image
from easyiceconfig import easyiceconfig as EasyIce
from videoTools.processVideo import processVideo
from videoTools.threadFlow import ThreadImage,ThreadDownload,ThreadChangeName
from JdeRobot.ImageProviderI import ImageProviderI,WorkQueue


def cleanup():

	if os.path.isfile( outdir + 'output.ts'):
				os.remove(outdir + "output.ts")
	if os.path.isfile(outdir + 'temp.jpg'):
				os.remove(outdir + "temp.jpg")
	if os.path.isfile(outdir + 'image.jpg'):
				os.remove(outdir + "image.jpg")


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
		outdir = cfg.getProperty("youtubeServer.ImageSrv.OutputDir")
		fps = cfg.getProperty("youtubeServer.ImageSrv.FPS")
		ydlFormat = cfg.getProperty("youtubeServer.ImageSrv.Format")

		if type(fps) is not str:
			fps = str(fps)
		if type(ydlFormat) is not str:
			ydlFormat = str(ydlFormat)


		lock = threading.Lock()
		workQueue = WorkQueue(lock)
		workQueue.setDaemon(True)

		dataFlow = processVideo(URL,liveBroadcast, outdir, fps, ydlFormat)
		
		downloadThread = ThreadDownload(dataFlow)
		imageThread = ThreadImage(dataFlow,liveBroadcast, outdir)
		nameThread = ThreadChangeName(dataFlow,lock)

		downloadThread.daemon = True
		imageThread.daemon = True
		nameThread.daemon = True
		workQueue.daemon = True

		downloadThread.start()
		imageThread.start()
		nameThread.start()
		workQueue.start()

		adapter = ic.createObjectAdapterWithEndpoints("youtubeServer",endpoint) #Properties
		object = ImageProviderI(workQueue, outdir)
		adapter.add(object, Ice.stringToIdentity("youtubeServer"))
		adapter.activate()
		workQueue.join()
		ic.waitForShutdown()
	except KeyboardInterrupt:
		print 'Cleaning up & killing...'
		cleanup()
		sys.exit(0)        
