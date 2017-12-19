import threading,time
import os
from datetime import timedelta,datetime

class ThreadImage(threading.Thread):
	
	def __init__(self,dataFlow,liveBroadcast, outdir):
		self.dataFlow = dataFlow
		self.format_time = '%H:%M:%S'
		self.init_time = datetime.strptime('00:00:00',self.format_time)
		self.liveBroadcast = liveBroadcast
		self.outdir = outdir
		threading.Thread.__init__(self)

	def run(self):
		print "Getting Images..."
		_run = True
		while not os.path.isfile(self.outdir + 'output.ts'):
			time.sleep(1)
			
		while(_run == True):
			self.end_time = self.dataFlow.getVideoDuration()

			if (self.end_time == self.init_time) and (self.dataFlow.endDownloading):
				_run  = False
			if not(self.end_time == self.init_time):
				self.dataFlow.getImage(self.init_time,self.end_time)
			self.init_time = self.end_time

class ThreadDownload(threading.Thread):
	
	def __init__(self,dataFlow):
		self.dataFlow = dataFlow
		threading.Thread.__init__(self)

	def run(self):
		print("*** Download Started ***")
		self.dataFlow.downloadVideo()

class ThreadChangeName(threading.Thread):
	
	def __init__(self,dataFlow,lock):

		self.dataFlow = dataFlow
		self.lock = lock
		threading.Thread.__init__(self)

	def run(self):
		while(self.dataFlow.getImages):
			self.lock.acquire()
			self.dataFlow.changeName()
			self.lock.release()
