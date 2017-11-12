import threading,time
import os
from datetime import timedelta,datetime


class ThreadImage(threading.Thread):
	#thread that extract images of video
	def __init__(self,dataFlow):
		self.dataFlow = dataFlow
		self.format_time = '%H:%M:%S'
		self.init_time =datetime.strptime('00:00:00',self.format_time)
		threading.Thread.__init__(self)

	def run(self):
		while not os.path.isfile('./output.ts'):
			time.sleep(1)
			
		while(True):
			self.end_time = self.dataFlow.getVideoDuration()
			self.dataFlow.getImage(self.init_time,self.end_time)
			#if video download finish, restart the proccess of extract images since init of video
			if self.init_time != self.end_time:
				self.init_time = self.end_time
			else
				self.init_time = datetime.strptime('00:00:00',self.format_time)

class ThreadDownload(threading.Thread):
	
	def __init__(self,dataFlow):
		self.dataFlow = dataFlow
		threading.Thread.__init__(self)

	def run(self):
		print "Download Started"
		self.dataFlow.downloadVideo()

class ThreadChangeName(threading.Thread):
	#thread that manage doble buffer
	def __init__(self,dataFlow):
		self.dataFlow = dataFlow
		threading.Thread.__init__(self)

	def run(self):
		while(True):
			self.dataFlow.changeName()
