import os,sys
import shlex,subprocess
from subprocess import PIPE,Popen
from datetime import datetime
import threading

class processVideo():

	def __init__(self,URL,liveBroadcast):

		self.url = URL
		self.liveBroadcast = liveBroadcast
		self.endDownloading = False
		self.getImages = True
		self.setFileList()

	def setFileList(self):

		if self.liveBroadcast:
			command = shlex.split('youtube-dl -f 92 -g ' + self.url)
		else:
			command = shlex.split('youtube-dl -f 18 -g ' + self.url)
		process= Popen(command,stdout=PIPE,stderr=PIPE)
		stdout, sterr = process.communicate()
		if process.poll() is None:
			process.terminate()
		self.fileList = stdout

	def getImage(self,init_time,end_time):

		init_time = datetime.strftime(init_time,'%H:%M:%S')
		end_time = datetime.strftime(end_time,'%H:%M:%S')
		command = shlex.split("ffmpeg -i output.ts -start_number 0 -vf fps=24 -ss " + init_time + " -to " + end_time + " -f image2 -updatefirst 1 temp.jpg")
		process= Popen(command,stdout=PIPE,stderr=PIPE)
		code = process.poll()
		while (code == None):
			code = process.poll()
		if process.poll() is None:
			print 'a'
			process.terminate()
	
	def downloadVideo(self):
		try:
			if os.path.isfile('output.ts'):
				os.remove("./output.ts")
				
			data = self.fileList.splitlines()
			data= data[0].decode('utf-8')
			if self.liveBroadcast:
				command=shlex.split('ffmpeg -i ' + data + ' -c copy output.ts')
			else:
				command=shlex.split('ffmpeg -r 24 -i ' + data + ' -r 24 output.ts')
			process= Popen(command,stdout=PIPE,stderr=PIPE)
			code = process.poll()
			while (code == None):
				code = process.poll()

			print "*** Download finished ***"
			self.endDownloading = True
		except:
			print "Can't download video"
			print sys.exc_info[0]
		finally:
			if process.poll() is None:
				process.terminate()
			
	def changeName(self):
		if os.path.isfile('./temp.jpg'):
			os.rename("temp.jpg","image.jpg")

	def getVideoDuration(self):
		try:
			if os.path.isfile('output.ts'):
				command = shlex.split('ffprobe -show_entries format=duration -sexagesimal output.ts') 
				process = Popen(command ,stdout=PIPE,stderr=PIPE)
				time,stderr = process.communicate()
				time = time.decode('utf-8')
				time = time.split('\n')[1].split('=')[1]
				time = datetime.strptime(time.split('.')[0],'%H:%M:%S')

			else:
				time = datetime.strptime('00:00:01', '%H:%M:%S')
		except:
			time = datetime.strptime('00:00:01', '%H:%M:%S')
		finally:
			if process.poll() is None:
				process.terminate()
		return time
