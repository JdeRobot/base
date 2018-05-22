import os,sys
import shlex,subprocess
from subprocess import PIPE,Popen
from datetime import datetime
import threading

class processVideo():

	def __init__(self,URL,liveBroadcast, outdir, fps, ydlFormat):

		self.url = URL
		self.liveBroadcast = liveBroadcast
		self.endDownloading = False
		self.getImages = True
		self.outdir = outdir
		self.fps = fps
		self.fmt = ydlFormat
		self.setFileList()

	def setFileList(self):

		if self.liveBroadcast:
			command = shlex.split('youtube-dl -f 92 -g ' + self.url)
		else:
			command = shlex.split('youtube-dl -f '+ self.fmt +' -g ' + self.url)
		print command
		process= Popen(command,stdout=PIPE,stderr=PIPE)
		stdout, sterr = process.communicate()
		if process.poll() is None:
			process.terminate()
		self.fileList = stdout

	def getImage(self,init_time,end_time):

		init_time = datetime.strftime(init_time,'%H:%M:%S')
		end_time = datetime.strftime(end_time,'%H:%M:%S')
		command = shlex.split("ffmpeg -i "+ self.outdir +"output.ts -start_number 0 -vf fps="+ self.fps + " -ss " + init_time + " -to " + end_time + " -f image2 -updatefirst 1 "+ self.outdir +"temp.jpg")
		print command
		process= Popen(command,stdout=PIPE,stderr=PIPE)
		code = process.poll()
		while (code == None):
			code = process.poll()
		if process.poll() is None:
			print 'a'
			process.terminate()
	
	def downloadVideo(self):
		try:
			if os.path.isfile(self.outdir + 'output.ts'):
				os.remove(self.outdir + "output.ts")
				
			data = self.fileList.splitlines()
			data= data[0].decode('utf-8')
			if self.liveBroadcast:
				command=shlex.split('ffmpeg -i ' + data + ' -c copy ' + self.outdir +'output.ts')
			else:
				command=shlex.split('ffmpeg -r '+ self.fps +' -i ' + data + ' -r '+ self.fps +' ' + self.outdir + 'output.ts')
			print command
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
		if os.path.isfile(self.outdir + 'temp.jpg'):
			os.rename(self.outdir + "temp.jpg",self.outdir + "image.jpg")

	def getVideoDuration(self):
		try:
			if os.path.isfile(self.outdir + 'output.ts'):
				command = shlex.split('ffprobe -show_entries format=duration -sexagesimal '+ self.outdir + 'output.ts') 
				print command
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
