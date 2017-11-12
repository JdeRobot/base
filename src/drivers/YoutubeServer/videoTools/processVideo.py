import os
import shlex,subprocess
from subprocess import PIPE,Popen
from datetime import datetime
class processVideo():

	def setURL(self,URL):
		self.url = URL
	
	def setliveBroadcast(self,liveBroadcast):
		self.liveBroadcast = liveBroadcast

	def setFileList(self):
		#get video download url
		if liveBroadcast:
			command = shlex.split('youtube-dl -f 92 -g ' + self.url)
		else:
			command = shlex.split('youtube-dl -f 18 -g ' + self.url)

		process= Popen(command ,stdout=PIPE,stderr=PIPE)
		self.fileList=process.stdout.read()
		process.stdout.close()

	def getImage(self,init_time,end_time):
		#extract frame to frame of video
		init_time = datetime.strftime(init_time,'%H:%M:%S')
		end_time = datetime.strftime(end_time,'%H:%M:%S')
		command =shlex.split("ffmpeg -i output.ts -start_number 0 -vf fps=25 -ss " + init_time + " -to " + end_time + " -f image2 -updatefirst 1 temp.jpg")
		process= Popen(command,stdout=PIPE,stderr=PIPE)
	
	def downloadVideo(self):
			data = self.fileList.splitlines()
			data= data[0].decode('utf-8')
		if liveBroadcast:
			command=shlex.split('ffmpeg -i ' + data + ' -c copy output.ts')
		else:
			command=shlex.split('ffmpeg -r 24 -i ' + data + ' -r 24 output.ts')
		process= Popen(command,stdout=PIPE,stderr=PIPE)

	def changeName(self):
		if os.path.isfile('./temp.jpg'):
			os.rename("temp.jpg","image.jpg")

	def getVideoDuration(self):
		#the video is downloaded by parts, here get the duration each part.
		command = shlex.split('ffprobe -show_entries format=duration -sexagesimal output.ts') 
		process = Popen(command ,stdout=PIPE,stderr=PIPE)
		time = process.stdout.read()
		time = time.decode('utf-8')
		time = time.split('\n')[1].split('=')[1]
		time = datetime.strptime(time.split('.')[0],'%H:%M:%S')
		process.stdout.close()
		return time