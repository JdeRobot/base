import threading, time
from datetime import datetime

time_cycle = 80;

class MiThread(threading.Thread):  
    def __init__(self, sensor):
        self.sensor = sensor
        self._continue = True
        threading.Thread.__init__(self)  

    def stop(self):
        self._continue = False 
    def run(self):  
        while(self.sensor.running):
            
            start_time = datetime.now()

            self.sensor.update()
            
            finish_Time = datetime.now()
            
            dt = finish_Time - start_time
            ms=(dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            if(ms < time_cycle):
                time.sleep((time_cycle-ms) / 1000.0);
