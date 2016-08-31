from __future__ import print_function
import sys, traceback, Ice
import easyiceconfig as EasyIce
import jderobot 
import threading  
import copy

class Sensor:
    def __init__(self):
        try:
            ic = EasyIce.initialize(sys.argv)
            proxy = ic.getPropertyWithDefault("emSensorView.emSensor.Proxy","emSensor:default -h localhost -p 9090")
            try:
                    base = ic.stringToProxy(proxy)
            except:
                print(("Failed to use defined 'emSensorView.emSensor.Proxy',"
                       "using 'emSensor:default -h localhost -p 9090' as fallback"),
                        file=sys.stderr)
                base = ic.stringToProxy("emSensor:default -h localhost -p 9090")
                
            self.emSensorProxy = jderobot.EMSensorPrx.checkedCast(base)
            if not self.emSensorProxy:
                raise RuntimeError("Invalid proxy")
        
            self.sensorData = self.emSensorProxy.getEMSensorData();
                
            self.lock = threading.Lock()  
            self.running = True
        except:
            traceback.print_exc()
            self.running = False
    
    def update(self):
        self.lock.acquire();
        self.sensorData = self.emSensorProxy.getEMSensorData();

        self.lock.release();

        
    def getData(self):
        self.lock.acquire();
        data = copy.deepcopy(self.sensorData);
        self.lock.release();
        return data;
