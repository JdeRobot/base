class ListenerLaser:
    def __init__(self, nodeName, topic, init=False):
        self.nodeName = nodeName
        self.topic = topic
        self.data = None
        self.node = None
        self.sub = None
        self.lock = threading.Lock()
        if (not init):
            self.node = rospy.init_node(self.nodeName, anonymous=True)
        #self.start()
        self.sub = rospy.Subscriber(self.topic, LaserScan, self.__callback)
 
    def __callback (self, data):
        print(self.nodeName)
        self.lock.acquire()
        self.data = data
        self.lock.release()
        
    def hasproxy (self):
        return self.sub != None
        
    def getLaserData(self):
        self.lock.acquire()
        data = self.data
        self.lock.release()
        laser = LaserDat()
        if (data != None):
            laser.distanceData = data.ranges
            laser.numLaser = len(data.ranges)
            laser.minAngle = data.angle_min  - PI/2
            laser.maxAngle = data.angle_max  - PI/2
            laser.maxRange = data.range_max
            laser.minRange = data.range_min
            #laser = jderobot.LaserData(data.ranges, len(data.ranges), data.angle_min  - PI/2, data.angle_max  - PI/2, 0, 0)
        return laser