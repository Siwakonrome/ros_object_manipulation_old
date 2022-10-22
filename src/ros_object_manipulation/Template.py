import tf

class Topic():
    def __new__(self, name, type):
        return {'name':name, 'type':type}

class Pose():
    ''' @brief
        position struct, unit = meter.
    '''
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
    
    def __call__(self):
        return (self.x, self.y, self.z)
    
    def set(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self):
        return f"{self.__class__.__name__}:[(x={self.x}),(y={self.y}),(z={self.z})]"

class Orientation():
    ''' @brief
        orientation struct, unit = rad.
    ''' 
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
    
    def __call__(self):
        return tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
    
    def set(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    def __str__(self):
        return f"{self.__class__.__name__}:[(roll={self.roll}),(pitch={self.pitch}),(yaw={self.yaw})]"

class Position():
    def __init__(self):
        self.pose = Pose()
        self.orientation = Orientation()
    
    def __str__(self):
        return f"{self.__class__.__name__}:[{str(self.pose)},{str(self.orientation)}]"