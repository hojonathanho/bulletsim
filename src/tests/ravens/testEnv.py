from openravepy import *
from numpy import *
import time



env = Environment()
env.SetViewer('qtcoin')
env.Load('models/ravens.env.xml')
RaveSetDebugLevel(DebugLevel.Debug)
ravens     = env.GetRobots()[0]
