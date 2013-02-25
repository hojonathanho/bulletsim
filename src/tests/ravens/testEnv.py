from openravepy import *
from numpy import *
import time

def drawTransform(env,T,length=0.2):
    """draws a set of arrows around a coordinate system
    """
    return [env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=0.005,color=[1.0,0.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=0.005,color=[0.0,1.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=0.005,color=[0.0,0.0,1.0])]

env = Environment()
env.SetViewer('qtcoin')
env.Load('models/ravens.env.xml')#'data/pr2test1.env.xml')#)
RaveSetDebugLevel(DebugLevel.Debug)
ravens     = env.GetRobots()[0]

h = []

for l in ravens.GetLinks():
    print l.GetName()
    if l.GetName()in ['r_gripper_l_finger_tip_link', 'r_gripper_r_finger_tip_link','l_finger_l_tip_link', 'l_grasper1_L', 'l_palm_L', 'r_palm_L']:
        T = l.GetTransform()
        print l.GetName()
        #h.append(drawTransform(env,T))
        raw_input()

