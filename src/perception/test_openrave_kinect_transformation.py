import openravepy
import numpy as np
import time
np.set_printoptions(precision=3)
env = openravepy.Environment()
env.Load("robots/pr2-beta-sim.robot.xml")
env.SetViewer('qtcoin')
pr2 = env.GetRobots()[0]


def draw_axes(env,tf):
    
    origin = tf[:3,3]
    for i in xrange(3):
        ax = tf[:3,i]
        color = [int(j==i) for j in xrange(3)] + [1]
        time.sleep(.01)
        env.drawarrow(origin, origin+3*ax)
        print origin, origin+3*ax

pos = {
   'x': -0.039678040598,
   'y': 0.108097852269,
   'z': 0.0041499301663}
ori = {
  'x': 0.0332678958013,
  'y': 0.00521221114218,
  'z': 0.00204780698795,
  'w': 0.99943078122 }


camLink = pr2.GetLink('wide_stereo_optical_frame')

worldFromCam = camLink.GetTransform()
kinFromCam = openravepy.matrixFromPose([ori['w'],ori['x'],ori['y'],ori['z'],pos['x'],pos['y'],pos['z']])
worldFromKin = np.dot(worldFromCam, np.linalg.inv(kinFromCam))
time.sleep(.01)
with env:
    draw_axes(env, worldFromCam)
