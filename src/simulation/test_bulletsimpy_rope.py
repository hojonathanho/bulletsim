import openravepy
import bulletsimpy
import numpy as np
import trajoptpy

env = openravepy.Environment()
env.Load('/home/robbie/trajopt/data/table.xml')
viewer = trajoptpy.GetViewer(env)

print 'number of objects in environment:', len(env.GetBodies())
print env.GetBodies()

bulletsimpy.sim_params.friction = 0
print 'friction from python', bulletsimpy.sim_params.friction
bt_env = bulletsimpy.BulletEnvironment(env, [])
print 'friction from python', bulletsimpy.sim_params.friction

rope_params = bulletsimpy.CapsuleRopeParams()
rope_params.radius = 0.005
rope_params.angStiffness=.1
rope_params.angDamping=.5
rope_params.linDamping=0
rope_params.angLimit=.4
rope_params.linStopErp=.2

n = 30
#c = np.array([.1, 0, .7])
#pts = np.array(np.c_[np.linspace(0, .5, n), np.zeros(n), np.zeros(n)]) + c
c = np.array([.5, .3, .7])
pts = np.array(np.c_[np.zeros(n), np.linspace(0, .5, n), np.zeros(n)]) + c
rope = bulletsimpy.CapsuleRope(bt_env, 'rope', pts, rope_params)

raw_input('start')
import time
t_start = time.time()
steps = 1000
for i in range(steps):
  bt_env.Step(0.01, 200, .005)
  rope.UpdateRave()
  env.UpdatePublishedBodies()
  viewer.Step()
  time.sleep(.01)

t_elapsed = time.time() - t_start
print 'took', t_elapsed, 'factor', t_elapsed/(.01*steps)
raw_input('enter to exit')
