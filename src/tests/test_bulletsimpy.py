import openravepy as rave
import bulletsimpy

env = rave.Environment()
env.SetViewer('qtcoin')

env.Load('data/lab1.env.xml')

print 'number of objects in environment:', len(env.GetBodies())
print env.GetBodies()

dyn_obj_names = ['mug1', 'mug2', 'mug3', 'mug4', 'mug5']

bullet_env = bulletsimpy.LoadFromRave(env, dyn_obj_names)
bullet_env.SetGravity([0, 0, -9.8])
print 'gravity set to:', bullet_env.GetGravity()

bullet_objs = [bullet_env.GetObjectByName(b.GetName()) for b in env.GetBodies()]
print [(o.GetName(), o.IsKinematic()) for o in bullet_objs]

dyn_objs = [bullet_env.GetObjectByName(b.GetName()) for b in env.GetBodies() if b.GetName() in dyn_obj_names]
print 'dyn objs', [o.GetName() for o in dyn_objs]

TIMESTEPS = 100
for t in range(TIMESTEPS):
  print t

  for o in bullet_objs:
    print o.GetName()
    print o.GetTransform()
    env.GetKinBody(o.GetName()).SetTransform(o.GetTransform())

  env.UpdatePublishedBodies()

  bullet_env.Step(0.01, 100, 0.01)
  raw_input('hi')

