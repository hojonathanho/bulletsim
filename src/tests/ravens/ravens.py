from openravepy import *
import time
from numpy import random


def main(env, robot):

    newrobots = []

    for ind in range(10):
        newrobot = RaveCreateRobot(env,robot.GetXMLId())
        newrobot.Clone(robot,0)
        for link in newrobot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(0.8)
        newrobots.append(newrobot)

    for link in robot.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(0.8)

    while True:
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            print "Auto generating IK"
            ikmodel.autogenerate()
        else:
            print "File name :", ikmodel.getfilename()
            print "Source file name : ", ikmodel.getsourcefilename()
        with env:
            # move the robot in a random collision-free position and call the IK
            while True:
                lower,upper = [v[ikmodel.manip.GetArmIndices()] for v in ikmodel.robot.GetDOFLimits()]
                robot.SetDOFValues(random.rand()*(upper-lower)+lower,ikmodel.manip.GetArmIndices()) # set random values
                if not robot.CheckSelfCollision():
                    solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetTransform(),IkFilterOptions.CheckEnvCollisions)
                    if solutions is not None and len(solutions) > 0: # if found, then break
                        break
                time.sleep(0.01)
            
            print 'found %d solutions, rendering solutions:'%len(solutions)
            if len(solutions) < 10:
                inds = range(len(solutions))
            else:
                inds = array(linspace(0,len(solutions)-1,options.maxnumber),int)
            for i,ind in enumerate(inds):
                print ind
                newrobot = newrobots[i]
                env.Add(newrobot,True)
                newrobot.SetTransform(robot.GetTransform())
                newrobot.SetDOFValues(solutions[ind],ikmodel.manip.GetArmIndices())

        env.UpdatePublishedBodies()
        print('waiting...')
        time.sleep(2)
        # remove the robots
        for newrobot in newrobots:
            env.Remove(newrobot)
        time.sleep(0.01)
    del newrobots


if __name__=='__main__':
    env = Environment() 
    env.SetViewer('qtcoin') 
    env.Load('data/pr2test1.env.xml')#raven_env.xml') 
#    robot = env.GetRobots()[0] 
#    raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\n"+repr(robot.GetDOFValues()))
#robot.SetDOFValues([0.5],[0]) # set joint 0 to value 0.5
#    T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
#    raveLogInfo("The transformation of link 1 is:\n"+repr(T))
#    raveLogInfo("Robot hash is :%s\n"%robot.GetManipulators()[0].GetName())#RobotStructureHash()))
    
    while True:
        time.sleep(0.01)
    #main(env, robot)
        
        

