import roslib
roslib.load_manifest("bulletsim_msgs")
from bulletsim_msgs.srv import *
from jds_image_proc.pcd_io import load_xyzrgb
from brett2.ros_utils import xyzrgb2pc
import rospy


if rospy.get_name() == "/unnamed":
    rospy.init_node("test_planner",disable_signals = True)
planner = rospy.ServiceProxy("plan_traj", PlanTraj)

req = PlanTrajRequest()
if 0:
    req.task = "cart"
    req.goal = [0,0,0,1,.5,0,.8]
    req.manip = "rightarm"
if 0:
    with open("pr2test1_no_pr2.xml") as fh:
        s = fh.read()
    req.task = "cart"
    req.manip = "rightarm"
    req.robot_joints = [0.]*39
    req.robot_transform = [0., -0., -0.10296736,  0.99468473, -0.798,  0.118,  0.075]
    req.xmlstring = s
    req.goal = 	[0.444178, 0.574522, -0.513645, 0.456946, -0.214549, 0.00710573, 0.820504]
if 1:
    xyz, bgr=load_xyzrgb("/home/joschu/Data/scp/three_objs.pcd")
    
    req.task = "cart"
    req.manip = "leftarm"
    req.robot_joints = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.261373, 0, 0, 1.62817, 0.114129, 1.5, -2.31862, 3.02074, -0.694258, 
                        1.77376, 0.548, -1.6237e-14, 0, -1.11022e-16, 0, -0.597069, -0.0458839, -4.44089e-15, -1.93778, -2.50727, 
                        -1.90577, 0.337549, 0, -3.41394e-15, 0, 3.60822e-16, 0]
    req.goal = [ 3.32359e-15, 0.707107, 4.36986e-15, 0.707107, 0.529845, 0.155853, 0.856245]
    pc = xyzrgb2pc(xyz, bgr, "base_footprint")
    req.cloud = pc
resp = planner.call(req)
print resp