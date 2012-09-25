import rospy
import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
import bulletsim_msgs.msg as bm
import bulletsim_msgs.srv as bs
from brett2 import ros_utils
from jds_utils import conversions
import roslib; roslib.load_manifest('tf'); import tf
import numpy as np


sub = rospy.Subscriber("/preprocessor/points", sm.PointCloud2)
service = rospy.ServiceProxy('/initialization', bs.Initialization)

rospy.init_node("loop_initialization")
listener = tf.TransformListener()

#rviz = ros_utils.RvizWrapper()



while not rospy.is_shutdown():
    cloud = rospy.wait_for_message("/preprocessor/points", sm.PointCloud2)
    cloud_tf = ros_utils.transformPointCloud2(cloud, listener, "ground", cloud.header.frame_id)
    
    req = bs.InitializationRequest()
    req.cloud = cloud_tf
    resp = service.call(req)
    #pose_array = gm.PoseArray()
    #pose_array.header = cloud.header
    #pose_array.poses = [gm.Pose(position=point, orientation=gm.Quaternion(0,0,0,1)) for point in resp.objectInit.rope.nodes]
    #rviz.draw_curve(pose_array, 0)
    #rospy.sleep(1)