#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--plotting", action="store_true")
parser.add_argument("--save_requests", action="store_true")
parser.add_argument("--load_request", type=str)
parser.add_argument("--simple_rope_init", action="store_true")
args = parser.parse_args()


import roslib
roslib.load_manifest("bulletsim_python")
from brett2.ros_utils import pc2xyzrgb
import bulletsim_msgs.srv as bs
import bulletsim_msgs.msg as bm
import geometry_msgs.msg as gm
import rospy
from brett2.ros_utils import RvizWrapper
import  tracking_initialization.rope_initialization as ri
from tracking_initialization.object_recognition_lol import determine_object_type
import numpy as np
import cv2
import cPickle, time
marker_handles = {}

def handle_initialization_request(req):
    "rope initialization service"
    
    if args.save_requests:
        fname = "/tmp/init_req_%i.pkl"%time.time()
        with open(fname,"w") as fh:
            cPickle.dump(req, fh)
            print "saved", fname
    
    xyz, _ = pc2xyzrgb(req.cloud)
    xyz = np.squeeze(xyz)
    obj_type = determine_object_type(xyz, plotting=args.plotting)
    print "object type:", obj_type
    
    
    if obj_type == "towel":
        xyz = xyz[(xyz[:,2]-np.median(xyz[:,2]))<.05,:]
        center, (height, width), angle = cv2.minAreaRect(xyz[:,:2].astype('float32').reshape(1,-1,2))
        angle *= np.pi/180
        corners = np.array(center)[None,:] + np.dot(
            np.array([[-height/2,-width/2],
                      [-height/2,width/2],
                      [height/2, width/2],
                      [height/2, -width/2]]),
            np.array([[np.cos(angle), np.sin(angle)],
                      [-np.sin(angle),np.cos(angle)]])) 
        resp = bs.InitializationResponse()
        resp.objectInit.type = "towel_corners"
        resp.objectInit.towel_corners.polygon.points = [gm.Point(corner[0], corner[1], np.median(xyz[:,2])) for corner in corners]
        resp.objectInit.towel_corners.header = req.cloud.header
        print req.cloud.header
        poly_pub.publish(resp.objectInit.towel_corners)
    
    elif obj_type == "rope":
        if args.simple_rope_init:
            total_path_3d = ri.find_path_through_point_cloud_simple(xyz, plotting=args.plotting)            
        else:
            total_path_3d = ri.find_path_through_point_cloud(xyz, plotting=args.plotting)            
        resp = bs.InitializationResponse()
        resp.objectInit.type = "rope"
        rope = resp.objectInit.rope = bm.Rope()
        
        rope.header = req.cloud.header
        rope.nodes = [gm.Point(x,y,z) for (x,y,z) in total_path_3d]    
        rope.radius = .006
        print "lengths:", [np.linalg.norm(total_path_3d[i+1] - total_path_3d[i]) for i in xrange(len(total_path_3d)-1)]        
        rospy.loginfo("created a rope with %i nodes, each has length %.2f, radius %.2f", len(rope.nodes), 
                      np.linalg.norm(total_path_3d[i+1] - total_path_3d[i]), rope.radius)
        
        pose_array = gm.PoseArray()
        pose_array.header = req.cloud.header
        pose_array.poses = [gm.Pose(position=point, orientation=gm.Quaternion(0,0,0,1)) for point in rope.nodes]
#        marker_handles[0] = rviz.draw_curve(pose_array,0)

    elif obj_type == "box":
        xyz = xyz[abs(xyz[:,2]-np.median(xyz[:,2]))<.05,:]
        center, (height, width), angle = cv2.minAreaRect(xyz[:,:2].astype('float32').reshape(1,-1,2))
        angle *= np.pi/180
        print 'width', width, 'height', height, 'angle', 'median', np.median(xyz[:,2]),angle*180/np.pi

        resp = bs.InitializationResponse()
        resp.objectInit.type = "box"
        rope = resp.objectInit.box = bm.Box()

        #rope.header = req.cloud.header                                                 
        rope.center.x = center[0]
        rope.center.y = center[1]
        rope.center.z = np.median(xyz[:,2])/2.0
        rope.extents.x = height
        rope.extents.y = width
        rope.extents.z = np.median(xyz[:,2])
        rope.angle = angle
    
    return resp
    

import time
if __name__ == "__main__":
    if args.load_request:
        import matplotlib
        matplotlib.use("wx")
        with open(args.load_request, "r") as fh:
            req = cPickle.load(fh)
        handle_initialization_request(req)
    else:
        rospy.init_node('rope_initialization_node',disable_signals=True)
        s = rospy.Service('/initialization', bs.Initialization, handle_initialization_request)
        print "Ready to initialize ropes."
        poly_pub = rospy.Publisher("/initialization/towel_corners_for_viz", gm.PolygonStamped)
        #rospy.spin()    
        time.sleep(999999)
