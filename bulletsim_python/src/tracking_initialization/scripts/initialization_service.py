#!/usr/bin/env python

from brett2.ros_utils import pc2xyzrgb
import bulletsim_msgs.srv as bs
import bulletsim_msgs.msg as bm
import geometry_msgs.msg as gm
import rospy
from brett2.ros_utils import RvizWrapper
from rope_initialization import find_path_through_point_cloud
from object_recognition_lol import determine_object_type
import numpy as np
import cv2

marker_handles = {}

def handle_initialization_request(req):
    "rope initialization service"
    
    xyz, _ = pc2xyzrgb(req.cloud)
    xyz = np.squeeze(xyz)
    obj_type = determine_object_type(xyz, plotting=False)
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
    
    if obj_type == "rope":
        total_path_3d = find_path_through_point_cloud(xyz, plotting=False)            
        resp = bs.InitializationResponse()
        resp.objectInit.type = "rope"
        rope = resp.objectInit.rope = bm.Rope()
        
        rope.header = req.cloud.header
        rope.nodes = [gm.Point(x,y,z) for (x,y,z) in total_path_3d]    
        rope.radius = .006
        rospy.logwarn("TODO: actually figure out rope radius from data. setting to .4cm")
        
        pose_array = gm.PoseArray()
        pose_array.header = req.cloud.header
        pose_array.poses = [gm.Pose(position=point, orientation=gm.Quaternion(0,0,0,1)) for point in rope.nodes]
        marker_handles[0] = rviz.draw_curve(pose_array,0)

        
    
    return resp
    


if __name__ == "__main__":
    rospy.init_node('rope_initialization_node')
    s = rospy.Service('/initialization', bs.Initialization, handle_initialization_request)
    print "Ready to initialize ropes."
    poly_pub = rospy.Publisher("/initialization/towel_corners_for_viz", gm.PolygonStamped)
    rviz = RvizWrapper()
    rospy.spin()    
