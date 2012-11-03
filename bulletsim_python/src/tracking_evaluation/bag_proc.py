import evaluation, rosbag
import numpy as np
from brett2.mytf import TransformListener

def ensure_slash(s):
    return s if s.startswith("/") else "/" + s

def extract_bag_data(fname, max_led_ind, owl_topic, tracker_topic, robot = False):
    bag = rosbag.Bag(fname)
    
    owl_topic = ensure_slash(owl_topic)
    tracker_topic = ensure_slash(tracker_topic)
    
    if robot: listener = TransformListener()
    
    K = max_led_ind
    
    tracker_arr = []    
    owl_arr = []
    tracker_times = []
    owl_times = []
    
    object_type = None
    faces = None
    got_tracker_msg = False
    
    for (i,(topic, msg, tbag)) in enumerate(bag.read_messages()):
        if topic == "/tf":
            t = tbag
        else:
            t = msg.header.stamp
        
        
        if i==0:
            t0 = t.to_sec()
        if i%1000 == 0:
            print "processed %i messages"%i
        #if i>9000: break
        if robot and topic=="/tf":
            listener.callback(msg)
        if topic == owl_topic:
            owl = np.empty((K,3))
            owl.fill(np.nan)
            if robot:
                try:
                    hmat = listener.transformer.lookup_transform_mat("/base_footprint", "/ground")
                except Exception:
                    continue
            else:
                hmat = np.eye(4)
            for marker in msg.markers:
                if marker.cond > 0:
                    owl[marker.id] = hmat.dot(.001*np.array([marker.point.x, marker.point.y, marker.point.z,1000]))[:3]
            owl_arr.append(owl)
            owl_times.append(t.to_sec()-t0)
        elif topic == tracker_topic:
            if not got_tracker_msg:
                object_type = msg.type
                if object_type == "towel":
                    mesh = msg.mesh
                    faces = [face.vertex_inds for face in mesh.faces]
                    N = len(mesh.vertices)
                elif object_type == "rope":
                    rope = msg.rope
                    N = len(rope.nodes)
                else: raise NotImplementedError
                got_tracker_msg = True
            
            tracker_pts = np.empty((N,3))
            if object_type == "rope": points = msg.rope.nodes
            elif object_type == "towel": points = msg.mesh.vertices
            for (n, pt) in enumerate(points):
                tracker_pts[n] = (pt.x, pt.y, pt.z)
            tracker_arr.append(tracker_pts)
            tracker_times.append(t.to_sec()-t0)
    assert got_tracker_msg
    print hmat
    return np.array(tracker_arr), np.array(tracker_times), faces, np.array(owl_arr), np.array(owl_times)
