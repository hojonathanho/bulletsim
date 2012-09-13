import evaluation, rosbag
import numpy as np

def extract_bag_data(fname, max_led_ind, owl_topic, tracker_topic):
    bag = rosbag.Bag(fname)
    
    K = max_led_ind
    
    tracker_arr = []    
    owl_arr = []
    tracker_times = []
    owl_times = []
    
    object_type = None
    faces = None
    got_tracker_msg = False
    
    for (i,(topic, msg, t)) in enumerate(bag.read_messages()):
        if i==0:
            t0 = t.to_sec()
        if i%1000 == 0:
            print "processed %i messages"%i
        #if i>9000: break
        #if topic=="/tf":
            #listener.callback(msg)
        if topic == owl_topic:
            owl = np.empty((K,3))
            owl.fill(np.nan)
            for marker in msg.markers:
                if marker.point.x != 0:
                    owl[marker.id] = np.array([marker.point.x, marker.point.y, marker.point.z])
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
    return np.array(tracker_arr), np.array(tracker_times), faces, np.array(owl_arr), np.array(owl_times)
