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
        #if topic=="/tf":
            #listener.callback(msg)
        if topic == owl_topic:
            owl = np.empty((K,3))
            owl.fill(np.nan)
            for marker in msg.markers:
                owl[marker.id] = np.array([owl.point.x, owl.point.y, owl.point.z])
            owl_arr.append(owl)
            owl_times.append(t)
        elif topic == tracker_topic:
            if not got_tracker_msg:
                object_type = msg.type
                if object_type == "mesh":
                    mesh = msg.mesh
                    faces = mesh.faces
                    N = len(msg.vertices)
                elif object_type == "mesh":
                    rope = msg.rope
                    N = len(rope.nodes)
                else: raise NotImplementedError
                got_tracker_msg = True
            
            tracker_pts = np.empty((N,3))
            if object_type == "rope": points = msg.rope.nodes
            elif object_type == "mesh": points = msg.mesh.vertices
            for (n, pt) in enumerate(points):
                tracker_pts[n] = (pt.x, pt.y, pt.z)
            tracker_arr.append(tracker_pts)
            tracker_times.append(t)
    assert got_tracker_msg
    return np.array(tracker_arr), np.array(tracker_times), faces, np.array(owl_arr), np.array(owl_times)

def evaluate_error_on_bag(fname, led_pairs):
    tracker_arr, tracker_times, tracker_triangles, owl_arr, owl_times = extract_bag_data(fname,100,"xxx","/tracker/object")
    
    used_leds = [i for pair in led_pairs for i in pair]
    
    led2ind = {}
    for (ind,led) in enumerate(used_leds):
        led2ind[led] = ind
    
    ind_pairs = [(led2ind[led0], led2ind[led1]) for (led0, led1) in led_pairs]
    
    return evaluation.calculate_cloth_tracking_score(tracker_arr, tracker_times, tracker_triangles, owl_arr[:,used_leds,:], owl_times, ind_pairs)