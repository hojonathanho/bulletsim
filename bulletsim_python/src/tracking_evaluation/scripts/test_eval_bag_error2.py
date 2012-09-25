from __future__ import division

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("bagfile")
parser.add_argument("--item",choices=["rope","cloth"],default="rope")
parser.add_argument("--robot", action="store_true")
parser.add_argument("--redo_bag", action="store_true")
parser.add_argument("--reset_cache", action="store_true")
args = parser.parse_args()


import roslib
roslib.load_manifest("bulletsim_python")
import tracking_evaluation.bag_proc as bp
import tracking_evaluation.evaluation as ev
import matplotlib.pyplot as plt
import cPickle as cP
import numpy as np
import os.path as osp
from jds_utils.math_utils import remove_duplicate_rows

#fname = "/home/joschu/Data/tracking_eval/dataset_test0_tracked.bag"
fname = args.bagfile
led_pairs =  [(24,17), (10,18), (13, 21), 
              (8, 16), (11, 19), (14, 22),
              (9, 25), (12, 20), (15, 23)]
rope_leds = [32,33,34,35,36,37,38,39]


bag_data_cache_file = osp.join("/tmp",osp.basename(fname)+".npz")
if args.redo_bag or not osp.exists(bag_data_cache_file):
    bag_data = bp.extract_bag_data(fname,50,"/phasespace_markers","/tracker/object", robot = args.robot)
    np.savez(bag_data_cache_file, *bag_data)
else:
    bag_data_file = np.load(bag_data_cache_file)
    bag_data = [bag_data_file[key] for key in sorted(bag_data_file.keys())]
tracker_arr, tracker_times, tracker_triangles, owl_arr, owl_times = bag_data



def calc_final_error(dt):
    
    tStartTracker = tracker_times.min() - owl_times.min() - dt
    
    if args.item == "cloth":
        used_leds = [i for pair in led_pairs for i in pair]    
        led2ind = {}
        for (ind,led) in enumerate(used_leds):
            led2ind[led] = ind
        ind_pairs = [(led2ind[led0], led2ind[led1]) for (led0, led1) in led_pairs]    
        marker2info =  ev.calculate_cloth_tracking_score1(tracker_arr, tracker_times-tStartTracker, tracker_triangles, owl_arr[:,used_leds,:], owl_times, ind_pairs)
    elif args.item == "rope":
        used_leds = rope_leds
        marker2info = ev.calculate_rope_tracking_score(tracker_arr, tracker_times-tStartTracker, owl_arr[:,used_leds,:], owl_times, used_leds)
    
    errors = []
    for info in marker2info:
        err_q = ev.norms(info["position_relative"],1)    
        errors.extend(err_q.tolist())
    final_error = np.median(errors)  
    print dt, final_error
    return final_error
    
cache_file = osp.join("/home/joschu/Data/cache", "offset_%s.txt"%osp.basename(args.bagfile))
if osp.exists(cache_file) and not args.reset_cache:
    with open(cache_file,"r") as fh: toff = float(fh.read())
    errmin = calc_final_error(toff)
else:
    import scipy.optimize
    toff, errmin = scipy.optimize.brent(calc_final_error, brack=(0,3), full_output=1, maxiter=10, tol=.05)[:2]
    print "min error: %.2f at time offset %.2f"%(errmin, toff)
    with open(cache_file,"w") as fh: fh.write("%.4f"%toff)


with open("/home/joschu/Data/tracking_results/all_results.txt","a") as fh:
    fh.write("%s: %.3f\n\n"%(osp.basename(args.bagfile), errmin))