from __future__ import division

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--redo_bag", action="store_true")
args = parser.parse_args()


import roslib
roslib.load_manifest("bulletsim_python")
import tracking_evaluation.bag_proc as bp
import tracking_evaluation.evaluation as ev
import matplotlib.pyplot as plt
import cPickle as cP
import numpy as np
import os.path as osp

fname = "/home/joschu/Data/tracking_eval/dataset_test0_tracked.bag"
led_pairs =  [(24,17), (10,18), (13, 21), 
              (8, 16), (11, 19), (14, 22),
              (9, 25), (12, 20), (15, 23)]

bag_data_cache_file = osp.join("/tmp",osp.basename(fname)+".npz")
if args.redo_bag or not osp.exists(bag_data_cache_file):
    bag_data = bp.extract_bag_data(fname,50,"phasespace_markers","tracker/object")
    np.savez(bag_data_cache_file, *bag_data)
else:
    bag_data_file = np.load(bag_data_cache_file)
    bag_data = [bag_data_file[key] for key in sorted(bag_data_file.keys())]
tracker_arr, tracker_times, tracker_triangles, owl_arr, owl_times = bag_data

owl_arr /= 10

used_leds = [i for pair in led_pairs for i in pair]

led2ind = {}
for (ind,led) in enumerate(used_leds):
    led2ind[led] = ind

ind_pairs = [(led2ind[led0], led2ind[led1]) for (led0, led1) in led_pairs]


marker2info =  ev.calculate_cloth_tracking_score(tracker_arr, tracker_times, tracker_triangles, owl_arr[:,used_leds,:], owl_times, ind_pairs,demean_at_beginning=False)

if 0:
    for info in marker2info:
        err_q = ev.norms(info["position_demeaned"],1)
        #err_q = err_q3[:,0]
        plt.plot(info["times"], err_q,'.')
    plt.xlabel('seconds')
    plt.ylabel('error (cm)')
    plt.legend([str(led) for led in used_leds])


if 0:
    import mayavi.mlab as mlab
    print "nodes"
    for node in xrange(0,tracker_arr.shape[1],40):
        xyz = tracker_arr[:,node,:]
        good_inds = np.flatnonzero(~np.isnan(xyz[:,0]))
        if len(good_inds) > 0:
            xyz_good = xyz[good_inds]
            x,y,z = xyz_good.T
            print xyz_good.min(axis=0), xyz_good.max(axis=0)
            mlab.plot3d(x,y,z,np.float32(good_inds)/good_inds.max(),tube_radius=.25,opacity=.5,colormap='Spectral')
    
    
    print "markers"
    for node in used_leds:
        xyz = owl_arr[:,node,:]
        good_inds = np.flatnonzero(np.isfinite(xyz[:,0]))
        good_inds = good_inds[::100]
        if len(good_inds) > 0:
            xyz_good = xyz[good_inds]
            print xyz_good.min(axis=0), xyz_good.max(axis=0)
            x,y,z = xyz_good.T
            mlab.points3d(x,y,z,np.float32(good_inds)/good_inds.max(),scale_mode='none',scale_factor=1,opacity=.5,colormap='Spectral')
    
    mlab.points3d([0],[0],[0], color=(1,1,1),scale_factor=5)
        
if 1:
    import mayavi.mlab as mlab
    led_ind = 0
    t = marker2info[led_ind]["times"]
    xyz = marker2info[led_ind]["position_prediction"]
    x,y,z = xyz.T
    mlab.plot3d(x,y,z,t,tube_radius=.25,opacity=.5,colormap='Spectral')
    
    
#x,y,z = marker2info[2]["position_actual"].T
#mlab.points3d(x,y,z,color=(1,0,0),scale_factor=3)

#x,y,z = marker2info[2]["position_prediction"].T
#mlab.points3d(x,y,z,color=(0,1,0),scale_factor=3)