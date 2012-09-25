from __future__ import division
import scipy.ndimage as ndi
import numpy as np
import cv2


def determine_object_type(xyz,plotting=False):
    if interior_frac(xyz,plotting=plotting) < .25:
        return "rope"
    else:
        if np.median(xyz[:,2]) < 0.05:
            return "towel"
        else:
            return "box"        
    
def get_biggest_cc(mask):
    labels, max_label = ndi.label(mask)
    counts = np.bincount(labels.flatten())
    return mask == counts.argmax()
        
def interior_frac(xyz,s=.01,plotting=False):
    xyz = np.asarray(xyz).reshape(-1,3)
    xmin, ymin, _ = xyz.min(axis=0) - .1
    xmax, ymax, _ = xyz.max(axis=0) + .1
    occupancy = np.zeros((int((xmax-xmin)/s), int((ymax-ymin)/s)),'uint8')

    for (x,y,_) in xyz:
        ix = int((x-xmin)/s)
        iy = int((y-ymin)/s)
        occupancy[ix, iy] = 1
    occupancy = ndi.binary_closing(occupancy, np.ones((3,3))).astype('uint8')
    
    if plotting:
        cv2.namedWindow("occ", 0)
        cv2.imshow("occ",occupancy*255)
        cv2.waitKey(10)
    
    return ndi.binary_erosion(occupancy,np.ones((5,5))).sum() / occupancy.sum() 
        