from mayavi import mlab
import numpy as np
import os.path as osp


def plot(runum, segnum):

    fpath = "/home/ankush/sandbox/bulletsim/src/tests/ravens/recorded/point_data/"
    point_file_format = "run%d-seg%d-points.txt"
    robot_file_format = "run%d-seg%d-robot.txt"

    ptfname = osp.join(fpath, point_file_format%(runum, segnum))
    robotfname = osp.join(fpath, robot_file_format%(runum, segnum))

    pts = np.loadtxt(ptfname)
    rpts = np.loadtxt(robotfname)

    mlab.points3d(pts[:,0], pts[:,1], pts[:,2], color=(1,0,0), resolution=20, scale_factor=0.001)
    mlab.points3d(rpts[:,0], rpts[:,1], rpts[:,2], color=(0,1,0), resolution=20, scale_factor=0.001)
