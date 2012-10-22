import openravepy
import numpy as np
env = openravepy.Environment()

def make_box(center, half_extents, name):
    box = openravepy.RaveCreateKinBody(env, '')
    extents = [0,0,0,.1,.1,.1]
    box.SetName(name)
    box.InitFromBoxes(np.array([extents]), True)
    trans = np.eye(4)
    trans[:3,3] = center
    box.SetTransform(trans)
    env.Add(box)
    box.GetLinks()[0].SetMass(1)
    return box
    
def make_mesh_box(center, half_extents, name):
    box = openravepy.RaveCreateKinBody(env, '')
    rx, ry, rz = half_extents
    verts = np.array([
        [-rx, -ry, -rz],
        [-rx, -ry, rz],
        [-rx, ry, -rz],
        [-rx, ry, rz],
        [rx, -ry, -rz],
        [rx, -ry, rz],
        [rx, ry, -rz],
        [rx, ry, rz]])
    faces= [
        [0,1,2],
        [3,1,2],
        [0,1,4],
        [5,1,4],
        [0,2,4],
        [6,2,4],
        [7,6,5],
        [4,6,5],
        [7,6,3],
        [2,6,3],
        [7,5,3],
        [1,5,3]]
    box.SetName(name)
    box.InitFromTrimesh(openravepy.TriMesh(verts, faces), True)
    trans = np.eye(4)
    trans[:3,3] = center
    box.SetTransform(trans)
    env.Add(box)
    return box
    

box0 = make_box([0,0,1], [.1,.1,.1],'box0')
box1 = make_box([.2,0,1], [.1,.1,.1],'box1')

#env.SetViewer('qtcoin')
