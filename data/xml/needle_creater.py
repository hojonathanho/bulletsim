#!/usr/bin/python

import numpy as np
import openravepy as opr
import sys

handles = []

def rotateByAngle (tfm, ang, r):
    pi = 3.14159265
    wTOee = tfm

    initT = np.eye(4)
    initT[0:3,3] = [-r,0,0]

    T = np.array ([[np.cos(ang), -np.sin(ang), 0, r],
                   [np.sin(ang), np.cos(ang) , 0, 0],
                   [0          , 0           , 1, 0],
                   [0          , 0           , 0, 1]])


    return wTOee.dot(T.dot(initT))

def plotTfm():
    angles = np.linspace(-np.pi/2, np.pi/2, 20)
    tfm = np.eye(4)
    #env.plot3(np.array([[0,0,0]]), 2)
    env.plot3(tfm[0:3,3], 10, [1,1,1])

    for ang in angles:
        tfm2 = rotateByAngle(tfm, ang, 0.1)
        h = env.plot3(tfm2[0:3,3], 10, [1,0,0])
        handles.append(h)
        print tfm2[0:3,3]
        print "tfm"

if __name__=="__main__":

    num_sphere = 50
    needle_rad = 0.0112

    if len(sys.argv) > 1:
        try: 
            num_sphere = int(sys.argv[1])
            print "Number of spheres: ", num_sphere
        except:
            pass
    if len(sys.argv) > 2:
        try: 
            needle_rad = float(sys.argv[2])
            print "Needle radius : ", needle_rad
        except:
            pass


    #Creating needle xml file
    fo = open("needle.xml","w")

    #Kinbody
    fo.write("<!--Needle-->\n")
    fo.write("<KinBody name=\"needle\">\n\n")

    #Body
    fo.write("  <Body name=\"needle\" type=\"dynamic\">\n\n")

    fo.write("    <!--Mass-->\n")
    fo.write("    <Mass type=\"mimicgeom\">\n      <density>1000</density>\n    </Mass>\n\n")

    fo.write("    <!--Spheres-->\n")
    sphere_rad = needle_rad*np.pi/(num_sphere)
    angles = np.linspace(-np.pi/2, np.pi/2, num_sphere)
    for ang in angles:
        tfm = rotateByAngle(np.eye(4), ang, needle_rad)
        c = tfm[0:3,3]
        fo.write("    <Geom type=\"sphere\">\n")
        fo.write("      <Translation>"+str(c[0])+" "+str(c[1])+" "+str(c[2])+"</Translation>\n")
        fo.write("      <Radius>"+str(sphere_rad)+"</Radius>\n")
        fo.write("    </Geom>\n")
        
    fo.write("\n  </Body>\n\n")

    fo.write("</KinBody>")
    fo.close()
