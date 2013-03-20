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

if __name__=="__main__":

    num_box = 50
    needle_rad = 0.0112

    if len(sys.argv) > 1:
        try: 
            num_box = int(sys.argv[1])
            print "Number of boxes: ", num_box
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

    fo.write("    <!--Boxes-->\n")
    box_len = needle_rad*np.pi/(num_box)
    angles = np.linspace(-np.pi/2, np.pi/2, num_box)
    he = [box_len/2, box_len, box_len/2]
    for ang in angles:
        tfm = rotateByAngle(np.eye(4), ang, needle_rad)
        c = tfm[0:3,3]
        fo.write("    <Geom type=\"box\">\n")
        fo.write("      <Extents>"+str(he[0])+" "+str(he[1])+" "+str(he[2])+"</Extents>\n")        
        fo.write("      <Translation>"+str(c[0])+" "+str(c[1])+" "+str(c[2])+"</Translation>\n")
        fo.write("      <RotationMat>"+str(tfm[0,0])+" "+str(tfm[0,1])+" "+str(tfm[0,2])+" "\
                                      +str(tfm[1,0])+" "+str(tfm[1,1])+" "+str(tfm[1,2])+" "\
                                      +str(tfm[2,0])+" "+str(tfm[2,1])+" "+str(tfm[2,2])+" "+"</RotationMat>\n")
        fo.write("    </Geom>\n")   
    
    fo.write("\n  </Body>\n\n")

    fo.write("</KinBody>")
    fo.close()
