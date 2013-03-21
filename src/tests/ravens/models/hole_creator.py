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

    s = 0.01
    h = 0.0015
    if len(sys.argv) > 1:
        try: 
            s = float(sys.argv[1])
            print "Side of box of hole: ", s
        except:
            pass
    if len(sys.argv) > 2:
        try: 
            h = float(sys.argv[2])
            print "Height of box of hole: ", h
        except:
            pass


    #Creating needle xml file
    fo = open("hole.xml","w")

    #Kinbody
    fo.write("<!--Hole-->\n")
    fo.write("<KinBody name=\"hole\">\n\n")

    #Body
    fo.write("  <Body name=\"hole\" type=\"dynamic\">\n\n")

    fo.write("    <!--Mass-->\n")
    fo.write("    <Mass type=\"mimicgeom\">\n      <density>250</density>\n    </Mass>\n\n")

    fo.write("    <!--Boxes-->\n")
    fo.write("    <Geom type=\"box\">\n")
    fo.write("      <Extents>"+str(s/6)+" "+str(s/2)+" "+str(h/2)+"</Extents>\n")
    fo.write("      <Translation>"+str(s/3)+" "+str(0)+" "+str(0)+"</Translation>\n")
    fo.write("    </Geom>\n")
    fo.write("    <Geom type=\"box\">\n")
    fo.write("      <Extents>"+str(s/6)+" "+str(s/2)+" "+str(h/2)+"</Extents>\n")
    fo.write("      <Translation>"+str(-s/3)+" "+str(0)+" "+str(0)+"</Translation>\n")
    fo.write("    </Geom>\n")
    fo.write("    <Geom type=\"box\">\n")
    fo.write("      <Extents>"+str(s/2)+" "+str(s/6)+" "+str(h/2)+"</Extents>\n")
    fo.write("      <Translation>"+str(0)+" "+str(s/3)+" "+str(0)+"</Translation>\n")
    fo.write("    </Geom>\n")
    fo.write("    <Geom type=\"box\">\n")
    fo.write("      <Extents>"+str(s/2)+" "+str(s/6)+" "+str(h/2)+"</Extents>\n")
    fo.write("      <Translation>"+str(0)+" "+str(-s/3)+" "+str(0)+"</Translation>\n")
    fo.write("    </Geom>\n")
    
    fo.write("\n  </Body>\n\n")

    fo.write("</KinBody>")
    fo.close()
