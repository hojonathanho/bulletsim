# example2.py
# simple rigid body tracking program 

from OWL import *

# change these to match your configuration

MARKER_COUNT = 4

SERVER_NAME = "localhost"
INIT_FLAGS = 0

RIGID_BODY = [[0, 300, 0],
              [300, 0, 0],
              [-300, 0, 0],
              [0, 76, -200]]

def owl_print_error(s, n):
    if n < 0: print "%s: %d\n" % (s, n)
    elif n == OWL_NO_ERROR: print "%s: No Error\n" % s
    elif n == OWL_INVALID_VALUE: print "%s: Invalid Value\n" % s
    elif n == OWL_INVALID_ENUM: print "%s: Invalid Enum\n" % s
    elif n == OWL_INVALID_OPERATION: print "%s: Invalid Operation\n" % s
    else: print "%s: 0x%x\n" % (s, n)
    pass

def copy_p(a, b):
    for i in range(7): b[i] = a[i]
    pass

def print_p(p) :
    for i in range(7): print "%f " % p[i]
    pass

def main():

    rigids = []
    markers = []
    cameras = []

    tracker = 0

    if(owlInit(SERVER_NAME, INIT_FLAGS) < 0): return 0

    # create rigid body tracker 0
    tracker = 0;  
    owlTrackeri(tracker, OWL_CREATE, OWL_RIGID_TRACKER)
  
    # set markers
    for i in range(MARKER_COUNT):
        
        # set markers
        owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i)

        # set marker positions
        owlMarkerf(MARKER(tracker, i), OWL_SET_POSITION, RIGID_BODY[i])
        pass

    # activate tracker
    owlTracker(tracker, OWL_ENABLE)

    # flush requests and check for errors
    if not owlGetStatus():    
        owl_print_error("error in point tracker setup", owlGetError())
        return 0
    

    # set default frequency
    owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY)
  
    # start streaming
    owlSetInteger(OWL_STREAMING, OWL_ENABLE)

    # main loop
    while 1:
    
        err = 0

        # get the rigid body
        n = owlGetRigids(rigids, 1)        

        # get the rigid body markers
        #  note: markers have to be read,
        #  even if they are not used
        m = owlGetMarkers(markers, 32)

        # get cameras
        o = owlGetCameras(cameras, 32)

        # check for error
        err = owlGetError()
        if err != OWL_NO_ERROR: 
            owl_print_error("error", err)
            break

        # no data yet
        if(n == 0 or o == 0):
            continue

        if(n > 0):
            
            print "%d rigid body, %d markers, %d cameras:\n" %(n,m,o)
            if(rigids[0].cond > 0):
                
                inv_pose = range(7)
                copy_p(rigids[0].pose, inv_pose)
                MATH.invert_p(inv_pose)
                
                print "\nRigid: "
                print_p(rigids[0].pose)
                
                print "\nCameras: "
                for i in range(o):                
                    # multiply each camera's pose by inverse of rigid pose
                    cam_pose = range(7)
                    copy_p(cameras[i].pose, cam_pose)
                    cameras[i].pose = MATH.mult_pp(inv_pose, cam_pose)
                    
                    print_p(cameras[i].pose)
                    print "\n"
                    pass
                
                print "\n"
                pass
            print "\n"
            pass
        pass
    
    # cleanup
    owlDone()
    pass

main()
