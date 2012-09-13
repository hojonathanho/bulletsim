#!/usr/bin/python
# example1.py
# simple point tracking program

import sys
from OWL import *

# change these to match your configuration

MARKER_COUNT = 4
SERVER_NAME = "localhost"
INIT_FLAGS = 0

if(owlInit(SERVER_NAME, INIT_FLAGS) < 0):
    print "init error: ", owlGetError()
    sys.exit(0)

# create tracker 0
tracker = 0
owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER)

# set markers
for i in range(MARKER_COUNT):
    owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i)

# activate tracker
owlTracker(tracker, OWL_ENABLE)

# flush requests and check for errors
if(owlGetStatus() == 0):
    owl_print_error("error in point tracker setup", owlGetError())
    sys.exit(0)

# set define frequency
owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY)

# start streaming
owlSetInteger(OWL_STREAMING, OWL_ENABLE)

# main loop
while(1):
    markers = []

    # get some markers
    n = owlGetMarkers(markers, 32)

    # check for error
    err = owlGetError()
    if(err != OWL_NO_ERROR):
        owl_print_error("error", err)
        break

    # no data yet
    if(n == 0): continue

    if(n > 0):
        print n, "markers(s):"
        for i in range(n):
            if(markers[i].cond > 0):
                print "%d) %.2f %.2f %.2f" % (i, markers[i].x, markers[i].y, markers[i].z)
        print ""

# cleanup
owlDone();


def owl_print_error(s, n):
    """Print OWL error."""
    if(n < 0): print "%s: %d" % (s, n)
    elif(n == OWL_NO_ERROR): print "%s: No Error" % s
    elif(n == OWL_INVALID_VALUE): print "%s: Invalid Value" % s
    elif(n == OWL_INVALID_ENUM): print "%s: Invalid Enum" % s
    elif(n == OWL_INVALID_OPERATION): print "%s: Invalid Operation" % s
    else: print "%s: 0x%x" % (s, n)
