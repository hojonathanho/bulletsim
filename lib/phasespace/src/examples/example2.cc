// example2.cc
// simple rigid body tracking program 

#include <stdio.h>

#include "owl.h"
#include "owl_math.h"

// change these to match your configuration

#define MARKER_COUNT 4

#define SERVER_NAME "169.229.222.231"
#define INIT_FLAGS 0

float RIGID_BODY[MARKER_COUNT][3] = {
  { 0, 300,  0},
  { 300, 0,  0},
  {-300, 0,  0},
  { 0,  76, -200}
};

void owl_print_error(const char *s, int n);

void copy_p(const float *a, float *b) { for(int i = 0; i < 7; i++) b[i] = a[i]; }
void print_p(const float *p) { for(int i = 0; i < 7; i++) printf("%f ", p[i]); }

int main()
{
  OWLRigid rigid;
  OWLMarker markers[32];
  OWLCamera cameras[32];

  int tracker;

  if(owlInit(SERVER_NAME, INIT_FLAGS) < 0) return 0;

  // create rigid body tracker 0
  tracker = 0;  
  owlTrackeri(tracker, OWL_CREATE, OWL_RIGID_TRACKER);
  
  // set markers
  for(int i = 0; i < MARKER_COUNT; i++)
    {
      // set markers
      owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i);

      // set marker positions
      owlMarkerfv(MARKER(tracker, i), OWL_SET_POSITION, RIGID_BODY[i]);
    }

  // activate tracker
  owlTracker(tracker, OWL_ENABLE);

  // flush requests and check for errors
  if(!owlGetStatus())
    {
      owl_print_error("error in point tracker setup", owlGetError());
      return 0;
    }

  // set default frequency
  owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);
  
  // start streaming
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);

  // main loop
  while(1)
    {
      int err;

      // get the rigid body
      int n = owlGetRigids(&rigid, 1);

      // get the rigid body markers
      //  note: markers have to be read,
      //  even if they are not used
      int m = owlGetMarkers(markers, 32);

      // get cameras
      int o = owlGetCameras(cameras, 32);

      // check for error
      if((err = owlGetError()) != OWL_NO_ERROR)
	{
	  owl_print_error("error", err);
	  break;
	}

      // no data yet
      if(n == 0 || o == 0) continue;

      if(n > 0)
	{
	  printf("%d rigid body, %d markers, %d cameras:\n", n, m, o);
	  if(rigid.cond > 0)
	    {
              float inv_pose[7];
              copy_p(rigid.pose, inv_pose);
              invert_p(inv_pose);

              printf("\nRigid: ");
              print_p(rigid.pose);

              printf("\nCameras: ");
              for(int i = 0; i < o; i++)
                {
                  // multiply each camera's pose by inverse of rigid pose
                  float cam_pose[7];
                  copy_p(cameras[i].pose, cam_pose);
                  mult_pp(inv_pose, cam_pose, cameras[i].pose);
                  
                  print_p(cameras[i].pose);
                  printf("\n");
                }

	      printf("\n");
	    }
	  printf("\n");
	}
    }
  
  // cleanup
  owlDone();
}

void owl_print_error(const char *s, int n)
{
  if(n < 0) printf("%s: %d\n", s, n);
  else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
  else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
  else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
  else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
  else printf("%s: 0x%x\n", s, n);
}
