#ifndef _CAM_SYNC_H_
#define _CAM_SYNC_H_

class Scene;

#include <fstream>
#include <string>
using std::fstream;
using std::string;

// Attaches to a scene. If in record mode, the transform of the camera
// will be written to the specified file at every step.
// If in playback mode, the transform will be read and set at every step.

class CamSync {
public:
  enum Mode {
    DISABLED = 0,
    PLAYBACK,
    RECORD
  };

  CamSync(Scene &s);
  ~CamSync();
  void enable(Mode mode_, const string &path);

private:
  Scene &scene;
  fstream file;
  Mode mode;
  void cb();
};


#endif // _CAM_SYNC_H_
