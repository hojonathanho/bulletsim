// helper functions to create bullet objects from simpler data
#include <vector>
#include "environment.h"
#include "basicobjects.h"
using namespace std;


struct Verts2BoxPars {
  vector<btVector3> halfExtents;
  btVector3 origin;
  Verts2BoxPars(const vector<btVector3>& verts>, float thickness);
};
