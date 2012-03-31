#pragma once
#include <boost/filesystem.hpp>
#include <btBulletDynamicsCommon.h>
#include <vector>

using boost::shared_ptr;
namespace fs = boost::filesystem;

extern fs::path KNOT_DATA;

static const int N_CTRL_PTS = 100;

struct RobotState{
  btTransform leftPose, rightPose;
  float leftGrip, rightGrip;
};

struct RobotAndRopeState {
  btTransform leftPose, rightPose;
  float leftGrip, rightGrip;
  int leftGrab, rightGrab;
  std::vector<btVector3> ctrlPts;
};

