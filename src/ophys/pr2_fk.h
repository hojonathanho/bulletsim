#pragma once

#include "ophys_common.h"
#include "simulation/openravesupport.h"

class PR2FastFK {
public:
  PR2FastFK(bool left);
  void calibrate(RobotManipulator::Ptr manip);
  Eigen::Vector3d jointToPos(const ophys::Vector7d &j);

private:
  Eigen::Vector3d m_offset;
  Eigen::Vector3d m_rot;
};
