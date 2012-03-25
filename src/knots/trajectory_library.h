#pragma once
#include "utils_python.h"

std::vector<RobotAndRopeState> fromNumpy(const npMatrixf&);
std::vector<RobotAndRopeState> fromNumpy(const py::object);

npMatrixf toNumpy(const std::vector<RobotAndRopeState>& rars);
py::object toNumpy1(const std::vector<RobotAndRopeState>& rars);

npMatrixf ropeToNumpy(const std::vector<btVector3>& pts);
py::object ropeToNumpy1(const std::vector<btVector3>& pts);
