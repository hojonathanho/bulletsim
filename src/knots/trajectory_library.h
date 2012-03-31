#pragma once
#include "utils_python.h"

std::vector<RobotAndRopeState> fromNumpy(const py::object);

py::object toNumpy1(const std::vector<RobotAndRopeState>& rars);

py::object ropeToNumpy1(const std::vector<btVector3>& pts);
