#pragma once
#include <utility>
#include <vector>
#include <openrave/openrave.h>
#include <sensor_msgs/JointState.h>
using namespace std;

typedef pair< vector<double>, vector<int> > ValuesInds;

ValuesInds getValuesInds(const vector<double>& joints); 
// gives the joint values and joint indices in the format that
// openrave requires

void setupROSRave(const OpenRAVE::RobotBasePtr& robot, const sensor_msgs::JointState& msg);
void setupDefaultROSRave();
void ROSRaveReset(const OpenRAVE::RobotBasePtr& robot, const sensor_msgs::JointState& msg);
