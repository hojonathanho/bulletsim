/* thread_socket_interface.h */

#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <string>
#include <cstdio>
#include "UDPSocket.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <Eigen/Core>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#define IP "128.32.37.98"
#define RECEIVE_PORT 9000
#define SEND_PORT 9001

using namespace std;

void parse(string buf, vector<string> &vect);

void connectionInit();

bool getDeviceState (Vector3d& start_proxy_pos, Matrix3d& start_proxy_rot, bool start_proxybutton[], Vector3d& end_proxy_pos, Matrix3d& end_proxy_rot, bool end_proxybutton[]);

void sendDeviceState (const Vector3d& start_feedback_pos, bool start_feedback_enabled, const Vector3d& end_feedback_pos, bool end_feedback_enabled);

void getDeviceState (double start_proxyxform[], bool start_proxybutton[], double end_proxyxform[], bool end_proxybutton[]);

void sendDeviceState (double start_feedback_pos[], bool start_feedback_enabled, double end_feedback_pos[], bool end_feedback_enabled);

//void getDeviceState (Vector3d &leftPosition, Matrix3d &leftRotation, Vector3d &rightPosition, Matrix3d &rightRotation);
