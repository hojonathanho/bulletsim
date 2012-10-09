#pragma once
#include <boost/shared_ptr.hpp>
class HydraPR2Teleop;
typedef boost::shared_ptr<HydraPR2Teleop> HydraPR2TeleopPtr;
class PR2SoftBodyGripper;
typedef boost::shared_ptr<PR2SoftBodyGripper> PR2SoftBodyGripperPtr;
class PR2Manager;
typedef boost::shared_ptr<PR2Manager> PR2ManagerPtr;
class Grab;
typedef boost::shared_ptr<Grab> GrabPtr;
class Monitor;
typedef boost::shared_ptr<Monitor> MonitorPtr;
class MonitorForGrabbing;
typedef boost::shared_ptr<MonitorForGrabbing> MonitorForGrabbingPtr;
class SoftMonitorForGrabbing;
typedef boost::shared_ptr<SoftMonitorForGrabbing> SoftMonitorForGrabbingPtr;
