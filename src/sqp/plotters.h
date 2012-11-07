#pragma once
#include "utils_sqp.h"
#include "sqp_fwd.h"
#include "simulation/simulation_fwd.h"
#include "utils/logging.h"
#include "collisions.h"
#include <osg/Group>


void pauseScene();

class PlotHandles {
	// in a plotting function, return plot handles
	// it'll add the plots to the environment when it's created
	// and then remove them when it's destroyed
	std::vector<EnvironmentObjectPtr> m_plots;
	EnvironmentPtr m_env;
public:
	PlotHandles(const std::vector<EnvironmentObjectPtr>& plots, EnvironmentPtr env);
	~PlotHandles();
};

class TrajPlotter {
public:
  typedef boost::shared_ptr<TrajPlotter> Ptr;
  virtual void plotTraj(const Eigen::MatrixXd& traj) = 0;
  virtual void clear() {}
  virtual ~TrajPlotter() {}
};

class TrajChangePlotter {
public:
	virtual void plotTrajs(const Eigen::MatrixXd& traj0, const Eigen::MatrixXd& traj1) = 0;
	virtual ~TrajChangePlotter() {}
};

class GripperAxesPlotter : public TrajPlotter {
	PlotHandlesPtr m_handles;
	RobotManipulatorPtr m_manip;
	int m_startCol;
	EnvironmentPtr m_env;
	float m_size;
	
public:
	GripperAxesPlotter(RobotManipulatorPtr, int startCol, EnvironmentPtr env, float size);
	void plotTraj(const Eigen::MatrixXd& traj);
	void clear();
	~GripperAxesPlotter();
};

class TwoTrajPlotters : public TrajChangePlotter {
	TrajPlotterPtr m_plotter0, m_plotter1;
public:
	TwoTrajPlotters(TrajPlotterPtr plotter0, TrajPlotterPtr plotter1) : m_plotter0(plotter0), m_plotter1(plotter1) {}
	void plotTrajs(const Eigen::MatrixXd& traj0, const Eigen::MatrixXd& traj1) {
		m_plotter0->plotTraj(traj0);
		m_plotter1->plotTraj(traj1);
	}
};

class GripperPlotter : public TrajPlotter {
public:
  std::vector<FakeGripperPtr> m_grippers;
  PlotCurvePtr m_curve;
  RobotManipulatorPtr m_rrom;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  int m_decimation;
  void plotTraj(const Eigen::MatrixXd& traj);
  void clear();
  GripperPlotter(RobotManipulatorPtr, Scene*, int decimation = 1);
  ~GripperPlotter();
  void setNumGrippers(int n);
};

class StatePlotter : public TrajPlotter {
  StateSetterPtr m_ss;
  Scene* m_scene;
public:
  StatePlotter(StateSetterPtr ss, Scene* scene) :
    m_ss(ss), m_scene(scene) {
  }
  void plotTraj(const Eigen::MatrixXd& traj);
};

class ArmPlotter : public TrajPlotter {
  typedef boost::shared_ptr<ArmPlotter> Ptr;
  RobotManipulatorPtr m_rrom;
  std::vector<BulletObjectPtr> m_origs;
  BasicArray<FakeObjectCopyPtr> m_fakes;
  osg::ref_ptr<PlotCurve> m_curve;
  PlotAxesPtr m_axes;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  int m_decimation;
  BulletRaveSyncherPtr m_syncher;
public:
  void plotTraj(const Eigen::MatrixXd& traj);
  ArmPlotter(RobotManipulatorPtr rrom, Scene* scene, int decimation = 1);
  void init(RobotManipulatorPtr, const std::vector<BulletObjectPtr>&, Scene*, int decimation);
  void setLength(int n);
  void clear() {
    setLength(0);
  }
  ~ArmPlotter();
};

void interactiveTrajPlot(const Eigen::MatrixXd& traj, RobotManipulatorPtr arm, Scene* scene);
void clearCollisionPlots();
void plotCollisions(const TrajCartCollInfo& trajCartInfo, double safeDist);

void adjustWorldTransparency(float);
