#pragma once
#include "simulation/environment.h"
#include "utils_sqp.h"
#include "simulation/plotting.h"
#include "simulation/fake_gripper.h"
#include "sqp_fwd.h"
#include "simulation/simulation_fwd.h"
#include "traj_costs.h"

#include "utils/logging.h"

class PlotHandles {
	// in a plotting function, return plot handles
	// it'll add the plots to the environment when it's created
	// and then remove them when it's destroyed
	std::vector<EnvironmentObject::Ptr> m_plots;
	Environment::Ptr m_env;
public:
	PlotHandles(const std::vector<EnvironmentObject::Ptr>& plots, Environment::Ptr env) : m_plots(plots), m_env(env) {
		for (int i=0; i < m_plots.size(); ++i) m_env->add(plots[i]);
	}
	~PlotHandles() {
		for (int i=0; i < m_plots.size(); ++i) {
			m_env->remove(m_plots[i]);
		}
	}
};

class TrajPlotter {
public:
  typedef boost::shared_ptr<TrajPlotter> Ptr;
  virtual void plotTraj(const Eigen::MatrixXd& traj) = 0;
  virtual void clear() {
  }
  virtual ~TrajPlotter() {
  }
};

class TrajChangePlotter {
public:
	virtual void plotTrajs(const Eigen::MatrixXd& traj0, const Eigen::MatrixXd& traj1) = 0;
	virtual ~TrajChangePlotter() {
	}
};

class GripperAxesPlotter : public TrajPlotter {
	PlotHandlesPtr m_handles;
	RaveRobotObject::Manipulator::Ptr m_manip;
	int m_startCol;
	Environment::Ptr m_env;
	float m_size;
	
public:
	GripperAxesPlotter(RaveRobotObject::Manipulator::Ptr, int startCol, Environment::Ptr env, float size);
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
  std::vector<FakeGripper::Ptr> m_grippers;
  PlotCurve::Ptr m_curve;
  RaveRobotObject::Manipulator::Ptr m_rrom;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  int m_decimation;
  void plotTraj(const Eigen::MatrixXd& traj);
  void clear();
  GripperPlotter(RaveRobotObject::Manipulator::Ptr, Scene*, int decimation = 1);
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
  RaveRobotObject::Manipulator::Ptr m_rrom;
  std::vector<BulletObject::Ptr> m_origs;
  BasicArray<FakeObjectCopy::Ptr> m_fakes;
  PlotCurve::Ptr m_curve;
  PlotAxes::Ptr m_axes;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  int m_decimation;
  BulletRaveSyncherPtr m_syncher;
public:
  void plotTraj(const Eigen::MatrixXd& traj);
  ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, int decimation = 1);
  void init(RaveRobotObject::Manipulator::Ptr, const std::vector<BulletObject::Ptr>&, Scene*, int decimation);
  void setLength(int n);
  void clear() {
    setLength(0);
  }
  ~ArmPlotter();
};

void interactiveTrajPlot(const Eigen::MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, Scene* scene);
void clearCollisionPlots();
void plotCollisions(const TrajCartCollInfo& trajCartInfo, double safeDist);

void adjustWorldTransparency(float);
