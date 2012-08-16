#pragma once
#include "utils_scp.h"
#include "traj_costs.h"
#include "kinematics_utils.h"
#include "simulation/openravesupport.h"
diana
class CollisionMap {
  // adds/removes collision objects from bullet world
public:
  typedef boost::shared_ptr<CollisionMap> Ptr;
  void clearObstacles();
  void addBoxesToMap(const std::vector<btVector3>& points, float size);  
};


class ArmPlanningProblem {
  // represents a single optimization problem and stores the solution
  ArmTrajPlotter::Ptr m_atp;
  CollisionMap::Ptr m_cm;
  ArmPlanningProblem();
  ~ArmPlanningProblem();
  void setPlotter(ArmTrajPlotter::Ptr atp) {m_atp = atp;}
};

class GetArmToJointGoal : public ArmPlanningProblem {
  Eigen::MatrixXf m_currentTraj;
  float m_currentObjective;
  Eigen::VectorXf m_maxStepMvmt;
  Eigen::VectorXf m_maxDiffPerIter;
  int m_nSteps;
  GRBEnvironment* m_grbenv;
  GRBModel* m_model;
  BasicArray<GRGVar >m_trajVars;
  CollisionCostEvaluator* m_cce;
  ArmTrajPlotter* m_trajPlotter;
public:
  void setProblem();
  void doIteration();
  void optimize(int maxIter=100);
};

class ArmTrajPlotter {
public:
  typedef boost::shared_ptr<ArmTrajPlotter> Ptr;
  std::vector<FakeGripper::Ptr> m_grippers;
  PlotCurve::Ptr m_curve;
  void plotTraj(const MatrixXf& traj);
  void clear();
  ArmTrajPlotter(RaveRobotObject::Manipulator::Ptr rrom, osg::Group*, int decimation=0);
  ~ArmTrajPlotter();
  void setNumGrippers(int n);  
}