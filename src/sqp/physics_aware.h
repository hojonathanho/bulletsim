#pragma once
#include "sqp_algorithm.h"
#include "simulation/environment.h"

class PushObject: public ProblemComponent {
public:
  std::vector<GRBVar> m_vars;
  GRBQuadExpr m_obj;


  BulletObject::Ptr m_object;
  btTransform m_target;
  RaveRobotObject::Ptr m_robot;
  Environment::Ptr m_env;
  std::vector<int> m_dofInds;
  double m_posCoeff;
  double m_rotCoeff;

  double m_exactObjective;  
  ArmPlotterPtr m_plotter;
  double simulateTraj(const Eigen::MatrixXd& traj);
  double simulateTraj2(const Eigen::MatrixXd& traj, bool drawing);

  PushObject(BulletObject::Ptr obj, btTransform target, RaveRobotObject::Ptr robot, Environment::Ptr env, const vector<int>& dofInds, double posCoeff, double rotCoeff) :
    m_object(obj), m_target(target), m_robot(robot), m_env(env), m_dofInds(dofInds), m_posCoeff(posCoeff), m_rotCoeff(rotCoeff) {
      m_attrs = HAS_COST;
//      m_plotter.reset(new ArmPlotter(robot->getManipByIndex(1), util::getGlobalScene(), SQPConfig::plotDecimation));
    }
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double calcApproxObjective() {return m_obj.getValue();}
  double calcExactObjective() {return m_exactObjective;}
};
