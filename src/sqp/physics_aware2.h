#pragma once

#include "sqp_algorithm.h"
#include "simulation/environment.h"

Eigen::VectorXd toXYZROD(const btTransform& tf);
btTransform fromXYZROD(const Eigen::VectorXd& xyzrod);

class PushCollision: public LinearizedCostFunc {
  OpenRAVE::RobotBasePtr m_robot;
  btRigidBody* m_body;
  const btConvexShape* m_linkBodyShape;
  const btConvexShape* m_bodyShape;
  GRBQuadExpr m_obj;
  btRigidBody* m_linkBody;
  std::vector<int> m_dofInds;
  ComboStateSetterPtr m_setter;
  double m_exactObjective;

public:

  PushCollision(RaveRobotObjectPtr robot, OpenRAVE::KinBody::LinkPtr link, btRigidBody* body, std::vector<int> dofInds);
  void removeVariablsAndConstraints();
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getApproxCost() {
    return m_obj.getValue();
  }
  double getCachedCost() {
    return m_exactObjective;
  }
  double getCost();
  void getCostAndGradient(double& cost, Eigen::MatrixXd& gradient);


  double calcViolCost(const Eigen::VectorXd& dofs0, const Eigen::VectorXd& dofs1);
  double calcPenCost(const Eigen::VectorXd& dofs);

};

