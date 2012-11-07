#pragma once
#include "simulation/simulation_fwd.h"
#include "sqp_fwd.h"
#include <Eigen/Dense>
#include <openrave/openrave.h>

class btRigidBody;

class StateSetter {
public:
  virtual int getNumDof()=0;
  virtual void setState(const Eigen::VectorXd&)=0;
  virtual Eigen::VectorXd getState()=0;
  virtual ~StateSetter() {
  }
};

class RobotJointSetter : public StateSetter {
  OpenRAVE::RobotBasePtr m_robot;
  std::vector<int> m_dofInds;
  BulletRaveSyncherPtr m_brs;
  bool m_useAffine;
public:
  RobotJointSetter(RaveRobotObjectPtr robot, const std::vector<int>& dofInds, bool useAffine=false);
  void setState(const Eigen::VectorXd&);
  Eigen::VectorXd getState();
  int getNumDof() {
    return m_dofInds.size()+m_useAffine*3;
  }
};

class ComboStateSetter : public StateSetter {
  std::vector<StateSetterPtr> m_setters;
public:
  void addSetter(StateSetterPtr setter) {
    m_setters.push_back(setter);
  }
  void setState(const Eigen::VectorXd& state);
  Eigen::VectorXd getState();
  int getNumDof();
};

class ObjectPoseSetter : public StateSetter {
  btRigidBody* m_body;
public:
  int getNumDof() {
    return 6;
  }
  ObjectPoseSetter(btRigidBody* body) :
    m_body(body) {
  }
  void setState(const Eigen::VectorXd&);
  Eigen::VectorXd getState();
};

class ScopedStateSave {
  StateSetter* m_setter;
  Eigen::VectorXd m_save;
public:
  ScopedStateSave(StateSetter* setter) :
    m_setter(setter), m_save(setter->getState()) {
  }
  ~ScopedStateSave() {
    m_setter->setState(m_save);
  }
};
