#pragma once
#include "utils_sqp.h"
#include "traj_costs.h"
#include "kinematics_utils.h"
#include "simulation/openravesupport.h"
#include "simulation/fake_gripper.h"
#include "simulation/plotting.h"
#include "gurobi_c++.h"
#include "sqp_fwd.h"


GRBEnv* getGRBEnv();

class Scene;

typedef BasicArray<GRBVar> VarArray;
typedef std::vector<GRBVar> VarVector;
typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;

ArmCCEPtr makeArmCCE(RaveRobotObject::Manipulator::Ptr arm, RaveRobotObject::Ptr robot, btDynamicsWorld*);

class PlanningProblem { // fixed size optimization problem
public:
  VarArray m_trajVars;
  Eigen::MatrixXd m_currentTraj;
  VectorXb m_optMask;
  boost::shared_ptr<GRBModel> m_model;
  std::vector<TrajPlotterPtr> m_plotters;
  std::vector<ProblemComponentPtr> m_comps;
  bool m_initialized;
  PlanningProblem();
  void addComponent(ProblemComponentPtr comp);
  void removeComponent(ProblemComponentPtr comp);
  void initialize(const Eigen::MatrixXd& initTraj, bool endFixed);

  void doIteration();
  void optimize(int maxIter);
  void addPlotter(TrajPlotterPtr plotter) {m_plotters.push_back(plotter);}

};

class ProblemComponent {
public:
  enum CompAttrs {
    RELAXABLE=1,
    HAS_CONSTRAINT=2,
    HAS_COST=4
  };
  unsigned int m_attrs;
  typedef boost::shared_ptr<ProblemComponent> Ptr;

  PlanningProblem* m_problem;

  ProblemComponent() {

  }
  virtual void onAdd() {}
  virtual void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) = 0;
  virtual double getValue() {return 0;}
  virtual void relax() {}
  virtual void onRemove() = 0;
};

class CollisionCost: public ProblemComponent {
protected:
  std::vector<GRBVar> m_vars;
  std::vector<GRBConstr> m_cnts;
  GRBLinExpr m_obj;
  CollisionCostEvaluatorPtr m_cce;
  double m_safeDistMinusPadding;
  double m_coeff;
  bool m_startFixed, m_endFixed;
public:
  CollisionCost(CollisionCostEvaluatorPtr cce, double safeDistMinusPadding, double coeff) :
    m_cce(cce), m_safeDistMinusPadding(safeDistMinusPadding), m_coeff(coeff) {}
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getValue() {return m_obj.getValue();}
  void onRemove();
  void setCoeff(double coeff) {m_coeff = coeff;}
};

class CollisionConstraint: public ProblemComponent {
protected:
  std::vector<GRBConstr> m_cnts;
  CollisionCostEvaluatorPtr m_cce;
  double m_safeDistMinusPadding;
  double m_coeff;
  bool m_startFixed, m_endFixed;
public:
  CollisionConstraint(bool startFixed, bool endFixed, CollisionCostEvaluatorPtr cce, double safeDistMinusPadding) :
    m_startFixed(startFixed), m_endFixed(endFixed), m_cce(cce), m_safeDistMinusPadding(safeDistMinusPadding) {
    m_attrs |= RELAXABLE;
  }
  void onAdd();
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  void onRemove();
  void relax();
};

class LengthConstraintAndCost : public ProblemComponent {
  // todo: set this
  GRBQuadExpr m_obj;
  std::vector<GRBConstr> m_cnts;
  bool m_startFixed, m_endFixed;
  Eigen::VectorXd m_maxStepMvmt;
  double m_coeff;
public:
  LengthConstraintAndCost(bool startFixed, bool endFixed, const Eigen::VectorXd& maxStepMvmt, double coeff) :
    m_startFixed(startFixed), m_endFixed(endFixed), m_maxStepMvmt(maxStepMvmt), m_coeff(coeff) {}

  void onAdd();
  // init is called by planning problem
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
    objective += m_coeff*m_obj;
  }
  double getValue() {
    return m_obj.getValue();
  }
  void onRemove();
  void setCoeff(double coeff) {m_coeff = coeff;}
  // destroy is called by planning problem
};

class JointBounds: public ProblemComponent {
  // todo: set these in constructor
  Eigen::VectorXd m_jointLowerLimit;
  Eigen::VectorXd m_jointUpperLimit;
  Eigen::VectorXd m_maxDiffPerIter;
  bool m_startFixed, m_endFixed;
public:
  JointBounds(bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter, OpenRAVE::RobotBase::ManipulatorPtr manip) :
    m_startFixed(startFixed), m_endFixed(endFixed), m_maxDiffPerIter(maxDiffPerIter) {
    vector<int> armInds = manip->GetArmIndices();
    m_jointLowerLimit.resize(armInds.size());
    m_jointUpperLimit.resize(armInds.size());
    vector<double> ul, ll;
    for (int i=0; i < armInds.size(); ++i) {
      manip->GetRobot()->GetJointFromDOFIndex(armInds[i])->GetLimits(ll, ul);
      m_jointLowerLimit(i) = ll[0];
      m_jointUpperLimit(i) = ul[0];
    }
  }


  void onAdd();
  void onRemove() {}
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
};

#if 0
class CartesianPoseCost: public ProblemComponent {
  GRBQuadExpr m_obj;
  OpenRAVE::RobotBasePtr m_robot;
  OpenRAVE::RobotBase::ManipulatorPtr m_manip;
  Eigen::Vector3d m_posTarg;
  Eigen::Vector4d m_rotTarg;
  VarVector m_timestepVars;
  double m_posCoeff, m_rotCoeff;
  int m_timestep;
public:
  CartesianPoseCost(OpenRAVE::RobotBasePtr robot, OpenRAVE::RobotBase::ManipulatorPtr manip,
      const btTransform& target, int timestep, double posCoeff, double rotCoeff) :
        m_robot(robot),
        m_manip(manip),
        m_posTarg(toVector3d(target.getOrigin())),
        m_rotTarg(toVector4d(target.getRotation())),
        m_posCoeff(posCoeff),
        m_rotCoeff(rotCoeff),
        m_timestep(timestep) {}

  void onAdd();
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getValue() {
    return m_obj.getValue();
  }

};
#endif

Eigen::VectorXd defaultMaxStepMvmt(const Eigen::MatrixXd& traj);
Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, int nSteps);
Eigen::MatrixXd makeTraj(RaveRobotObject::Manipulator::Ptr manip, const Eigen::VectorXd& startJoints, const btTransform endTransform, int nSteps);
Eigen::MatrixXd makeTraj(RaveRobotObject::Manipulator::Ptr manip, const std::vector<btTransform>& transforms);
void updateTraj(const VarArray& trajVars, const VectorXb& optmask, Eigen::MatrixXd& traj);


class TrajPlotter {
public:
  typedef boost::shared_ptr<TrajPlotter> Ptr;
  virtual void plotTraj(const Eigen::MatrixXd& traj) = 0;
  virtual void clear() {}
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
  GripperPlotter(RaveRobotObject::Manipulator::Ptr, Scene*, int decimation=1);
  ~GripperPlotter();
  void setNumGrippers(int n);
};

class ArmPlotter : public TrajPlotter {
public:
  typedef boost::shared_ptr<ArmPlotter> Ptr;
  RaveRobotObject::Manipulator::Ptr m_rrom;
  std::vector<BulletObject::Ptr> m_origs;
  BasicArray<FakeObjectCopy::Ptr> m_fakes;
  PlotCurve::Ptr m_curve;
  PlotAxes::Ptr m_axes;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  int m_decimation;
  BulletRaveSyncher* m_syncher;
  void plotTraj(const Eigen::MatrixXd& traj);
  ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, std::vector<KinBody::LinkPtr>& links, BulletRaveSyncher& syncher, int decimation=1);
  ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, const std::vector<BulletObject::Ptr>& origs, Scene* scene, BulletRaveSyncher*syncher, int decimation);
  void init(RaveRobotObject::Manipulator::Ptr, const std::vector<BulletObject::Ptr>&, Scene*, BulletRaveSyncher*, int decimation);
  void setLength(int n);
  void clear() {setLength(0);}
  ~ArmPlotter();
};

void interactiveTrajPlot(const Eigen::MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, BulletRaveSyncher* syncher, Scene* scene);




