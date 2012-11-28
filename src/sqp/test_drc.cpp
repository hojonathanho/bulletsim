#include "drc_costs.h"
#include "simulation/simplescene.h"
#include "simulation/openravesupport.h"
#include "simulation/config_bullet.h"
#include "sqp/config_sqp.h"
#include "sqp/state_setter.h"
#include "sqp/plotters.h"
using namespace util;

class DRCJointSetter : public StateSetter {
  RaveRobotObjectPtr m_rro;
  std::vector<int> m_dofInds;
  BulletRaveSyncherPtr m_brs;
public:
  DRCJointSetter(RaveRobotObjectPtr robot);
  void setState(const Eigen::VectorXd&);
  Eigen::VectorXd getState();
  int getNumDof() {
    return DRCInfo::NUM_DOF;
  }
};

DRCJointSetter::DRCJointSetter(RaveRobotObjectPtr robot) : m_rro(robot) {}
void DRCJointSetter::setState(const Eigen::VectorXd& state) {
	cout << "state: " << state.transpose().block(0,0,1,DRCInfo::NUM_DOF) << endl;
	m_rro->robot->SetActiveDOFValues(toDoubleVec(state.block(0,0,DRCInfo::NUM_DOF,1)));
	m_rro->updateBullet();
}
VectorXd DRCJointSetter::getState() {
  vector<double> x;
  m_rro->robot->GetActiveDOFValues(x);
  return toVectorXd(x);
}

void setVariableBounds(VarArray& vars, RobotBasePtr robot) {
  
  vector<double> posLower, posUpper, torqueMag;
  robot->GetDOFLimits(posLower, posUpper);
  robot->GetDOFTorqueLimits(torqueMag);
  
  cout << "lower: " << posLower << endl;
  cout << "upper: " << posUpper << endl;


  assert (vars.cols() == DRCInfo::I_END);

  for (int i=0; i < vars.rows(); ++i) {
    for (int j=0; j < DRCInfo::NUM_JOINTS; ++j) {
      vars(i,j).set(GRB_DoubleAttr_LB, posLower[j]);
      vars(i,j).set(GRB_DoubleAttr_UB, posUpper[j]);
    }
    for (int j=0; j < DRCInfo::NUM_JOINTS; ++j) {
      vars(i,j+DRCInfo::I_TORQUE).set(GRB_DoubleAttr_LB, 0);
      vars(i,j+DRCInfo::I_TORQUE).set(GRB_DoubleAttr_UB, 0);
    }
    for (int j=DRCInfo::I_LCOP; j < DRCInfo::I_LFORCE; ++j) {
    	vars(i,j).set(GRB_DoubleAttr_LB, 0);
    	vars(i,j).set(GRB_DoubleAttr_UB, 0);
    }
    for (int j=DRCInfo::I_LFORCE; j < DRCInfo::I_RCOP; ++j) {
    	vars(i,j).set(GRB_DoubleAttr_LB, 0);
    	vars(i,j).set(GRB_DoubleAttr_UB, 0);
    }
    for (int j=DRCInfo::I_RCOP; j < DRCInfo::I_RFORCE; ++j) {
    	vars(i,j).set(GRB_DoubleAttr_LB, 0);
    	vars(i,j).set(GRB_DoubleAttr_UB, 0);
    }
    for (int j=DRCInfo::I_RFORCE; j < DRCInfo::I_END; ++j) {
    	vars(i,j).set(GRB_DoubleAttr_LB, 0);
    	vars(i,j).set(GRB_DoubleAttr_UB, 0);
    }
  }
  
}

int main(int argc, char* argv[]) {

	Parser parser;
	parser.addGroup(SQPConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(GeneralConfig());
	parser.read(argc, argv);
	Scene scene;
	util::setGlobalScene(&scene);
	util::setGlobalEnv(scene.env);
	initializeGRB();

	scene.env->remove(scene.env->objects[0]);
	scene.startViewer();
	Load(scene.env, scene.rave, "/home/joschu/Proj/drc/gfe.xml");
	Load(scene.env, scene.rave, "/home/joschu/Proj/drc/floorwalls.xml");
	RaveRobotObjectPtr rro = getRobots(scene.env, scene.rave)[0];
	AABB aabb = rro->robot->ComputeAABB();
	double zmin = aabb.pos[2] - aabb.extents[2];
	OpenRAVE::Transform tf = rro->robot->GetTransform();
	tf.trans.z -= zmin;
	rro->robot->SetTransform(tf);
	rro->updateBullet();
	setAllDofsActive(rro->robot);

	boost::shared_ptr<DRCJointSetter> robotSetter(new DRCJointSetter(rro));



  TrajOptimizer opt;
  opt.addCost(CostPtr(new EulerLagrangeCost(&opt, rro->robot, 10)));
//  opt.addConstraint(ConstraintPtr(new StayAboveGround(&opt, rro->robot, 10)));
//  opt.setTrustRegion(TrustRegionPtr(new SoftTrustRegion(&opt, 999)));
  vector<double> posLower, posUpper;
  rro->robot->GetDOFLimits(posLower, posUpper);
  VectorXd upper(DRCInfo::I_END), lower(DRCInfo::I_END);
  upper.setZero();
  lower.setZero();
  upper.topRows(DRCInfo::NUM_JOINTS) = toVectorXd(posUpper);
  lower.topRows(DRCInfo::NUM_JOINTS) = toVectorXd(posLower);
  upper.middleRows(DRCInfo::NUM_JOINTS, 7).setConstant(+GRB_INFINITY);
  lower.middleRows(DRCInfo::NUM_JOINTS, 7).setConstant(-GRB_INFINITY);


  opt.setTrustRegion(TrustRegionPtr(new JointBounds(&opt, VectorXd::Ones(DRCInfo::I_END)*.01, lower, upper)));

  boost::shared_ptr<DRCJointSetter> setter(new DRCJointSetter(rro));
  StatePlotterPtr statePlotter(new StatePlotter(setter,&scene));
  opt.m_plotters.push_back(statePlotter);

  const int TRAJ_LENGTH = 20;
  MatrixXd initTraj(TRAJ_LENGTH, DRCInfo::I_END);
  initTraj.setZero();
  vector<double> curvals;
  rro->robot->GetActiveDOFValues(curvals);
  for (int i=0; i < TRAJ_LENGTH; ++i) {
  	for (int j=0; j < DRCInfo::NUM_DOF; ++j) {
  		initTraj(i,j) = curvals[j];
  	}
  }
  opt.initialize(initTraj, arange(TRAJ_LENGTH)*DT1);



//  setVariableBounds(opt.m_vars, rro->robot);
  setStartFixed(opt);


	scene.step(0);
	pauseScene();
	try {
	  opt.optimize();
	}
	catch (GRBException& err) {
		cout << err.getMessage() << endl;
		throw;
	}
}
