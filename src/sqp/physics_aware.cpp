#include "physics_aware.h"
#include "sqp_algorithm.h"
#include "utils/logging.h"
#include "simulation/util.h"
#include "simulation/simplescene.h"
#include "config_sqp.h"
using namespace std;
using namespace Eigen;

inline float sq(float x) {return x*x;}


double PushObject::simulateTraj(const Eigen::MatrixXd& traj) {

  static RaveInstancePtr copyRave(new RaveInstance(*util::getGlobalScene()->rave, OpenRAVE::Clone_Bodies));
  Fork::Ptr fork(new Fork(m_env, copyRave));
  m_env->osg->root->addChild(fork->env->osg->root.get());

#if 0
  static bool firstTime=true;
  static Scene scene2;
  if (firstTime) {
    scene2.startViewer();
    firstTime=false;
  }
  scene2.env = fork->env;
  scene2.rave = fork->rave;
  scene2.osg = fork->env->osg;
  scene2.bullet = fork->env->bullet;
#endif

  for (int iStep=0; iStep < traj.rows(); ++iStep) {
    boost::dynamic_pointer_cast<RaveRobotObject>(fork->forkOf(m_robot))->setDOFValues(m_dofInds, toDoubleVec(traj.row(iStep)));
    fork->env->step(1./traj.rows(), 1, 1./traj.rows());
//    util::getGlobalScene()->draw();
#if 0
    scene2.step(.05, 5, .01);
    scene2.idle(true);
#endif
  }

  BulletObject::Ptr objCopy = boost::dynamic_pointer_cast<BulletObject>(fork->forkOf(m_object));
  ENSURE(objCopy);

  btTransform objTransform = objCopy->rigidBody->getCenterOfMassTransform();

  m_env->osg->root->removeChild(fork->env->osg->root);


  return m_posCoeff*(m_target.getOrigin()-objTransform.getOrigin()).length2()/sq(METERS) +
    m_rotCoeff*sq(m_target.getRotation().angle(objTransform.getRotation()));
}

double PushObject::simulateTraj2(const Eigen::MatrixXd& traj, bool drawing) {
  btTransform origTF = m_object->rigidBody->getCenterOfMassTransform();
  for (int iStep=0; iStep < traj.rows(); ++iStep) {
    m_robot->setDOFValues(m_dofInds, toDoubleVec(traj.row(iStep)));
    if (drawing) {
      util::getGlobalScene()->step(.05, 1, .05);
      util::getGlobalScene()->idle(true);
    }
    else m_env->step(.05, 1, .05);
  }
  btTransform objTransform = m_object->rigidBody->getCenterOfMassTransform();
  m_object->rigidBody->setCenterOfMassTransform(origTF);
  m_object->rigidBody->setAngularVelocity(btVector3(0,0,0));
  m_object->rigidBody->setLinearVelocity(btVector3(0,0,0));


  return m_posCoeff*(m_target.getOrigin()-objTransform.getOrigin()).length2()/sq(METERS) +
    m_rotCoeff*sq(m_target.getRotation().angle(objTransform.getRotation()));
}



MatrixXd getSinBasis(const VectorXd& t, int maxFreq) {
  MatrixXd out(t.size(), 2*maxFreq+1);
  out.col(0).setConstant(1./sqrtf(2.));
  for (int f=1; f <= maxFreq; ++f) {
    out.col(2*f-1) = (SIMD_PI*f*t).array().sin();
    out.col(2*f) = (SIMD_PI*f*t).array().cos();
  }  
  return out;
}

Eigen::MatrixXd PushObject::getGradient();

void PushObject::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
  VectorXd& times = m_problem->m_times;
  MatrixXd perts_tk = getSinBasis(times/times.maxCoeff(), fmin(6, times.size()/2));
  MatrixXd pinvperts_tk = perts_tk * (perts_tk.transpose() * perts_tk).inverse();

  m_exactObjective = simulateTraj2(traj, true); // current value
  LOG_INFO_FMT("current val: %.3f", m_exactObjective);
  
  MatrixXd dy_jk(traj.cols(), perts_tk.cols());
  double eps = 3e-4; // scale for joint angle change
  
  Matrix3d A;
  A << sq(eps/2), eps/2, 1,
        0, 0, 1,
      sq(eps/2),  -eps/2,   1;
  Matrix3d Ainv = A.inverse();
  MatrixXd grad_tj(traj.rows(), traj.cols());

  m_obj = GRBQuadExpr(0);

  
  for (int j = 0; j < traj.cols(); ++j) {

    VarVector v;
    VectorXd vactual(traj.rows()-1);
    for (int t=1; t < traj.rows(); ++t) {
      v.push_back(m_problem->m_trajVars.at(t,j));
      vactual(t-1) = traj(t,j);
    }

    for (int k = 0; k < perts_tk.cols(); ++k) {
      MatrixXd newTraj = traj;
      newTraj.col(j) = traj.col(j) + (eps/2)*perts_tk.col(k);
      double plusVal = simulateTraj2(newTraj, false);
      newTraj.col(j) = traj.col(j) - (eps/2)*perts_tk.col(k);
      double minusVal = simulateTraj2(newTraj, false);
      LOG_DEBUG_FMT("joint %i, basis %i, pert vals: %.4e %.4e ", j, k, plusVal-m_exactObjective,minusVal-m_exactObjective);
      dy_jk(j,k) = (plusVal - minusVal)/eps;
      Vector3d y;
      y << plusVal-m_exactObjective, 0, minusVal - m_exactObjective;
      Vector3d abc = Ainv*y;
      GRBLinExpr q;
      int T = traj.rows()-1;
      VectorXd pertVec=perts_tk.block(1,k,T, 1);
      q.addTerms(pertVec.data(),v.data(), T);
      double qactual = pertVec.dot(vactual);
      m_obj += fmax(abc(0),0)*(q-qactual)*(q-qactual) + abc(1)*(q-qactual);
    }
    
//    grad_tj.col(j) = pinvperts_tk * dy_jk.row(j).transpose();
  }
  m_obj += m_exactObjective;

//  cout << "dy_jk:" << endl;
//  cout << dy_jk << endl;
//  cout << "grad_tj:" << endl;
//  cout << grad_tj << endl;

  
//  m_obj.addTerms(grad_tj.data()+7, m_problem->m_trajVars.m_data.data()+7, (traj.rows()-1)*traj.cols());
//  m_obj += m_exactObjective - (grad_tj.middleRows(1, traj.rows()-1).array() * traj.middleRows(1, traj.rows()-1).array()).sum();
  objective += m_obj;
}
