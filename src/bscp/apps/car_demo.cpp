#include <iostream>
#include <Eigen/Dense>
#include "robots.h"
#include "car.h"
#include "scp_solver.h"
#include <osgViewer/Viewer>
#include <osg/Node>
#include <osg/Geode>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "eigen_multivariate_normal.h"

using namespace Eigen;
using namespace std;

int main()
{
  // initialize random number generator
  srand(time(NULL));

  // initialize plotting
  osg::Group* root = new osg::Group(); 
  osg::Geode* geode = new osg::Geode();
  vector<osg::Node*> render;

  // intialize the robot
  int T = 20;
  int NX = 4;
  int NU = 2;
  int NS = 0;
  int NUM_TEST = 0;
  double rho_x = 0.1; 
  double rho_u = 0.1;
  int N_iter = 100;
  
  MatrixXd W_cov = 0.0000*MatrixXd::Identity(NX,NX);
  W_cov(2,2) = 0.00;
  EigenMultivariateNormal<double> sampler(VectorXd::Zero(NX), W_cov);

  VectorXd x0(NX);
  x0 << 0.0, 0.0, 0.0, 0.3;
  vector<VectorXd> X_bar(T+1);
  vector<VectorXd> U_bar(T);
  vector<MatrixXd> W_bar(T); 
  X_bar[0] = x0; 
  
  Car c(x0);
  c.greet();


  for (int t = 0; t < T; t++) {
    VectorXd u = VectorXd::Random(2) / 3;
    if (t > T/2) u = - 2 * u;  
    U_bar[t] = u; 
    c.dynamics(X_bar[t], U_bar[t], X_bar[t+1]);
    sampler.generateSamples(W_bar[t], NS);
  }


  Vector4d purple(1.0, 0.0, 1.0, 0.4);
  for (int s = 0; s < NUM_TEST; s++) {
    VectorXd x = x0;   
    for (int t = 0; t < T; t++) {
      VectorXd xt1;
      c.dynamics(x, U_bar[t], xt1);
      xt1 += sampler.nextSample();
      c.draw(xt1, purple, root); 
      x = xt1;
    }
  }

  Vector4d red(1.0, 0.0, 0.0, 0.9); 
  render = c.draw_trajectory(X_bar, red, root);

  //Setup for SCP

  // Define a goal state
  VectorXd x_goal = X_bar[T];
  // Output variables
  vector<VectorXd> opt_X, opt_U; // noiseless trajectory
  MatrixXd K; VectorXd u0;  // control policy 

  scp_solver(c, X_bar, U_bar, W_bar, rho_x, rho_u, x_goal, N_iter,
      opt_X, opt_U, K, u0, false);

  cout << "scp done" << endl; 

  Vector4d blue(0.3, 0.3, 1.0, 0.4);

  for (int s = 0; s < NUM_TEST; s++) {
    VectorXd x = x0;
    for (int t = 0; t < T; t++) {
      VectorXd u_policy = K.block(NU*t,0,NU,NX*(t+1)) * x + u0.segment(t*NU, NU); 
      VectorXd xt1;
      c.dynamics(x.segment(t*NX, NX), u_policy, xt1);
      //xt1 += W_bar[t].col(s); 
      xt1 += sampler.nextSample();
      VectorXd tmp(NX*(t+2));
      tmp.segment(0,NX*(t+1)) = x;
      tmp.segment(NX*(t+1), NX) = xt1; 
      x = tmp; 
      c.draw(xt1, blue, root); 
    }
  }


  Vector4d green(0.0, 1.0, 0.0, 0.9); 
  render = c.draw_trajectory(opt_X, green, root);


  // visualize
  osg::Geode *fgeode = new osg::Geode; 
  osg::ShapeDrawable *floor = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0,0.0,0.0), 1.0, 1.0, 0.01));
  floor->setColor(osg::Vec4(0.1,0.1,0.1,0.5));
  fgeode->addDrawable(floor);
  root->addChild(fgeode); 
  osgViewer::Viewer viewer;
  viewer.setSceneData(root);
  //viewer.setUpViewInWindow(400, 400, 640, 480);
  return viewer.run();

}
