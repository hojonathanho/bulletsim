#include "visibility.h"
#include "utils_perception.h"
#include "clouds/utils_pcl.h"
#include "simulation/rope.h"
#include "simulation/bullet_typedefs.h"
#include "utils/vector_io.h"
#include "utils/config.h"
using namespace Eigen;
using namespace pcl;


// todo: we should do a raycast
Eigen::VectorXf calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask) {
  VectorXf ptDists = pts.rowwise().norm();
  MatrixXi uvs = xyz2uv(pts);
  VectorXf vis(pts.rows(),true);

  for (int iPt=0; iPt<pts.rows(); iPt++) {
    int u = uvs(iPt,0);
    int v = uvs(iPt,1);
    if (u<depth.rows() && v<depth.cols() && u>0 && v>0) {
      vis[iPt] = ropeMask.at<uint8_t>(u,v) || isfinite(depth(u,v)) && (depth(u,v) + .04 > ptDists[iPt]);
    // see it if there's no non-rope pixel in front of it
    }
  }
  return vis;
}

vector<btVector3> calcCandPositions(const btVector3& center, float stdev, int nSamples) {
  vector<btVector3> out(nSamples);

  if (nSamples==1) {
    out[0] = center;
  }
  else if (nSamples==5) { //todo: use a better basis
    out[0] = center;
    out[1] = center + btVector3(stdev,0,0);
    out[2] = center + btVector3(-stdev,0,0);
    out[3] = center + btVector3(0,stdev,0);
    out[4] = center + btVector3(0,-stdev,0);
  }
  else if (nSamples==7) {
    out[0] = center;
    out[1] = center + btVector3(stdev,0,0);
    out[2] = center + btVector3(-stdev,0,0);
    out[3] = center + btVector3(0,stdev,0);
    out[4] = center + btVector3(0,-stdev,0);
    out[5] = center + btVector3(0,0,stdev);
    out[6] = center + btVector3(0,0,-stdev);
  }
  else ENSURE("nSamples must be 1, 5, or 7" && false);
  return out;
}

float calcVisProbability(RigidBodyPtr body, const btVector3& cameraPos, btDynamicsWorld* world, float stdev, int nSamples) {
  vector<btVector3> candPositions = calcCandPositions(body->getCenterOfMassPosition(), stdev, nSamples);
  VectorXf vis(nSamples);
  for (int i=0; i < nSamples; i++) {
    btCollisionWorld::ClosestRayResultCallback rayCallback(cameraPos, candPositions[i]);
    world->rayTest(cameraPos, candPositions[i], rayCallback);
  //btRigidBody* hitBody = btRigidBody::upcast(rayCallback.m_collisionObject);
    btRigidBody* hitBody = dynamic_cast<btRigidBody*>(rayCallback.m_collisionObject);
    vis[i] =  (hitBody==NULL || hitBody == body.get());
  }
  return vis.sum()/nSamples;
}

Eigen::VectorXf calcVisibility(const vector<RigidBodyPtr> bodies, btDynamicsWorld* world, const btVector3& cameraPos, float stdev, int nSamples) {
  int nPts = bodies.size();
  VectorXf vis(nPts);
  for (int i=0; i < nPts; i++)
    vis[i] = calcVisProbability(bodies[i], cameraPos, world, stdev, nSamples);
  return vis;
}

Eigen::VectorXf calcVisibility(btSoftBody* softBody, btDynamicsWorld* world, const btVector3& cameraPos) {
  btAlignedObjectArray<btSoftBody::Node> nodes = softBody->m_nodes;
  VectorXf vis(nodes.size());
  for (int i=0; i < nodes.size(); i++) {
    btVector3 target = nodes[i].m_x + (cameraPos - nodes[i].m_x).normalized() * .005*METERS;
    btCollisionWorld::ClosestRayResultCallback rayCallback(cameraPos, target);
    world->rayTest(cameraPos, target, rayCallback);
    btCollisionObject* hitBody = rayCallback.m_collisionObject;
    vis[i] = (hitBody==NULL);
  }
  return vis;
}


void colorByVisibility(CapsuleRope::Ptr rope, const VectorXf& pVis) {
  ENSURE(rope->children.size() == pVis.rows());
  for (int i=0; i<pVis.rows(); i++) {
    float p = pVis[i];
    rope->children[i]->setColor(0,0,p,1);
  }
}

void colorByVisibility(btSoftBody* psb, const VectorXf& pVis, PointCloudPlot::Ptr plot) {
  int nPts = pVis.rows();
  vector<btVector3> pts(nPts);
  vector<btVector4> cols(nPts);
  for (int i=0; i<nPts; i++) {
    pts.push_back(psb->m_nodes[i].m_x);
    float p = pVis[i];
    cols.push_back(btVector4(p,p,p,1));
  }
  plot->setPoints(pts,cols);
}
