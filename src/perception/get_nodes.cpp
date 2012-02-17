#include "get_nodes.h"
using namespace std;

vector<btVector3> getNodes(BulletSoftObject::Ptr psb) {
  btAlignedObjectArray<btSoftBody::Node> nodes = psb->softBody->m_nodes;
  vector<btVector3> out(nodes.size());
  for (int i=0; i < nodes.size(); i++) out[i] = nodes[i].m_x;
  return out;
}

vector<btVector3> getNodeVels(BulletSoftObject::Ptr psb) {
  btAlignedObjectArray<btSoftBody::Node> nodes = psb->softBody->m_nodes;
  vector<btVector3> out(nodes.size());
  for (int i=0; i < nodes.size(); i++) out[i] = nodes[i].m_v;
  return out;
}

vector<btVector3> getNodeVels(CapsuleRope::Ptr rope) {
  vector<btVector3> out(rope->nLinks);
  for (int i=0; i < rope->nLinks; i++) out[i] = rope->bodies[i]->getLinearVelocity();
  return out;
}


vector<float> getNodeMasses(BulletSoftObject::Ptr psb) {
  int nNodes = psb->softBody->m_nodes.size();
  return vector<float>(nNodes, psb->softBody->getTotalMass()/nNodes);
}

vector<float> getNodeMasses(CapsuleRope::Ptr rope) {
  return vector<float>(rope->nLinks,1);
}
