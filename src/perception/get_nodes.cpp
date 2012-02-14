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

vector<float> getNodeMasses(BulletSoftObject::Ptr psb) {
  int nNodes = psb->softBody->m_nodes.size();
  return vector<float>(nNodes, 1/((float)nNodes));
}
