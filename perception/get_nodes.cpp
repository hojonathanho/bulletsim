#include "get_nodes.h"
using namespace std;

vector<btVector3> getNodes(BulletSoftObject::Ptr psb) {
  btAlignedObjectArray<btSoftBody::Node> nodes = psb->softBody->m_nodes;
  vector<btVector3> out(nodes.size());
  for (int i=0; i < nodes.size(); i++) out[i] = nodes[i].m_x;
  return out;
}
