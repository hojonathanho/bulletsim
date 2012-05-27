#include "tracked_object.h"
#include <boost/foreach.hpp>
#include <utility>
#include "config_tracking.h"
#include "simulation/config_bullet.h"
#include "utils/conversions.h"

using namespace std;

static inline int idx(int row,int col,int nCols) {return col+row*nCols;}
// mesh vertices are in row major order

typedef pair<int, int> intpair;


TrackedTowel::TrackedTowel(BulletSoftObject::Ptr sim, int nRows, int nCols) : TrackedObject(sim) {
  m_type = "towel";
  assert(sim->softBody->m_nodes.size() == nRows * nCols);
  
  map<intpair,int> coord2node; 
  m_vert2nodes = vector< vector<int> >(nRows*nCols); // map from softbody node index to indices of nearby knots

  int decimation = 2;
  for (int row = 0; row < nRows; row += decimation) {
    for (int col = 0; col < nCols; col += decimation) {
      coord2node[intpair(row,col)] = idx(row, col, nCols);
      m_node2vert.push_back(idx(row,col,nCols));
    }
  }
  m_nNodes = m_node2vert.size();

  // now look in a window around each vertex and give it a node for each vertex in this window
  int radius = decimation/2;
  for (int row = 0; row < nRows; ++row)
    for (int col = 0; col < nCols; ++col)
      for (int dRow = -radius; dRow <= radius; ++dRow)
      	for (int dCol = -radius; dCol <= radius; ++dCol) {
          map<intpair,int>::iterator it = coord2node.find(intpair(row+dRow, col+dCol));
          if (it != coord2node.end())
            m_vert2nodes[idx(row+dRow, col+dCol, nCols)].push_back(it->second);
      	}

  // sanity check: for each vertex, the vertex of its node should be either the same (if it's a node) or linked to it
  if (decimation == 2) {
    for (int iVert=0; iVert < nRows*nCols; ++iVert) {
      BOOST_FOREACH(int iNode, m_vert2nodes[iVert]) {
        int nodeVert = m_node2vert[iNode];
        assert(nodeVert == iVert || sim->softBody->checkLink(nodeVert, iVert));
      }
    }
  }

  const btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
  for (int i=0; i < verts.size(); ++i) {
    m_masses(i) = 1/verts[i].m_im;
  }

}

void TrackedTowel::applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts) {

  vector<btVector3> estPos(m_nNodes);
  vector<btVector3> estVel(m_nNodes);
  
  btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
  for (int iNode=0; iNode < m_nNodes; ++iNode)  {
    estPos[iNode] = verts[m_node2vert[iNode]].m_x;
    estVel[iNode] = verts[m_node2vert[iNode]].m_v;
  }

  vector<btVector3> nodeImpulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts), corr, toVec(m_masses), TrackingConfig::kp_cloth, TrackingConfig::kd_cloth);

  // m_sigs = calcSigs(corr, toEigenMatrix(estPos), toEigenMatrix(obsPts), .1*METERS, .5);

  int nVerts = verts.size();

  for (int iVert=0; iVert < nVerts; iVert++) {
    btVector3 impulse;
    BOOST_FOREACH(int iNode, m_vert2nodes[iVert]) impulse += nodeImpulses[iNode];
    impulse/= m_vert2nodes[iVert].size();
    getSim()->softBody->addForce(impulse, iVert);
  }
  
}
