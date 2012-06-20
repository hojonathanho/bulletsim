#include "tracked_object.h"
#include <boost/foreach.hpp>
#include <utility>
#include "config_tracking.h"
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/format.hpp>
#include "simulation/bullet_io.h"

using namespace std;

static inline int idx(int row,int col,int nCols) {return col+row*nCols;}
// mesh vertices are in row major order

typedef pair<int, int> intpair;


TrackedTowel::TrackedTowel(BulletSoftObject::Ptr sim, int nCols, int nRows) : TrackedObject(sim, "towel") {
	cout << "(rows, cols) = " << nRows << "," << nCols << endl;
  assert(sim->softBody->m_nodes.size() == nRows * nCols);
  
  map<intpair,int> coord2node;
  m_vert2nodes = vector< vector<int> >(nRows*nCols); // map from softbody node index to indices of nearby knots

  int decimation = 2;
  for (int row = 0; row < nRows; row += decimation) {
    for (int col = 0; col < nCols; col += decimation) {
      coord2node[intpair(row,col)] = m_node2vert.size();
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
          if (it != coord2node.end()) {
            m_vert2nodes[idx(row, col, nCols)].push_back(it->second);
//            if (row==0) cout << boost::format("(%i, %i), %i, %i") %(row+dRow)%(col+dCol)%idx(row+dRow, col+dCol, nCols)%it->second << endl;
          }
      	}


  // sanity check: for each vertex, the vertex of its node should be either the same (if it's a node) or linked to it
//  if (decimation == 2) {
//    for (int iVert=0; iVert < nRows*nCols; ++iVert) {
//      BOOST_FOREACH(int iNode, m_vert2nodes[iVert]) {
//        int nodeVert = m_node2vert[iNode];
//        if (nodeVert == iVert || sim->softBody->checkLink(nodeVert, iVert)) {
//        	cout << nodeVert << " is linked to " << iVert << endl;
//        }
//        else {
//        	cout << nodeVert << " is not linked to " << iVert << endl;
//        	throw runtime_error("fuck");
//        }
//      }
//    }
//  }
  for (int iVert = 0; iVert < nRows*nCols; ++iVert) {
	  BOOST_FOREACH(int iNode, m_vert2nodes[iVert]) {
//		  cout << iVert << " " << iNode << " " << m_node2vert[iNode] << endl;
		  assert(m_node2vert[iNode] == iVert || sim->softBody->checkLink(m_node2vert[iNode], iVert));
	  }
  }

  const btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nNodes; ++i) {
    m_masses(i) = 1/verts[m_node2vert[i]].m_im;
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
    btVector3 impulse(0,0,0);
    BOOST_FOREACH(int iNode, m_vert2nodes[iVert]) impulse += nodeImpulses[iNode];
    impulse/= m_vert2nodes[iVert].size();
    getSim()->softBody->addForce(impulse, iVert);
  }
  
}

vector<btVector3> TrackedTowel::getPoints() {
	vector<btVector3> out(m_nNodes);
	btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
	for (int iNode=0; iNode < m_nNodes; ++iNode)
		out[iNode] = verts[m_node2vert[iNode]].m_x;
	return out;
}

float cross (const osg::Vec2f& p0, const osg::Vec2f& p1) {
	return (p0.x()*p1.y() - p0.y()*p1.x());
}

osg::Vec2f inverseBilerp(const osg::Vec2f& point, const osg::Vec2Array& corners) {
	float A = cross( (corners[0]-point) , (corners[0]-corners[2]) );
  float B = ( cross( (corners[0]-point) , (corners[1]-corners[3]) ) + cross( (corners[1]-point) , (corners[0]-corners[2]) ) ) / 2.0;
  float C = cross( (corners[1]-point) , (corners[1]-corners[3]) );
  float s = A / (A-C);
  if ((A - 2*B + C) != 0) {
  	s = ( (A-B) + sqrt(B*B - A*C) ) / ( A - 2*B + C );
		if (s < 0 || s > 1)
			s = ( (A-B) - sqrt(B*B - A*C) ) / ( A - 2*B + C );
  }
  float t = ( (1-s)*(corners[0].x()-point.x()) + s*(corners[1].x()-point.x()) ) / ( (1-s)*(corners[0].x()-corners[2].x()) + s*(corners[1].x()-corners[3].x()) );
  return osg::Vec2f(s,t);
}

BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, int resolution_x, int resolution_y, btSoftBodyWorldInfo& worldInfo) {
  btVector3 offset(0,0,.01*METERS);
  int n_tex_coords = (resolution_x - 1)*(resolution_y -1)*12;
  float * tex_coords = new float[ n_tex_coords ];
  btSoftBody* psb=btSoftBodyHelpers::CreatePatchUV(worldInfo,
						 points[0]+offset,
						 points[1]+offset,
						 points[3]+offset,
						 points[2]+offset,
						 resolution_x, resolution_y,
						 0/*1+2+4+8*/, false,
						 tex_coords);

  psb->m_cfg.piterations = 8;
  psb->m_cfg.collisions = btSoftBody::fCollision::CL_RS
      | btSoftBody::fCollision::CL_SS
      | btSoftBody::fCollision::CL_SELF;
  psb->m_cfg.kDF = 0.9;
  psb->m_cfg.kAHR = 1; // anchor hardness
  psb->m_cfg.kSSHR_CL = 1.0; // so the cloth doesn't penetrate itself
  psb->m_cfg.kSRHR_CL = 0.7;
  psb->m_cfg.kSKHR_CL = 0.7;
  psb->m_cfg.kDP = 0.1;

  psb->getCollisionShape()->setMargin(0.05);

  btSoftBody::Material *pm = psb->appendMaterial();
  pm->m_kLST = 0.1;
  pm->m_kAST = 0.5;

  psb->generateBendingConstraints(2, pm);

  // this function was not in the original bullet physics
  // this allows to swap the texture coordinates to match the swaped faces
  psb->randomizeConstraints(tex_coords);

  psb->setTotalMass(10, true);
  psb->generateClusters(100);

  BulletSoftObject::Ptr bso = BulletSoftObject::Ptr(new BulletSoftObject(psb));
	bso->tritexcoords = new osg::Vec2Array;
  for (int i = 0; i < n_tex_coords; i+=2) {
  	bso->tritexcoords->push_back(osg::Vec2f(tex_coords[i], tex_coords[i+1]));
  }

	return bso;
}


