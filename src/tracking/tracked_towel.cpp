#include "tracked_object.h"
#include <boost/foreach.hpp>
#include <utility>
#include "config_tracking.h"
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/format.hpp>
#include "simulation/bullet_io.h"
#include <pcl/common/transforms.h>
#include "clouds/utils_pcl.h"
#include "feature_extractor.h"
#include "utils_tracking.h"

using namespace std;
using namespace Eigen;

static inline int idx(int row,int col,int nCols) {return col+row*nCols;}
// mesh vertices are in row major order

typedef pair<int, int> intpair;


TrackedTowel::TrackedTowel(BulletSoftObject::Ptr sim, int nCols, int nRows, float sx, float sy) : TrackedObject(sim, "towel"), m_sx(sx), m_sy(sy) {
	cout << "(rows, cols) = " << nRows << "," << nCols << endl;
	assert(sim->softBody->m_nodes.size() == nRows * nCols);

	map<intpair,int> coord2node;
	m_vert2nodes = vector< vector<int> >(nRows*nCols); // map from softbody node index to indices of nearby knots

	//  int decimation = 2;
	//  for (int row = 0; row < nRows; row += decimation) {
	//    for (int col = 0; col < nCols; col += decimation) {
	//      coord2node[intpair(row,col)] = m_node2vert.size();
	//      m_node2vert.push_back(idx(row,col,nCols));
	//    }
	//  }

	// the decimation doesn't apply for the edge nodes
	int decimation = 2;
	for (int row = 0; row < nRows; row++) {
		for (int col = 0; col < nCols; col++) {
			if (row==0 || row==(nRows-1) || col==0 || col==(nCols-1) || (row%decimation==0 && col%decimation==0)) {
				coord2node[intpair(row,col)] = m_node2vert.size();
				m_node2vert.push_back(idx(row,col,nCols));
			}
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

	const btSoftBody::tNodeArray& verts = getSim()->softBody->m_nodes;
	const btSoftBody::tFaceArray& faces = getSim()->softBody->m_faces;
	// vertex                         <->   texture coordinate
	// faces[j].m_n[0]                      texcoords[3*j]
	// faces[j].m_n[1]                      texcoords[3*j+1]
	// faces[j].m_n[2]                      texcoords[3*j+2]
	// verts[i] == faces[j].m_n[c]		<->   texcoords[3*j+c] == texcoords[m_vert2tex[i]]
	int found = 0;
	int not_found = 0;
	for (int i=0; i<verts.size(); i++) {
		int j,c;
		for(j=0; j<faces.size(); j++) {
			for(c=0; c<3; c++) {
				if (&verts[i] == faces[j].m_n[c]) break;
			}
			if (&verts[i] == faces[j].m_n[c]) break;
		}
		m_vert2tex.push_back(3*j+c);
	}
	for (int iVert=0; iVert<verts.size(); iVert++) {
		assert(&verts[iVert] == faces[m_vert2tex[iVert]/3].m_n[m_vert2tex[iVert]%3]);
	}

	/*
	m_face2verts = vector< vector<int> > (faces.size());
	for (int j=0; j>faces.size(); j++) {
		m_face2verts[j] = vector<int> (3);
		for(int c=0; c<3; c++) {
			int i;
			for (i=0; i<verts.size(); i++) {
				if (&verts[i] == faces[j].m_n[c]) break;
			}
			m_face2verts[j][c] = i;
		}
	}
	for(int j=0; j<faces.size(); j++) {
		for (int c=0; c<3; c++) {
			assert(&verts[m_face2verts[j][c]] == faces[j].m_n[c]);
		}
	}
	 */

	m_masses.resize(m_nNodes);
	for (int i=0; i < m_nNodes; ++i) {
		m_masses(i) = 1/verts[m_node2vert[i]].m_im;
	}
}

void TrackedTowel::applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts) {
	vector<btVector3> estPos(m_nNodes);
	vector<btVector3> estVel(m_nNodes);

	btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
	for (int iNode=0; iNode < m_nNodes; ++iNode)  {
		estPos[iNode] = verts[m_node2vert[iNode]].m_x;
		estVel[iNode] = verts[m_node2vert[iNode]].m_v;
	}

	vector<btVector3> nodeImpulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts), corr, toVec(m_masses), TrackingConfig::kp_cloth, TrackingConfig::kd_cloth);

	int nVerts = verts.size();

	for (int iVert=0; iVert < nVerts; iVert++) {
		btVector3 impulse(0,0,0);
		BOOST_FOREACH(int iNode, m_vert2nodes[iVert]) impulse += nodeImpulses[iNode];
		impulse/= m_vert2nodes[iVert].size();
		getSim()->softBody->addForce(impulse, iVert);

		assert(isfinite(impulse.x()) && isfinite(impulse.y()) && isfinite(impulse.z()));
	}
}

vector<btVector3> TrackedTowel::getPoints() {
	vector<btVector3> out(m_nNodes);
	btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
	for (int iNode=0; iNode < m_nNodes; ++iNode)
		out[iNode] = verts[m_node2vert[iNode]].m_x;
	return out;
}

vector<btVector3> TrackedTowel::getNormals() {
	vector<btVector3> out(m_nNodes);
	btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
	for (int iNode=0; iNode < m_nNodes; ++iNode)
		out[iNode] = verts[m_node2vert[iNode]].m_n;
	return out;
}

//  corners:
//
//                        s3 = sx
//  [0][0]     corners[0] -------- corners[3]   [resx][0]
//                 |                   |
//         s0 = sy |                   | s2 = sy
//                 |                   |
//  [0][resy]  corners[1] -------- corners[2]  [resx][resy]
//                        s1 = sx
//
// node_density = (# nodes)/(side length) = (resolution)/(side length)
// surface_density = (total mass)/(total area)
BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, int resolution_x, int resolution_y, float mass) {
	vector<btVector3> corners;
	corners.push_back(points[0]);
	corners.push_back(points[1]);
	corners.push_back(points[3]);
	corners.push_back(points[2]);

	int n_tex_coords = (resolution_x - 1)*(resolution_y -1)*12;
	float * tex_coords = new float[ n_tex_coords ];
	btVector3 offset(0,0,0.01*METERS);
	btSoftBodyWorldInfo unusedWorldInfo;
	btSoftBody* psb=btSoftBodyHelpers::CreatePatchUV(unusedWorldInfo,
			corners[0]+offset,
			corners[1]+offset,
			corners[2]+offset,
			corners[3]+offset,
			resolution_x, resolution_y,
			0, true,
			tex_coords);

	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST = 0.01;
	//pm->m_kAST = 0.5;
	//pm->m_kAST = 0.0;

	psb->generateBendingConstraints(2,pm);

	psb->setTotalMass(mass);

	psb->generateClusters(512);
	psb->getCollisionShape()->setMargin(0.002*METERS);

	psb->m_cfg.collisions	=	0;
	psb->m_cfg.collisions += btSoftBody::fCollision::SDF_RS; ///SDF based rigid vs soft
	//psb->m_cfg.collisions += btSoftBody::fCollision::CL_RS; ///Cluster vs convex rigid vs soft
	//psb->m_cfg.collisions += btSoftBody::fCollision::VF_SS;	///Vertex vs face soft vs soft handling
	psb->m_cfg.collisions += btSoftBody::fCollision::CL_SS; ///Cluster vs cluster soft vs soft handling
	psb->m_cfg.collisions	+= btSoftBody::fCollision::CL_SELF; ///Cluster soft body self collision

	psb->m_cfg.kDF = 1.0; //0.9; // Dynamic friction coefficient
	//  psb->m_cfg.kAHR = 1; // anchor hardness
	//  psb->m_cfg.kSSHR_CL = 1.0; // so the cloth doesn't penetrate itself
	//  psb->m_cfg.kSRHR_CL = 0.7;
	//  psb->m_cfg.kSKHR_CL = 0.7;
	//  psb->m_cfg.kDP = 0.1; //1.0; // Damping coefficient [0,1]

	psb->m_cfg.piterations = 50; //8;
	psb->m_cfg.citerations = 50;
	psb->m_cfg.diterations = 50;
	//	psb->m_cfg.viterations = 10;

	// this function was not in the original bullet physics
	// this allows to swap the texture coordinates to match the swapped faces
	psb->randomizeConstraints(tex_coords);

	BulletSoftObject::Ptr bso = BulletSoftObject::Ptr(new BulletSoftObject(psb));
	bso->tritexcoords = new osg::Vec2Array;
	for (int i = 0; i < n_tex_coords; i+=2) {
		bso->tritexcoords->push_back(osg::Vec2f(tex_coords[i], tex_coords[i+1]));
	}

	return bso;
}

cv::Mat TrackedTowel::makeTexture(const vector<btVector3>& corners, cv::Mat image, CoordinateTransformer* transformer) {
	vector<Vector3f> points = toEigenVectors(corners);
	BOOST_FOREACH(Vector3f& point, points) point = transformer->camFromWorldEigen * point;

	MatrixXi src_pixels = xyz2uv(toEigenMatrix(points));
	MatrixXi dst_pixels(4,2);

	int resolution_x = m_sx/(TrackingConfig::node_distance*METERS) + 1;
	int resolution_y = m_sy/(TrackingConfig::node_distance*METERS) + 1;
	int tex_sx = TrackingConfig::node_pixel * resolution_x;
	int tex_sy = TrackingConfig::node_pixel * resolution_y;

	dst_pixels <<   0,   0,
			0,  tex_sx,
			tex_sy,  tex_sx,
			tex_sy,   0;
	cv::Mat src_mat(4, 2, CV_32FC1);
	cv::Mat dst_mat(4, 2, CV_32FC1);
	for(int i=0; i<src_mat.rows; i++) {
		for(int j=0; j<src_mat.cols; j++) {
			src_mat.at<float>(i,j) = (float) src_pixels(i,(j+1)%2);
			dst_mat.at<float>(i,j) = (float) dst_pixels(i,(j+1)%2);
		}
	}
	cv::Mat H = cv::findHomography(src_mat, dst_mat);
	cv::Mat tex_image(tex_sy, tex_sx, CV_8UC3);
	cv::warpPerspective(image, tex_image, H, cv::Size(tex_sx, tex_sy));
	cv::Mat tex_image_lab(tex_sy, tex_sx, CV_8UC3);

	return tex_image;
}

void TrackedTowel::initColors() {
	m_colors.resize(m_nNodes, 3);
	cv::Mat tex_image = getSim()->getTexture();
	if (!tex_image.empty()) {
		cout << "initColors using texture" << endl;
		const osg::Vec2Array& texcoords = *(getSim()->tritexcoords);
		for (int i=0; i < m_nNodes; i++) {
			cv::Point2f pixel = textureCoordinate(i);
			int i_pixel = pixel.y;
			int j_pixel = pixel.x;
			int range = 20;
			//TODO weighted average window
			cv::Mat window_pixels = windowRange(tex_image, i_pixel, j_pixel, range, range);

			Vector3f bgr = toEigenMatrixImage(window_pixels).colwise().mean();
			m_colors.row(i) = bgr.transpose();
		}
	}
	else {
		cout << "initColors using color" << endl;
		osg::Vec4f color = getSim()->getColor();
		for (int i=0; i < m_nNodes; i++) {
			m_colors.row(i) = Eigen::Vector3f(color.r(), color.g(), color.b()).transpose();
		}
	}
}

//order: (x,y) or (j,i)
cv::Point2f TrackedTowel::textureCoordinate (int node_id) {
	cv::Mat tex_image = getSim()->getTexture();
	if (tex_image.empty()) return cv::Point2i(0,0);
	const osg::Vec2Array& texcoords = *(getSim()->tritexcoords);
	return cv::Point2f( (int) (texcoords[m_vert2tex[m_node2vert[node_id]]].x() * (tex_image.cols-1)) ,
			tex_image.rows-1 - (int) (texcoords[m_vert2tex[m_node2vert[node_id]]].y() * (tex_image.rows-1)) );
}
