#include "tracked_object.h"
#include <boost/foreach.hpp>
#include <utility>
#include "config_tracking.h"
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/format.hpp>
#include "simulation/bullet_io.h"
#include <pcl/common/transforms.h>
#include "clouds/utils_pcl.h"
#include "feature_extractor.h"
#include <algorithm>
#include "utils/utils_vector.h"
#include "utils/cvmat.h"
#include "simulation/softBodyHelpers.h"
#include "tracking/utils_tracking.h"
#include "clouds/plane_finding.h"
#include "simulation/util.h"

//DEBUG
#include "plotting_tracking.h"

using namespace std;
using namespace Eigen;

class TrackedNode {
public:
  typedef boost::shared_ptr<TrackedCloth> Ptr;

  TrackedNode(BulletSoftObject::Ptr sim, btVector3 point);

  int m_binding_id;

  enum BindingType { NODE, FACE };
  BindingType m_binding_type;

  void addForce();
  btVector3 getPoint();
};

static inline int idx(int row,int col,int nCols) {return col+row*nCols;}
// mesh vertices are in row major order

typedef pair<int, int> intpair;


BulletSoftObject::Ptr makeCloth(const vector<btVector3>& corners, int resolution_x, int resolution_y, float mass) {
//	int n_tex_coords = (resolution_x - 1)*(resolution_y -1)*12;
//  float * tex_coords = new float[ n_tex_coords ];
//  btVector3 offset(0,0,0.01*METERS);
  btSoftBodyWorldInfo unusedWorldInfo;
  btSoftBody* psb = CreatePolygonPatch(unusedWorldInfo, corners, resolution_x, resolution_y, true);

  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.01;
  //pm->m_kAST = 0.5;
  //pm->m_kAST = 0.0;

  psb->generateBendingConstraints(2,pm);

  psb->setTotalMass(mass);

  //TODO uncomment this when there are no little pieces of faces
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
  psb->randomizeConstraints();
  //psb->randomizeConstraints(tex_coords);

  BulletSoftObject::Ptr bso = BulletSoftObject::Ptr(new BulletSoftObject(psb));
//	bso->tritexcoords = new osg::Vec2Array;
//  for (int i = 0; i < n_tex_coords; i+=2) {
//  	bso->tritexcoords->push_back(osg::Vec2f(tex_coords[i], tex_coords[i+1]));
//  }

	return bso;
}

vector<btVector3> polyCorners(ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer) {
	cv::Mat mask;
	cv::cvtColor(image, mask, CV_BGR2GRAY);
	vector<cv::Point2f> pixels = polyCorners(mask);

	//unproject the pixel into a ray
	vector<Vector3f> rays(pixels.size());
	for (int i=0; i<rays.size(); i++) {
		rays[i] = Vector3f(pixels[i].x - cx, pixels[i].y - cy, f);
	}

	//transform the camera origin and rays
	Vector3f cam = transformer->worldFromCamEigen * Vector3f(0,0,0);
	BOOST_FOREACH(Vector3f& ray, rays) ray = transformer->worldFromCamEigen * ray;

	//find the intersection of the ray with the plane
	vector<float> abcd = getPlaneCoeffsRansac(cloud);
	Vector3f abc(abcd[0], abcd[1], abcd[2]);
	float d = abcd[3];
	vector<btVector3> corners(pixels.size());
	for (int i=0; i<corners.size(); i++) {
		float t = (abc.dot(cam)+d)/abc.dot(cam-rays[i]);
		Vector3f intersection = cam + t * (rays[i] - cam);
		corners[i] = toBulletVector(intersection);
	}

	//DEBUG
//	util::drawLines(toBulletVectors(rays), vector<btVector3>(rays.size(), toBulletVector(transformer->worldFromCamEigen * Vector3f(0,0,0))), Vector3f(1,1,0), 1, env);

	return corners;
}



TrackedCloth::TrackedCloth(BulletSoftObject::Ptr sim, cv::Mat image,  int nCols, int nRows, float sx, float sy) : TrackedObject(sim, "towel"), m_sx(sx), m_sy(sy) {
/*
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
*/
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
/*
  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nNodes; ++i) {
    m_masses(i) = 1/verts[m_node2vert[i]].m_im;
  }
*/
}


void TrackedCloth::applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts) {
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

vector<btVector3> TrackedCloth::getPoints() {
	vector<btVector3> out(m_nNodes);
	btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
	for (int iNode=0; iNode < m_nNodes; ++iNode)
		out[iNode] = verts[m_node2vert[iNode]].m_x;
	return out;
}

vector<btVector3> TrackedCloth::getNormals() {
	vector<btVector3> out(m_nNodes);
	btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
	for (int iNode=0; iNode < m_nNodes; ++iNode)
		out[iNode] = verts[m_node2vert[iNode]].m_n;
	return out;
}

const Eigen::VectorXf TrackedCloth::getPriorDist() {
	Eigen::MatrixXf prior_dist(1,FeatureExtractor::m_allDim);
	prior_dist << TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS,  //FT_XYZ
			0.2, 0.2, 0.2, 	//FT_BGR
			TrackingConfig::colorLPriorDist, TrackingConfig::colorABPriorDist, TrackingConfig::colorABPriorDist,	//FT_LAB
			1.0, 1.0, 1.0,  //FT_NORMAL
			1.0,  //FT_LABEL
			MatrixXf::Ones(1, FE::FT_SIZES[FE::FT_SURF])*0.4,  //FT_SURF
			MatrixXf::Ones(1, FE::FT_SIZES[FE::FT_PCASURF])*0.4,  //FT_PCASURF
			0.5;  //FT_GRADNORMAL
	return FeatureExtractor::all2ActiveFeatures(prior_dist).transpose();
}

cv::Mat TrackedCloth::makeTexture(const vector<btVector3>& corners, cv::Mat image, CoordinateTransformer* transformer) {
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

void TrackedCloth::initColors() {
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
cv::Point2f TrackedCloth::textureCoordinate (int node_id) {
	cv::Mat tex_image = getSim()->getTexture();
	if (tex_image.empty()) return cv::Point2i(0,0);
	const osg::Vec2Array& texcoords = *(getSim()->tritexcoords);
	return cv::Point2f( (int) (texcoords[m_vert2tex[m_node2vert[node_id]]].x() * (tex_image.cols-1)) ,
			tex_image.rows-1 - (int) (texcoords[m_vert2tex[m_node2vert[node_id]]].y() * (tex_image.rows-1)) );
}
