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
#include "utils_tracking.h"

//DEBUG
#include "simulation/util.h"

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

TrackedCloth::TrackedCloth(BulletSoftObject::Ptr sim, int nCols, int nRows, float sx, float sy) : TrackedObject(sim, "towel"), m_sx(sx), m_sy(sy) {
	for (int i=0; i<getSim()->softBody->m_nodes.size(); i++) {
		m_node2vert.push_back(i);
		m_vert2nodes.push_back(vector<int>(1,i));
	}
  m_nNodes = m_node2vert.size();

	const btSoftBody::tNodeArray& verts = getSim()->softBody->m_nodes;
  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nNodes; ++i) {
    m_masses(i) = 1/verts[m_node2vert[i]].m_im;
  }
}

inline cv::Point2f TrackedCloth::getTexCoord(const int& nodeIdx) {
	return getSim()->getTexCoord(m_node2vert[nodeIdx]);
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

void TrackedCloth::initColors() {
	m_colors.resize(m_nNodes, 3);
	cv::Mat tex_image = getSim()->getTexture();
	if (!tex_image.empty()) {
		cout << "initColors using texture" << endl;
		for (int i=0; i < m_nNodes; i++) {
			cv::Point2f pixel = getTexCoord(i);
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

BulletSoftObject::Ptr makeCloth(const vector<btVector3>& corners, int resolution_x, int resolution_y, float mass) {
  btSoftBodyWorldInfo unusedWorldInfo;
  btSoftBody* psb = CreatePolygonPatch(unusedWorldInfo, corners, resolution_x, resolution_y, true);

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

  psb->randomizeConstraints();

  BulletSoftObject::Ptr bso = BulletSoftObject::Ptr(new BulletSoftObject(psb));

	return bso;
}

vector<btVector3> polyCorners(ColorCloudPtr cloud, cv::Mat mask, CoordinateTransformer* transformer) {
	vector<cv::Point2f> pixels = polyCorners(mask.clone());

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

	return corners;
}

#include "simulation/tetgen_helpers.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

// Assumes corners are in the xy-plane and the thickness is the z-axis direction
// good max_tet_vol: 0.0008*METERS*METERS*METERS
BulletSoftObject::Ptr makeSponge(const vector<btVector3>& corners, float thickness, float max_tet_vol, float mass) {
  btSoftBodyWorldInfo unusedWorldInfo;
	btSoftBody* psb=CreatePrism(unusedWorldInfo, corners, btVector3(0,0,thickness), 1.414, max_tet_vol, false,true,true);

  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;

  //psb->generateBendingConstraints(2,pm);

	psb->setVolumeMass(mass);

  psb->generateClusters(16);
	psb->getCollisionShape()->setMargin(0.001*METERS);

  psb->m_cfg.collisions	=	0;
  //psb->m_cfg.collisions += btSoftBody::fCollision::SDF_RS; ///SDF based rigid vs soft
  psb->m_cfg.collisions += btSoftBody::fCollision::CL_RS; ///Cluster vs convex rigid vs soft
  //psb->m_cfg.collisions += btSoftBody::fCollision::VF_SS;	///Vertex vs face soft vs soft handling
  psb->m_cfg.collisions += btSoftBody::fCollision::CL_SS; ///Cluster vs cluster soft vs soft handling
  //psb->m_cfg.collisions	+= btSoftBody::fCollision::CL_SELF; ///Cluster soft body self collision

  psb->m_cfg.kDF = 1.0; // Dynamic friction coefficient

	psb->m_cfg.piterations=1;

  BulletSoftObject::Ptr bso = BulletSoftObject::Ptr(new BulletSoftObject(psb));

	return bso;
}

// Returns the approximate polygon of the concave hull of the cloud
// The points are being projected to the xy plane
vector<btVector3> polyCorners(ColorCloudPtr cloud) {
	ColorCloudPtr cloud_filtered (new ColorCloud(*cloud));
	ColorCloudPtr cloud_projected (new ColorCloud);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_filtered);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ConcaveHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud (cloud_projected);
	chull.setAlpha (0.01*METERS);
	chull.reconstruct (*cloud_hull);

	vector<btVector3> pts;
	for (int i=0; i<cloud_hull->size(); i++) {
		ColorPoint pt = cloud_hull->at(i);
		btVector3 xyz(pt.x,pt.y,pt.z);
		pts.push_back(xyz);
	}

	vector<float> pts_z;
	for (int i=0; i<pts.size(); i++)
		pts_z.push_back(pts[i].z());
	float median_z = median(pts_z);

	vector<cv::Point2f> pts_2d;
	for (int i=0; i<pts.size(); i++)
		pts_2d.push_back(cv::Point2f(pts[i].x(), pts[i].y()));

	vector<cv::Point2f> pts_approx_2d;
	cv::approxPolyDP(cv::Mat(pts_2d), pts_approx_2d, 0.01*METERS, true);

	vector<btVector3> pts_approx;
	for (int i=0; i<pts_approx_2d.size(); i++)
		pts_approx.push_back(btVector3(pts_approx_2d[i].x, pts_approx_2d[i].y, median_z));

	return pts_approx;
}
