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

using namespace std;
using namespace Eigen;

TrackedCloth::TrackedCloth(BulletSoftObject::Ptr sim) : TrackedObject(sim, "towel") {
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

void TrackedCloth::initColors() {
	m_colors.resize(m_nNodes, 3);
	cv::Mat tex_image = getSim()->getTexture();
	if (!tex_image.empty()) {
		cout << "initColors using texture" << endl;
//		cv::Mat circle_image = tex_image.clone();
		for (int i=0; i < m_nNodes; i++) {
			cv::Point2f pixel = getTexCoord(i);
			int i_pixel = pixel.y;
			int j_pixel = pixel.x;
			int range = 5;
			//TODO weighted average window
//			printf("tex image: %i %i %i %i\n", tex_image.rows, tex_image.cols, i_pixel, j_pixel);
			cv::Mat window_pixels = windowRange(tex_image, i_pixel, j_pixel, range, range);
//			if (i%100 == 0 || (i+1)%100==0)
//				cv::circle(circle_image, pixel, range, cv::Scalar(0,255,0));

			Vector3f bgr = toEigenMatrixImage(window_pixels).colwise().mean();
			m_colors.row(i) = bgr.transpose();
		}
//		cv::imwrite("/home/alex/Desktop/circle_image.jpg", circle_image);
	}
	else {
		cout << "initColors using color" << endl;
		osg::Vec4f color = getSim()->getColor();
		for (int i=0; i < m_nNodes; i++) {
			m_colors.row(i) = Eigen::Vector3f(color.r(), color.g(), color.b()).transpose();
		}
	}
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
