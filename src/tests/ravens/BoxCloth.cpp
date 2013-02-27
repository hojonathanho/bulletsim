#include "BoxCloth.h"
#include <iostream>
#include "bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/conversions.h"
#include "utils/utils_vector.h"

using namespace std;


/** Useful for creating rigid bodies: creates collision shape, motion state etc.*/
boost::shared_ptr<btRigidBody> createRigidBody(const boost::shared_ptr<btCollisionShape> shapePtr, const btTransform& trans, btScalar mass) {
  bool isDynamic = (mass != 0.f);
  btVector3 localInertia(0,0,0);
  if (isDynamic) shapePtr->calculateLocalInertia(mass,localInertia);
  btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
  btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shapePtr.get(),localInertia);
  return boost::shared_ptr<btRigidBody>(new btRigidBody(cInfo));
}

/** Creates a matrix of transformation for each of the n x m boxes.
 *  The side length along x,y dimensions is assumed to be s.
 *  The transforms are centered around CENTER. */
void createBoxTransforms(vector<btTransform> &transforms, unsigned int n, unsigned int m, btScalar s, btVector3 center) {
	transforms.clear();
	for(int i=0; i<n; i += 1) {
		for (int j=0; j<m; j+= 1) {
			btTransform T;
			T.setIdentity();
			btVector3 loc(-(n/2)*s+i*s, -(m/2)*s+j*s, 0.0);
			loc += center;
			T.setOrigin(loc);
			transforms.push_back(T);
		}
	}
}

BoxCloth::BoxCloth(unsigned int n_, unsigned int m_, btScalar s_, btScalar h_, float angStiffness_, float linDamping_, float angDamping_, float angLimit_) {
	angStiffness = angStiffness_;
	angDamping   = angDamping_;
	angLimit     = angLimit_;
	linDamping   = linDamping_;

	vector<btTransform> transforms;
	vector< vector<BoxObject> >  boxes;

	createBoxTransforms(transforms,n,m,s, btVector3(0,0,0));
	btVector3 halfExtents(s/2, s/2, h/2);
	btScalar  mass = 1.0;


	// create boxes.
	for (unsigned int i=0; i < n; i++) {
		for(unsigned int j=0; j<m; j++) {

			btTransform trans = transforms[i*m+j];
			BoxObject::Ptr child(new BoxObject(mass, halfExtents, trans));
			child->rigidBody->setDamping(linDamping, angDamping);
			child->rigidBody->setFriction(1);
			children.push_back(child);

		}
	}

	// create constraints
	for (unsigned int i=0; i < n; i++) {
		for(unsigned int j=0; j<m; j++) {

			unsigned int right =

			//boost::shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*children[i-1]->rigidBody,*children[i]->rigidBody,btVector3(len/2,0,0),btVector3(-len/2,0,0)));
			//jointPtr->setParam(BT_CONSTRAINT_STOP_ERP, linStopErp_);
			//jointPtr->setBreakingImpulseThreshold(10.2); /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< added as per the demos
			joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr, true)));
			boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr = createBendConstraint(len,children[i-1]->rigidBody,children[i]->rigidBody,angDamping,angStiffness,angLimit);
			joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtr, true)));
		}

	}
}





boost::shared_ptr<btGeneric6DofSpringConstraint>  createBendConstraint(btScalar len,
		const boost::shared_ptr<btRigidBody> rbA, const boost::shared_ptr<btRigidBody>& rbB, float damping, float stiffness, float limit) {

	btTransform tA,tB;
	tA.setIdentity(); tB.setIdentity();
	tA.setOrigin(btVector3(len/2,0,0)); tB.setOrigin(btVector3(-len/2,0,0));
	boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr = boost::shared_ptr<btGeneric6DofSpringConstraint>(new btGeneric6DofSpringConstraint(*rbA,*rbB,tA,tB,false));
	for (int i=3; i<=5; i++) {
		springPtr->enableSpring(i,true);
		springPtr->setStiffness(i,stiffness);
		springPtr->setDamping(i,damping);
	}
	springPtr->setAngularLowerLimit(btVector3(-limit,-limit,-limit));
	springPtr->setAngularUpperLimit(btVector3(limit,limit,limit));
	return springPtr;
}

const float SMALL_FLOAT = 1e-7;
static btMatrix3x3 makePerpBasis(const btVector3& a0) {
	btVector3 a = a0.normalized();
	if ((a.x() == 0) && (a.y() == 0)) {
		return btMatrix3x3(0,0,1,0,1,0,0,0,1);
	}
	else {
		btVector3 b = btVector3(a0.y(), -a0.x(), 0);
		b.normalize();
		btVector3 c = a.cross(b);
		return btMatrix3x3(a.x(), a.y(), a.z(),
				b.x(), b.y(), b.z(),
				c.x(), c.y(), c.z());
	}
}


void createRopeTransforms(vector<btTransform>& transforms, vector<btScalar>& lengths, const vector<btVector3>& ctrlPoints) {
	int nLinks = ctrlPoints.size()-1;
	for (int i=0; i < nLinks; i++) {
		btVector3 pt0 = ctrlPoints[i];
		btVector3 pt1 = ctrlPoints[i+1];
		btVector3 midpt = (pt0+pt1)/2;
		btVector3 diff = (pt1-pt0);

		btMatrix3x3 rotation = makePerpBasis(diff);


		btTransform trans(rotation,midpt);

		float len = diff.length();
		transforms.push_back(trans);
		lengths.push_back(len);
	}

}

void CapsuleRope::init() {
	CompoundObject<BulletObject>::init();
	for (int i=0; i< joints.size(); i++) {
		getEnvironment()->addConstraint(joints[i]);
	}
}


void CapsuleRope::destroy() {
	CompoundObject<BulletObject>::destroy();
}


// For every capsule, split it into x_res bins along its axis. Each of these bins has one color.
// For every bin in a capsule, look at the color of the bin center and the rings around the bin center. The mean of all these determines the one color of the bin.
void CapsuleRope::setTexture(cv::Mat image, cv::Mat mask, const btTransform& camFromWorld) {
	vector<btVector3> nodes = getNodes();
	int x_res = 4;
	int ang_res = 10;
	cv::Mat tex_image(1, nodes.size()*x_res, CV_8UC3);
	vector<btMatrix3x3> rotations = getRotations();
	vector<float> half_heights = getHalfHeights();

	for (int j=0; j<nodes.size(); j++) {
		btVector3 node = nodes[j];
//		Matrix3f node_rot = toEigenMatrix(rotations[j]);
		btMatrix3x3 node_rot = rotations[j];
		float node_half_height = half_heights[j];

		vector<vector<float> > B_bins(x_res), G_bins(x_res), R_bins(x_res);
		for (int xId=0; xId<x_res; xId++) {
//			btVector3 sub_node = node + toBulletVector(node_rot.col(0)*(-node_half_height + node_half_height*(1+2*xId)/((float)x_res)));
			btVector3 sub_node = node + node_rot.getColumn(0)*(-node_half_height + node_half_height*(1+2*xId)/((float)x_res));

			vector<cv::Vec3b> bin_colors;
			float radius_frac = 0.0;
			while (bin_colors.size() < 10 || radius_frac <= 2.0) {

				if (radius_frac == 0.0) {
					cv::Point2f uv = xyz2uv(camFromWorld * sub_node);
					cv::Vec3b color = image.at<cv::Vec3b>(uv.y, uv.x);
					if (mask.at<bool>(uv.y, uv.x)) {
						bin_colors.push_back(color);
						//util::drawSpheres(sub_node, Vector3f(((float)color[2])/255.0, ((float)color[1])/255.0, ((float)color[0])/255.0), 1, 0.001*METERS, util::getGlobalEnv());
					}
				} else {
					for (int angId=0; angId<ang_res; angId++) {
						float angle = 2.0*M_PI*((float)angId)/((float)(ang_res));
//						btVector3 surf_pt = sub_node + radius_frac*radius*toBulletVector(AngleAxisf(angle, node_rot.col(0)) * node_rot.col(1));
						btVector3 surf_pt = sub_node + radius_frac*radius* (btMatrix3x3(btQuaternion(node_rot.getColumn(0), angle)) * node_rot.getColumn(1));
						cv::Point2f uv = xyz2uv(camFromWorld * surf_pt);
						cv::Vec3b color = image.at<cv::Vec3b>(uv.y, uv.x);
						if (mask.at<bool>(uv.y, uv.x)) {
							bin_colors.push_back(color);
							//util::drawSpheres(surf_pt, Vector3f(((float)color[2])/255.0, ((float)color[1])/255.0, ((float)color[0])/255.0), 1, 0.001*METERS, util::getGlobalEnv());
						}
					}
				}

				radius_frac += 0.5;
			}

			BOOST_FOREACH(const cv::Vec3b& color, bin_colors) {
				B_bins[xId].push_back(color[0]);
				G_bins[xId].push_back(color[1]);
				R_bins[xId].push_back(color[2]);
			}
		}

		for (int xId=0; xId<x_res; xId++) {
			tex_image.at<cv::Vec3b>(0,j*x_res+xId)[0] = mean(B_bins[xId]);
			tex_image.at<cv::Vec3b>(0,j*x_res+xId)[1] = mean(G_bins[xId]);
			tex_image.at<cv::Vec3b>(0,j*x_res+xId)[2] = mean(R_bins[xId]);
		}

	}

	CompoundObject::setTexture(tex_image);
}
