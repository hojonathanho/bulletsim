#include "surface_sampling.h"
#include <boost/foreach.hpp>
#include "utils/conversions.h"
#include "simulation/util.h"
#include "simulation/bullet_io.h"
#include "clouds/pcl_typedefs.h"
#include <pcl/filters/voxel_grid.h>
#include "simulation/config_bullet.h"

using namespace std;

//print "{"+",\n".join(["{%.4f, %.4f, %.4f}"%(x,y,z) for (x,y,z) in sphere_sampling.get_sphere_points(2)])+"}"
const static float SPHERE_SAMPLES[][3] =
		{{0.0000, -0.5257, -0.8507},
		{0.0000, -0.2952, -0.9554},
		{0.0000, 0.0000, -1.0000},
		{0.0000, 0.2952, -0.9554},
		{0.0000, 0.5257, -0.8507},
		{-0.2389, -0.4429, -0.8642},
		{0.2389, -0.4429, -0.8642},
		{-0.2629, -0.1625, -0.9511},
		{0.2629, -0.1625, -0.9511},
		{-0.2629, 0.1625, -0.9511},
		{0.2629, 0.1625, -0.9511},
		{-0.2389, 0.4429, -0.8642},
		{0.2389, 0.4429, -0.8642},
		{-0.5000, -0.3090, -0.8090},
		{0.5000, -0.3090, -0.8090},
		{-0.5257, 0.0000, -0.8507},
		{0.5257, 0.0000, -0.8507},
		{-0.5000, 0.3090, -0.8090},
		{0.5000, 0.3090, -0.8090},
		{-0.1476, -0.6817, -0.7166},
		{0.1476, -0.6817, -0.7166},
		{-0.1476, 0.6817, -0.7166},
		{0.1476, 0.6817, -0.7166},
		{-0.7166, -0.1476, -0.6817},
		{0.7166, -0.1476, -0.6817},
		{-0.7166, 0.1476, -0.6817},
		{0.7166, 0.1476, -0.6817},
		{-0.4253, -0.5878, -0.6882},
		{0.4253, -0.5878, -0.6882},
		{-0.4253, 0.5878, -0.6882},
		{0.4253, 0.5878, -0.6882},
		{-0.8507, 0.0000, -0.5257},
		{0.8507, 0.0000, -0.5257},
		{-0.6882, -0.4253, -0.5878},
		{0.6882, -0.4253, -0.5878},
		{-0.6882, 0.4253, -0.5878},
		{0.6882, 0.4253, -0.5878},
		{-0.3090, -0.8090, -0.5000},
		{0.0000, -0.8507, -0.5257},
		{0.3090, -0.8090, -0.5000},
		{-0.3090, 0.8090, -0.5000},
		{0.0000, 0.8507, -0.5257},
		{0.3090, 0.8090, -0.5000},
		{-0.8642, -0.2389, -0.4429},
		{0.8642, -0.2389, -0.4429},
		{-0.8642, 0.2389, -0.4429},
		{0.8642, 0.2389, -0.4429},
		{-0.5878, -0.6882, -0.4253},
		{0.5878, -0.6882, -0.4253},
		{-0.5878, 0.6882, -0.4253},
		{0.5878, 0.6882, -0.4253},
		{-0.8090, -0.5000, -0.3090},
		{0.8090, -0.5000, -0.3090},
		{-0.9554, 0.0000, -0.2952},
		{0.9554, 0.0000, -0.2952},
		{-0.8090, 0.5000, -0.3090},
		{0.8090, 0.5000, -0.3090},
		{-0.4429, -0.8642, -0.2389},
		{-0.1625, -0.9511, -0.2629},
		{0.1625, -0.9511, -0.2629},
		{0.4429, -0.8642, -0.2389},
		{-0.4429, 0.8642, -0.2389},
		{-0.1625, 0.9511, -0.2629},
		{0.1625, 0.9511, -0.2629},
		{0.4429, 0.8642, -0.2389},
		{-0.6817, -0.7166, -0.1476},
		{0.6817, -0.7166, -0.1476},
		{-0.9511, -0.2629, -0.1625},
		{0.9511, -0.2629, -0.1625},
		{-0.9511, 0.2629, -0.1625},
		{0.9511, 0.2629, -0.1625},
		{-0.6817, 0.7166, -0.1476},
		{0.6817, 0.7166, -0.1476},
		{-0.5257, -0.8507, 0.0000},
		{-0.2952, -0.9554, 0.0000},
		{0.0000, -1.0000, 0.0000},
		{0.2952, -0.9554, 0.0000},
		{0.5257, -0.8507, 0.0000},
		{-0.8507, -0.5257, 0.0000},
		{0.8507, -0.5257, 0.0000},
		{-1.0000, 0.0000, 0.0000},
		{1.0000, 0.0000, 0.0000},
		{-0.8507, 0.5257, 0.0000},
		{0.8507, 0.5257, 0.0000},
		{-0.5257, 0.8507, 0.0000},
		{-0.2952, 0.9554, 0.0000},
		{0.0000, 1.0000, 0.0000},
		{0.2952, 0.9554, 0.0000},
		{0.5257, 0.8507, 0.0000},
		{-0.6817, -0.7166, 0.1476},
		{0.6817, -0.7166, 0.1476},
		{-0.9511, -0.2629, 0.1625},
		{0.9511, -0.2629, 0.1625},
		{-0.9511, 0.2629, 0.1625},
		{0.9511, 0.2629, 0.1625},
		{-0.6817, 0.7166, 0.1476},
		{0.6817, 0.7166, 0.1476},
		{-0.4429, -0.8642, 0.2389},
		{-0.1625, -0.9511, 0.2629},
		{0.1625, -0.9511, 0.2629},
		{0.4429, -0.8642, 0.2389},
		{-0.4429, 0.8642, 0.2389},
		{-0.1625, 0.9511, 0.2629},
		{0.1625, 0.9511, 0.2629},
		{0.4429, 0.8642, 0.2389},
		{-0.8090, -0.5000, 0.3090},
		{0.8090, -0.5000, 0.3090},
		{-0.9554, 0.0000, 0.2952},
		{0.9554, 0.0000, 0.2952},
		{-0.8090, 0.5000, 0.3090},
		{0.8090, 0.5000, 0.3090},
		{-0.5878, -0.6882, 0.4253},
		{0.5878, -0.6882, 0.4253},
		{-0.5878, 0.6882, 0.4253},
		{0.5878, 0.6882, 0.4253},
		{-0.8642, -0.2389, 0.4429},
		{0.8642, -0.2389, 0.4429},
		{-0.8642, 0.2389, 0.4429},
		{0.8642, 0.2389, 0.4429},
		{-0.3090, -0.8090, 0.5000},
		{0.0000, -0.8507, 0.5257},
		{0.3090, -0.8090, 0.5000},
		{-0.3090, 0.8090, 0.5000},
		{0.0000, 0.8507, 0.5257},
		{0.3090, 0.8090, 0.5000},
		{-0.6882, -0.4253, 0.5878},
		{0.6882, -0.4253, 0.5878},
		{-0.6882, 0.4253, 0.5878},
		{0.6882, 0.4253, 0.5878},
		{-0.8507, 0.0000, 0.5257},
		{0.8507, 0.0000, 0.5257},
		{-0.4253, -0.5878, 0.6882},
		{0.4253, -0.5878, 0.6882},
		{-0.4253, 0.5878, 0.6882},
		{0.4253, 0.5878, 0.6882},
		{-0.7166, -0.1476, 0.6817},
		{0.7166, -0.1476, 0.6817},
		{-0.7166, 0.1476, 0.6817},
		{0.7166, 0.1476, 0.6817},
		{-0.1476, -0.6817, 0.7166},
		{0.1476, -0.6817, 0.7166},
		{-0.1476, 0.6817, 0.7166},
		{0.1476, 0.6817, 0.7166},
		{-0.5000, -0.3090, 0.8090},
		{0.5000, -0.3090, 0.8090},
		{-0.5257, 0.0000, 0.8507},
		{0.5257, 0.0000, 0.8507},
		{-0.5000, 0.3090, 0.8090},
		{0.5000, 0.3090, 0.8090},
		{-0.2389, -0.4429, 0.8642},
		{0.2389, -0.4429, 0.8642},
		{-0.2629, -0.1625, 0.9511},
		{0.2629, -0.1625, 0.9511},
		{-0.2629, 0.1625, 0.9511},
		{0.2629, 0.1625, 0.9511},
		{-0.2389, 0.4429, 0.8642},
		{0.2389, 0.4429, 0.8642},
		{0.0000, -0.5257, 0.8507},
		{0.0000, -0.2952, 0.9554},
		{0.0000, 0.0000, 1.0000},
		{0.0000, 0.2952, 0.9554},
		{0.0000, 0.5257, 0.8507}};
const static int NUM_SPHERE_SAMPLES = 162;



void getAabb(const std::set<btRigidBody*>& objs, btVector3& allMin, btVector3& allMax) {
	assert(objs.size() > 0);
	allMin.setValue(SIMD_INFINITY, SIMD_INFINITY, SIMD_INFINITY);
	allMax.setValue(-SIMD_INFINITY, -SIMD_INFINITY, -SIMD_INFINITY);
	btVector3 oneMin, oneMax;
	BOOST_FOREACH(btRigidBody* obj, objs) {
		obj->getAabb(oneMin, oneMax);
		allMin.setMin(oneMin);
		allMax.setMax(oneMax);
	}
}

void getBoundingSphere(const std::set<btRigidBody*>& objs, btVector3& center, float& radius) {
	btVector3 mins, maxes;
	getAabb(objs, mins, maxes);
	center = (mins+maxes)/2;
	radius = (maxes - center).length();
}


CloudPtr downsampleCloud1(const CloudPtr& in, float sz) {
  CloudPtr out(new Cloud());
  pcl::VoxelGrid<Point> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(sz,sz,sz);
  vg.filter(*out);
  return out;
}

void getSurfacePoints(const std::set<btRigidBody*> objs, const btCollisionWorld* world, std::vector<btVector3>& surfacePoints, std::vector<btRigidBody*>& ownerBodies) {
	btVector3 sphereCenter;
	float sphereRadius;
	getBoundingSphere(objs, sphereCenter, sphereRadius);

	for (int i=0; i < NUM_SPHERE_SAMPLES; ++i) {
		for (int j=0; j < NUM_SPHERE_SAMPLES; ++j) {
			btVector3 start(SPHERE_SAMPLES[i][0], SPHERE_SAMPLES[i][1], SPHERE_SAMPLES[i][2]);
			btVector3 end(SPHERE_SAMPLES[j][0], SPHERE_SAMPLES[j][1], SPHERE_SAMPLES[j][2]);
			start *= sphereRadius;
			start += sphereCenter;
			end *= sphereRadius;
			end += sphereCenter;

#if 1
			btCollisionWorld::AllHitsRayResultCallback rayCallback(start, end);
			world->rayTest(start, end, rayCallback);
			int nHits = rayCallback.m_hitFractions.size();
			for (int k=0; k < nHits; ++k) {
				btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObjects[k]);
				if (objs.find(body) != objs.end()) {
					surfacePoints.push_back(rayCallback.m_hitPointWorld[k]);
					ownerBodies.push_back(body);
				}
			}
#else
			btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
			world->rayTest(start, end, rayCallback);
			if (rayCallback.hasHit()) {
				btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
				if (objs.find(body) != objs.end()) {
					surfacePoints.push_back(rayCallback.m_hitPointWorld);
					ownerBodies.push_back(body);
				}
			}
#endif
		}
	}
}

// todo: use normals to ensure correct behavior on thin objects

void downsamplePoints(const std::vector<btVector3>& oldPts, float voxelSize, std::vector<btVector3>& newPts) {
	CloudPtr oldCloud(new Cloud());
	oldCloud->points.assign(reinterpret_cast<const Point*>(&oldPts[0]), reinterpret_cast<const Point*>(&oldPts[0]+oldPts.size()));
	oldCloud->width = oldPts.size();
	oldCloud->height = 1;
	oldCloud->is_dense = true;
	CloudPtr newCloud = downsampleCloud1(oldCloud, voxelSize);
	newPts.assign(reinterpret_cast<const btVector3*>(&newCloud->points[0]), reinterpret_cast<const btVector3*>(&newCloud->points[0]+newCloud->size()));
}


void downsamplePointsOnEachBody(const std::vector<btVector3>& oldPts, const std::vector<btRigidBody*>& oldBodies, float voxelSize, std::vector<btVector3>& newPts, std::vector<btRigidBody*>& newBodies) {
	typedef std::map<btRigidBody*, vector<btVector3> > BodyToPointsMap;
	BodyToPointsMap body2points;
	for (int i=0; i < oldPts.size(); ++i) {
		body2points[oldBodies[i]].push_back(oldPts[i]);
	}
	for (BodyToPointsMap::iterator it = body2points.begin(); it != body2points.end(); ++it) {
		vector<btVector3>& oldPointsOnBody = it->second;
		vector<btVector3> dsPointsOnBody;
		downsamplePoints(oldPointsOnBody, voxelSize, dsPointsOnBody);
		BOOST_FOREACH(btVector3& bodyPt, dsPointsOnBody) {
			newPts.push_back(bodyPt);
			newBodies.push_back(it->first);
		}
	}
}

void getSampledDescription(CompoundObject<BulletObject>::Ptr compoundObj, btCollisionWorld* world, float voxelSize, std::vector<int>& bodyInds, std::vector<btVector3>& pointRelPositions) {


	set<btRigidBody*> objs;
	map<btRigidBody*, int> body2ind;
	for (int i=0; i < compoundObj->children.size(); ++i) {
		body2ind[compoundObj->children[i]->rigidBody.get()] = i;
		objs.insert(compoundObj->children[i]->rigidBody.get());
	}

	vector<btVector3> surfacePoints;
	vector<btRigidBody*> ownerBodies;
	getSurfacePoints(objs, world, surfacePoints, ownerBodies);
	vector<btVector3> dsSurfacePoints;
	vector<btRigidBody*> dsOwnerBodies;
	downsamplePointsOnEachBody(surfacePoints, ownerBodies, .015*METERS, dsSurfacePoints, dsOwnerBodies);

	int nPtsDS = dsOwnerBodies.size();
	bodyInds.resize(nPtsDS);
	pointRelPositions.resize(nPtsDS);
	for (int i=0; i < nPtsDS; ++i) {
		bodyInds[i] = body2ind[dsOwnerBodies[i]];
		pointRelPositions[i] = dsOwnerBodies[i]->getCenterOfMassTransform().invXform(dsSurfacePoints[i]);
	}
}

