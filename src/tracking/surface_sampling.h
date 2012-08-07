#pragma once
#include <simulation/environment.h>
#include <simulation/basicobjects.h>
void getSurfacePoints(const std::set<btRigidBody*> objs, const btDynamicsWorld* world, std::vector<btVector3>& surfacePoints, std::vector<btRigidBody*>& ownerBodies);
void downsamplePointsOnEachBody(const std::vector<btVector3>& oldPts, const std::vector<btRigidBody*>& oldBodies, float voxelSize, std::vector<btVector3>& newPts, std::vector<btRigidBody*>& newBodies);
void getSampledDescription(CompoundObject<BulletObject>::Ptr, btDynamicsWorld*, float voxelSize, std::vector<int>& bodyInds, std::vector<btVector3> offsets);
