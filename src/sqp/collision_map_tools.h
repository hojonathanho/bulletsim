#pragma once
#include "collision_boxes.h"
#include "clouds/utils_pcl.h"
#include "simulation/simulation_fwd.h"

CollisionBoxes::Ptr collisionBoxesFromPointCloud(ColorCloudPtr cloud, float voxelSize);
BulletObjectPtr collisionMeshFromPointCloud(ColorCloudPtr colorCloud, float voxelSize);
