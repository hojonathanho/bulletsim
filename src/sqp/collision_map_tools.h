#pragma once
#include "collision_boxes.h"
#include "clouds/utils_pcl.h"

CollisionBoxes::Ptr collisionBoxesFromPointCloud(ColorCloudPtr cloud, float voxelSize);
