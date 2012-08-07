#pragma once

enum
{
	FEAT_ALL = -1,
	FEAT_XYZ,
	FEAT_BGR,
	FEAT_LAB,
	FEAT_NORMAL,
	FEAT_LABEL,

	FEAT_COUNT
};
const static size_t FEATURE_SIZES[] = {3,3,3,3,1};
typedef int FeatureType;
