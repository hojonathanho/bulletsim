/*
 * tetgen_helpers.h
 *
 *  Created on: Aug 2, 2012
 *      Author: alex
 */

#ifndef TETGEN_HELPERS_H_
#define TETGEN_HELPERS_H_

#include "tetgen.h"
#include <vector>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

btSoftBody* CreateFromTetGenFile(btSoftBodyWorldInfo& worldInfo,
																	const char* ele_filename,
																	const char* face_filename,
																	const char* node_filename,
																	bool bfacelinks,
																	bool btetralinks,
																	bool bfacesfromtetras);

// quality:  Quality mesh generation. Minimum radius-edge ratio.
// max_tet_vol: Maximum tetrahedron volume constraint.
btSoftBody* CreatePrism(btSoftBodyWorldInfo& worldInfo,
		const std::vector<btVector3>& corners_base,
		const btVector3 &polygon_translation,
		float quality,
		float max_tet_vol,
		bool bfacelinks,
		bool btetralinks,
		bool bfacesfromtetras);

#endif /* TETGEN_HELPERS_H_ */
