/*
 * softBodyHelpers.cpp
 *
 *  Created on: Aug 18, 2012
 *      Author: alex
 */

#include "softBodyHelpers.h"
#include "util.h"
#include <algorithm>
#include "utils/utils_vector.h"
#include "utils/conversions.h"
#include "utils/cvmat.h"
#include "utils/config.h"

using namespace std;

btSoftBody*	CreateFromSoftBodyExcludeNodes(btSoftBody* softBody, vector<int> exclude_nodes_idx)
{
	btSoftBody::tNodeArray& nodes = softBody->m_nodes;
	btSoftBody::tFaceArray& faces = softBody->m_faces;
	btSoftBody::tLinkArray& links = softBody->m_links;

	// compute nodes to exclude (sorted)
	sort(exclude_nodes_idx.begin(), exclude_nodes_idx.end());

	// compute face to nodes indices
	vector<vector<int> > face2nodes(faces.size(), vector<int>(3,-1));
	for (int i=0; i<nodes.size(); i++) {
		int j,c;
		for(j=0; j<faces.size(); j++) {
			for(c=0; c<3; c++) {
				if (&nodes[i] == faces[j].m_n[c]) {
					face2nodes[j][c] = i;
				}
			}
		}
	}

	// compute link to nodes indices
	vector<vector<int> > link2nodes(links.size(), vector<int>(2,-1));
	for (int i=0; i<nodes.size(); i++) {
		int l,c;
		for(l=0; l<links.size(); l++) {
			for(c=0; c<2; c++) {
				if (&nodes[i] == links[l].m_n[c]) {
					link2nodes[l][c] = i;
				}
			}
		}
	}

	/* Create nodes	*/
	const int	tot=nodes.size() - exclude_nodes_idx.size();
	btVector3*	x=new btVector3[tot];
	btScalar*	m=new btScalar[tot];

	int exclude_idx = 0;
	int idx=0;
	vector<int> oldNodes2newNodes(nodes.size());
	for (int i=0; i<nodes.size(); i++) {
		if (exclude_nodes_idx[exclude_idx] == i) {
			oldNodes2newNodes[i] = -1;
			exclude_idx++;
		} else {
			x[idx] = nodes[i].m_x;
			m[idx] = 1.0;// cluster-cluster collision explodes when set to this 1.0/nodes[i].m_im;
			oldNodes2newNodes[i] = idx;
			idx++;
		}
	}
	assert(exclude_idx == exclude_nodes_idx.size());
	assert(idx == tot);

	btSoftBody*		psb=new btSoftBody(softBody->m_worldInfo,tot,x,m);
	delete[] x;
	delete[] m;

	/* Create faces */
	for (int j=0; j<faces.size(); j++) {
		assert(face2nodes[j].size() == 3);
		for (int c=0; c<3; c++) assert(face2nodes[j][c] != -1);
		int node0 = oldNodes2newNodes[face2nodes[j][0]];
		int node1 = oldNodes2newNodes[face2nodes[j][1]];
		int node2 = oldNodes2newNodes[face2nodes[j][2]];
		if (node0 == -1 || node1 == -1 || node2 == -1) continue;
		psb->appendFace(node0, node1, node2);
	}

	/* Create links */
	for (int l=0; l<links.size(); l++) {
		assert(link2nodes[l].size() == 2);
		for (int c=0; c<2; c++) assert(link2nodes[l][c] != -1);
		int node0 = oldNodes2newNodes[link2nodes[l][0]];
		int node1 = oldNodes2newNodes[link2nodes[l][1]];
		if (node0 == -1 || node1 == -1) continue;
		psb->appendLink(node0, node1);
	}

	return psb;
}


btSoftBody*	CreateFromSoftBodyExcludeFaces(btSoftBody* softBody, vector<int> exclude_faces_idx)
{
	btSoftBody::tNodeArray& nodes = softBody->m_nodes;
	btSoftBody::tFaceArray& faces = softBody->m_faces;
	btSoftBody::tLinkArray& links = softBody->m_links;

	// compute faces to nodes indices and vice versa
	vector<vector<int> > node2faces(nodes.size());
	vector<vector<int> > face2nodes(faces.size(), vector<int>(3,-1));
	for (int i=0; i<nodes.size(); i++) {
		int j,c;
		for(j=0; j<faces.size(); j++) {
			for(c=0; c<3; c++) {
				if (&nodes[i] == faces[j].m_n[c]) {
					node2faces[i].push_back(j);
					face2nodes[j][c] = i;
				}
			}
		}
	}
	// remove all the instances of excluded faces from node2faces
	// so, if all the faces attached to node are excluded, then
	// node2faces[node].size()==0 and this node should be excluded
	for (int j=0; j<exclude_faces_idx.size(); j++) {
		for (int c=0; c<3; c++) {
			int node = face2nodes[exclude_faces_idx[j]][c];
			remove(node2faces[node], exclude_faces_idx[j]);
		}
	}

	// compute link to nodes indices
	vector<vector<int> > link2nodes(links.size(), vector<int>(2,-1));
	for (int i=0; i<nodes.size(); i++) {
		int l,c;
		for(l=0; l<links.size(); l++) {
			for(c=0; c<2; c++) {
				if (&nodes[i] == links[l].m_n[c]) {
					link2nodes[l][c] = i;
				}
			}
		}
	}

	/* Create nodes	*/
	int exclude_nodes_size = 0;
	for (int i=0; i<nodes.size(); i++)
		if (node2faces[i].size() == 0)
			exclude_nodes_size++;
	const int	tot=nodes.size() - exclude_nodes_size;
	btVector3*	x=new btVector3[tot];
	btScalar*	m=new btScalar[tot];

	int idx=0;
	vector<int> oldNodes2newNodes(nodes.size());
	for (int i=0; i<nodes.size(); i++) {
		if (node2faces[i].size() == 0) {
			oldNodes2newNodes[i] = -1;
		} else {
			x[idx] = nodes[i].m_x;
			m[idx] = 1.0;// cluster-cluster collision explodes when set to this 1.0/nodes[i].m_im;
			oldNodes2newNodes[i] = idx;
			idx++;
		}
	}
	assert(idx == tot);

	btSoftBody*		psb=new btSoftBody(softBody->m_worldInfo,tot,x,m);
	delete[] x;
	delete[] m;

	/* Create faces */
	int exclude_idx = 0;
	idx = 0;
	for (int j=0; j<faces.size(); j++) {
		if (j == exclude_faces_idx[exclude_idx]) {
			exclude_idx++;
		} else {
			assert(face2nodes[j].size() == 3);
			for (int c=0; c<3; c++) assert(face2nodes[j][c] != -1);
			int node0 = oldNodes2newNodes[face2nodes[j][0]];
			int node1 = oldNodes2newNodes[face2nodes[j][1]];
			int node2 = oldNodes2newNodes[face2nodes[j][2]];
			if (node0 == -1 || node1 == -1 || node2 == -1) continue;
			psb->appendFace(node0, node1, node2);
			idx++;
		}
	}
	assert(exclude_idx == exclude_faces_idx.size());
	assert(idx == (faces.size()-exclude_faces_idx.size()));

	/* Create links */
	for (int l=0; l<links.size(); l++) {
		assert(link2nodes[l].size() == 2);
		for (int c=0; c<2; c++) assert(link2nodes[l][c] != -1);
		int node0 = oldNodes2newNodes[link2nodes[l][0]];
		int node1 = oldNodes2newNodes[link2nodes[l][1]];
		if (node0 == -1 || node1 == -1) continue;
		psb->appendLink(node0, node1);
	}

	return psb;
}

// assumes the polygon is in the xy plane
struct	ImplicitPolygon : btSoftBody::ImplicitFn
{
	vector<btVector3> corners;
	vector<cv::Point> corners_2d;

	ImplicitPolygon() {}
	ImplicitPolygon(const vector<btVector3>& c) : corners(c)
	{
		for (int i=0; i<corners.size(); i++) {
			corners_2d.push_back(cv::Point(corners[i].x(), corners[i].y()));
		}
	}

	btScalar	Eval(const btVector3& point)
	{
		return btScalar(cv::pointPolygonTest(corners_2d, cv::Point2f(point.x(), point.y()), true));
	}

	btScalar	EvalFast(const btVector3& point)
	{
		return btScalar(cv::pointPolygonTest(corners_2d, cv::Point2f(point.x(), point.y()), false));
	}
};

//assumes all the corners are in a plane
//the corners are specified in a clockwise order
btSoftBody* CreatePolygonPatch(btSoftBodyWorldInfo& worldInfo, std::vector<btVector3> corners, int resx, int resy, bool gendiags) {

	//DEBUG
//	float factor = 1/((float)corners.size()-1);
//	for (int i=0; i<corners.size(); i++) {
//		util::drawSpheres(corners[i], Vector3f(0,((float)i)*factor,1-((float)i)*factor), 0.5, 4, env);
//	}

	// compute the transformation for the corners to lie in the xy plane
	btVector3 normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]).normalized();
	btVector3 align(0,0,1);
	btScalar angle = align.angle(normal);
	btVector3 axis = normal.cross(align);
	btMatrix3x3 align_rot(btQuaternion(axis, angle));

	btTransform align_transform(align_rot, btVector3(0,0,-(align_rot*corners[0]).z()));
	//if align_rot is nan, there is probably no alignment needed (degenerate case)
	if (!util::isfinite(align_rot))
		align_transform = btTransform::getIdentity();

	// transform corners so that their z axis are zero
	vector<btVector3> xy_corners(corners.size());
	for (int i=0; i<xy_corners.size(); i++)
		xy_corners[i] = align_transform * corners[i];

	// transformed corners with their z axis dropped off
	vector<cv::Point2f> corners_2d(xy_corners.size());
	for (int i; i<corners_2d.size(); i++)
		corners_2d[i] = cv::Point2f(xy_corners[i].x(), xy_corners[i].y());

	// transformed rectangle corners of the transformed corners
	cv::RotatedRect rect = cv::minAreaRect(corners_2d);
	vector<cv::Point2f> rect_corners_2d = rectCorners(rect);

	// transformed rectangle corners with their z axis put back in
	vector<btVector3> xy_rect_corners(rect_corners_2d.size());
	for (int i=0; i<xy_rect_corners.size(); i++)
		xy_rect_corners[i] = btVector3(rect_corners_2d[i].x, rect_corners_2d[i].y, 0);

	btSoftBody* psb = btSoftBodyHelpers::CreatePatch(worldInfo, xy_rect_corners[0], xy_rect_corners[1], xy_rect_corners[3], xy_rect_corners[2], resx, resy, 0, gendiags);

	// Refine the patch around the polygon contour
	ImplicitPolygon ipolygon(xy_corners);
	psb->refine(&ipolygon, 0.00001 * METERS, false);

	// Determine the faces that are outside the polygon contour
	vector<int> exclude_faces_idx;
	btSoftBody::tFaceArray& faces = psb->m_faces;
	for (int j=0; j<faces.size(); j++)
		if (ipolygon.EvalFast((faces[j].m_n[0]->m_x+faces[j].m_n[1]->m_x+faces[j].m_n[2]->m_x)/3.0) < 0) exclude_faces_idx.push_back(j);

	// Create a new btSoftBody containing only the nodes, faces and links inside the polygon contour
	psb = CreateFromSoftBodyExcludeFaces(psb, exclude_faces_idx);

	// rotate the nodes back to their unaligned positions
	psb->transform(align_transform.inverse());

	return psb;
}






