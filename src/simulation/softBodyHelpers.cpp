/*
 * softBodyHelpers.cpp
 *
 *  Created on: Aug 18, 2012
 *      Author: alex
 */


#include <BulletSoftBody/btSoftBodyInternals.h>
#include "softBodyHelpers.h"
#include "util.h"
#include <algorithm>
#include "utils/utils_vector.h"
#include "utils/conversions.h"
#include "utils/cvmat.h"
#include "utils/config.h"
#include "simulation/bullet_io.h"

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


btMatrix3x3 calcAlignRot(btVector3 v0, btVector3 v1) {
  // find rotation R so that R*v0 || v1
  v0.normalize();
  v1.normalize();
  if  ( (v0-v1).length2() < 1e-9 ) return btMatrix3x3::getIdentity();
  else if ((v0+v1).length2() < 1e-9) {
    btVector3 randvec(0.0567162 , -0.28040618,  0.95820439);
    btVector3 ax = randvec.cross(v0);
    return btMatrix3x3(btQuaternion(ax,SIMD_PI));
  }
  else {
    btScalar angle = v0.angle(v1);
    btVector3 axis = v0.cross(v1);
    return btMatrix3x3(btQuaternion(axis, angle));
  }
}

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

	btMatrix3x3 align_rot = calcAlignRot(normal, btVector3(0,0,1));
	btTransform align_transform(align_rot, btVector3(0,0,-(align_rot*corners[0]).z()));
	//if align_rot is nan, there is probably no alignment needed (degenerate case)

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

//	for (int i=0; i < psb->m_nodes.size(); ++i) {
//	    util::drawSpheres(psb->m_nodes[i].m_x, Eigen::Vector3f(0,0,1), 1, .01*METERS, util::getGlobalEnv());
//	}
	// rotate the nodes back to their unaligned positions
	psb->transform(align_transform.inverse());
//	cout << align_transform << endl;
//    for (int i=0; i < psb->m_nodes.size(); ++i) {
//        util::drawSpheres(psb->m_nodes[i].m_x, Eigen::Vector3f(0,1,0), 1, .011*METERS, util::getGlobalEnv());
//    }
	return psb;
}


static btScalar		ImplicitSolve(	cutPlane* inf,
								  	btVector3& a,
								  	btVector3& b,
								  	const btScalar accuracy,
								  	const int maxiterations) {
	btScalar	span[2]={0,1};
	btScalar	values[2]={inf->Eval(a),inf->Eval(b)};
	if(values[0]>values[1])
	{
		btSwap(span[0],span[1]);
		btSwap(values[0],values[1]);
	}
	if(values[0]>-accuracy) return(-1);
	if(values[1]<+accuracy) return(-1);
	for(int i=0;i<maxiterations;++i)
	{
		const btScalar	t=Lerp(span[0],span[1],values[0]/(values[0]-values[1]));
		btVector3 c = Lerp(a,b,t);
		const btScalar	v=inf->Eval(c);
		if((t<=0)||(t>=1))		break;
		if(btFabs(v)<accuracy)	return(t);
		if(v<0)
		{ span[0]=t;values[0]=v; }
		else
		{ span[1]=t;values[1]=v; }
	}
	return(-1);
}



/** Function for cutting a planar soft-body, using an implicit function.
 *  Based on btSoftBody::refine.
 *
 *  @param
 *   - psb            : pointer to the soft-body to be cut.
 *   - info           : pointer to cut-plane information
 *   - accuracy       : the length of the region around the cut which belongs to both the sides
 *   - fill_cut_nodes : if true, the vectors cut_nodes1 and cut_nodes2
 *   	                 are populated with the indices of the nodes on the two sides of the cut.
 *  */
void CutPlaneSoftBody(btSoftBody* psb, cutPlane* info, btScalar accuracy,
					  std::vector<int> &cut_nodes1,
					  std::vector<int> &cut_nodes2, bool fill_cut_nodes) {


	// if have to fill information about cut nodes => clear the indices arrays
	if (fill_cut_nodes) {
		cut_nodes1.clear();
		cut_nodes2.clear();
	}

	// First node address
	const btSoftBody::Node*	nbase = &(psb->m_nodes[0]);

	// Number of nodes (initial)
	int					ncount = psb->m_nodes.size();

	// Undirected graph representation: represents links and face-edges b/w nodes
	btSymMatrix<int>	edges(ncount,-2);

	int	newnodes=0;
	int i,j,k,ni;

	//printf("Number of initial links: %d\n", psb->m_links.size());


	/* Filter out		*/
	//Remove BENDING links connecting nodes on different sides of implicit function boundary
	for(i=0;i<psb->m_links.size();++i) {
		btSoftBody::Link&	l=psb->m_links[i];
		if(l.m_bbending) {
			btScalar t = ImplicitSolve(info, l.m_n[0]->m_x, l.m_n[1]->m_x,accuracy);
			if (t<0) continue;
			const btVector3 x = Lerp(l.m_n[0]->m_x, l.m_n[1]->m_x,t);
			if (info->shouldConsider(x)){
				if(!SameSign(info->Eval(l.m_n[0]->m_x),info->Eval(l.m_n[1]->m_x)))
				{
					btSwap(psb->m_links[i],psb->m_links[psb->m_links.size()-1]);
					psb->m_links.pop_back();--i;
				}
			}
		}
	}
	//printf("Number of links after rough filtering: %d\n", psb->m_links.size());

	/* Fill edges : Build the edge matrix, by going over links and faces*/
	for(i=0;i<psb->m_links.size();++i) { // LINKS
		btSoftBody::Link&	l=psb->m_links[i];
		edges(int(l.m_n[0]-nbase),int(l.m_n[1]-nbase))=-1;
	}


	for(i=0;i<psb->m_faces.size();++i) {// FACE-EDGES
		btSoftBody::Face&	f=psb->m_faces[i];
		edges(int(f.m_n[0]-nbase),int(f.m_n[1]-nbase))=-1;
		edges(int(f.m_n[1]-nbase),int(f.m_n[2]-nbase))=-1;
		edges(int(f.m_n[2]-nbase),int(f.m_n[0]-nbase))=-1;
	}


	/* Intersect		*/
	for(i=0;i<ncount;++i) {
		for(j=i+1;j<ncount;++j) {
			// For all pairs of nodes:
			if(edges(i,j)==-1)  // if there is an edge b/w them
			{
				btSoftBody::Node&			a=psb->m_nodes[i];  // a <-- the first node
				btSoftBody::Node&			b=psb->m_nodes[j];  // b <-- the second node
				const btScalar	t=ImplicitSolve(info,a.m_x,b.m_x,accuracy);  // t : ta + (1-t)b is on the edge if 0<t, else -1
				if(t>0)   // if the nodes are on the opposite sides of the boundary
				{
					//printf("found a link connecting opposite sides\n");
					const btVector3	x=Lerp(a.m_x,b.m_x,t); // x <-- position of the node at the boundary: linear interpolation of the positions of a and b

					if (!info->shouldConsider(x)) continue;

					const btVector3	v=Lerp(a.m_v,b.m_v,t); // v <-- velocity of the node at the boundary: linear interpolation of the velocities of a and b

					// find out the mass of the new node.
					btScalar		m=0;
					if(a.m_im>0)  // if a movable
					{
						if(b.m_im>0) // if a and b are movable, m is finite (!=0) else 0
						{
							const btScalar	ma=1/a.m_im;
							const btScalar	mb=1/b.m_im;
							const btScalar	mc=Lerp(ma,mb,t);
							const btScalar	f=(ma+mb)/(ma+mb+mc);
							a.m_im=1/(ma*f);
							b.m_im=1/(mb*f);
							m=mc*f;
						}
						else
						{ a.m_im/=0.5;m=1/a.m_im; }
					}
					else
					{
						if(b.m_im>0)
						{ b.m_im/=0.5;m=1/b.m_im; }
						else
							m=0;
					}

					// Create a new node with interpolated position and mass
					psb->appendNode(x,m);
					edges(i,j)=psb->m_nodes.size()-1;  // edges(i,j) <-- index of the interpolated new node ( > 0)
					psb->m_nodes[edges(i,j)].m_v=v;
					++newnodes;
				}
			}
		}
	}
	//printf("number of interpolated nodes : %d\n", newnodes);


	nbase=&psb->m_nodes[0];  // <-- address of the first node
	/* Refine links
	 *
	 *  a------->b |==> a------->t---->b
	 *  t : interpolated node
	 **/
	for(i=0,ni=psb->m_links.size();i<ni;++i)
	{
		btSoftBody::Link&		feat=psb->m_links[i];
		const int	idx[]={	int(feat.m_n[0]-nbase),
				int(feat.m_n[1]-nbase) };
		if((idx[0]<ncount)&&(idx[1]<ncount))
		{
			const int ni=edges(idx[0],idx[1]);
			if(ni>0)
			{
				psb->appendLink(i);
				btSoftBody::Link*		pft[]={	&psb->m_links[i],
						&psb->m_links[psb->m_links.size()-1]};
				pft[0]->m_n[0]=&psb->m_nodes[idx[0]];
				pft[0]->m_n[1]=&psb->m_nodes[ni];
				pft[1]->m_n[0]=&psb->m_nodes[ni];
				pft[1]->m_n[1]=&psb->m_nodes[idx[1]];
			}
		}
	}
	//printf("Number of links after edge-interpolation: %d\n", psb->m_links.size());


	/* Refine faces:
	 * Similar to "edge-breaking" for links [above]
	 * */
	for(i=0;i<psb->m_faces.size();++i)
	{
		const btSoftBody::Face&	feat=psb->m_faces[i];
		const int	idx[]={	int(feat.m_n[0]-nbase),
				int(feat.m_n[1]-nbase),
				int(feat.m_n[2]-nbase)};
		for(j=2,k=0;k<3;j=k++)
		{
			if((idx[j]<ncount)&&(idx[k]<ncount))
			{
				const int ni=edges(idx[j],idx[k]);
				if(ni>0)
				{
					psb->appendFace(i);
					const int	l=(k+1)%3;
					btSoftBody::Face*		pft[]={	&psb->m_faces[i],
							&psb->m_faces[psb->m_faces.size()-1]};
					pft[0]->m_n[0]=&psb->m_nodes[idx[l]];
					pft[0]->m_n[1]=&psb->m_nodes[idx[j]];
					pft[0]->m_n[2]=&psb->m_nodes[ni];
					pft[1]->m_n[0]=&psb->m_nodes[ni];
					pft[1]->m_n[1]=&psb->m_nodes[idx[k]];
					pft[1]->m_n[2]=&psb->m_nodes[idx[l]];
					psb->appendLink(ni,idx[l],pft[0]->m_material);
					--i;break;
				}
			}
		}
	}
	//printf("Number of links after face refine: %d\n", psb->m_links.size());



	/* Cut	: Deletes the links/ faces/ nodes/ anchors. */
	btAlignedObjectArray<int>	cnodes;
	const int			 pcount=ncount;
	ncount=psb->m_nodes.size();
	cnodes.resize(ncount,0);

	nbase=&psb->m_nodes[0];



	for(i=0,ni=psb->m_links.size();i<ni;++i)  {
		const int		id[]={	int(psb->m_links[i].m_n[0]-nbase),
				                int(psb->m_links[i].m_n[1]-nbase) };

		btVector3 n1 = psb->m_nodes[id[0]].m_x;

		btVector3 n2 = psb->m_nodes[id[1]].m_x;


		btScalar eval1 = info->Eval(n1);
		btScalar eval2 = info->Eval(n2);


		bool   should1 = info->shouldConsider(n1);
		bool   should2 = info->shouldConsider(n2);

		if (btFabs(eval1) < accuracy
				&& btFabs (eval2) < accuracy
				&& eval1*eval2 < 0
				&& should1 && should2) { // throw out the link.
			btSwap(psb->m_links[i],psb->m_links[psb->m_links.size()-1]);
			psb->m_links.pop_back();--i;
		} else {
			if (btFabs(eval1) < accuracy
					&& should1) {
				if (eval2 > accuracy) { // (2) in far +ve. (1) is within accuracy and is shouldConsider
					if(!cnodes[id[0]]) {// create a copy of the node
						const btVector3	v=psb->m_nodes[id[0]].m_v;
						btScalar		m=psb->getMass(id[0]);
						if(m>0) { m*=0.5;psb->m_nodes[id[0]].m_im/=0.5; }
						psb->appendNode(n1,m);
						cnodes[id[0]]=psb->m_nodes.size()-1;
						psb->m_nodes[cnodes[id[0]]].m_v=v;

						// store the indices of the nodes on either side of the cut
						if (fill_cut_nodes) {
							cut_nodes1.push_back(id[0]);
							cut_nodes2.push_back(psb->m_nodes.size()-1);
						}
					}
					psb->m_links[i].m_n[0] = &psb->m_nodes[cnodes[id[0]]];
				}
			} else if (btFabs(eval2) < accuracy
					&& should2) {
				if (eval1 > accuracy) { // (1) in far +ve. (2) is within accuracy and is shouldConsider
					if(!cnodes[id[1]]) {// create a copy of the node
						const btVector3	v=psb->m_nodes[id[1]].m_v;
						btScalar		m=psb->getMass(id[1]);
						if(m>0) { m*=0.5;psb->m_nodes[id[1]].m_im/=0.5; }
						psb->appendNode(n2,m);
						cnodes[id[1]]=psb->m_nodes.size()-1;
						psb->m_nodes[cnodes[id[1]]].m_v=v;

						// store the indices of the nodes on either side of the cut
						if (fill_cut_nodes) {
							cut_nodes1.push_back(id[1]);
							cut_nodes2.push_back(psb->m_nodes.size()-1);
						}
					}
					psb->m_links[i].m_n[1] = &psb->m_nodes[cnodes[id[1]]];

				}
			}
		}
	}


	/* Faces : split faces 	*/
	for(i=0,ni=psb->m_faces.size();i<ni;++i) {
		btSoftBody::Node**	n  = psb->m_faces[i].m_n;
		const int	id[]={	int(n[0]-nbase),
				            int(n[1]-nbase),
				            int(n[2]-nbase) };


		btVector3 n0 = n[0]->m_x;
		btVector3 n1 = n[1]->m_x;
		btVector3 n2 = n[2]->m_x;

		btScalar eval0 = info->Eval(n0);
		btScalar eval1 = info->Eval(n1);
		btScalar eval2 = info->Eval(n2);

		bool   should0 = info->shouldConsider(n0);
		bool   should1 = info->shouldConsider(n1);
		bool   should2 = info->shouldConsider(n2);

		if( (eval0 > accuracy)
				||(eval1 > accuracy)
				||(eval2 > accuracy) ) {

			int iden = 0;

			// for node 0:
			if (btFabs(eval0) < accuracy && should0 ) {
				if(!cnodes[id[0]]) {// create a copy of the node
					const btVector3	v=psb->m_nodes[id[0]].m_v;
					btScalar		m=psb->getMass(id[0]);
					if(m>0) { m*=0.5;psb->m_nodes[id[0]].m_im/=0.5; }
					psb->appendNode(n0,m);
					cnodes[id[0]]=psb->m_nodes.size()-1;
					psb->m_nodes[cnodes[id[0]]].m_v=v;

					// store the indices of the nodes on either side of the cut
					if (fill_cut_nodes) {
						cut_nodes1.push_back(id[0]);
						cut_nodes2.push_back(psb->m_nodes.size()-1);
					}

				}
				n[0] = &psb->m_nodes[cnodes[id[0]]];
				iden += 1;
			}

			// for node 1:
			if (btFabs(eval1) < accuracy && should1 ) {
				if(!cnodes[id[1]]) {// create a copy of the node
					const btVector3	v=psb->m_nodes[id[1]].m_v;
					btScalar		m=psb->getMass(id[1]);
					if(m>0) { m*=0.5;psb->m_nodes[id[1]].m_im/=0.5; }
					psb->appendNode(n1,m);
					cnodes[id[1]]=psb->m_nodes.size()-1;
					psb->m_nodes[cnodes[id[1]]].m_v=v;

					// store the indices of the nodes on either side of the cut
					if (fill_cut_nodes) {
						cut_nodes1.push_back(id[1]);
						cut_nodes2.push_back(psb->m_nodes.size()-1);
					}
				}
				n[1] = &psb->m_nodes[cnodes[id[1]]];
				iden += 10;
			}

			// for node 2:
			if (btFabs(eval2) < accuracy && should2 ) {
				if(!cnodes[id[2]]) {// create a copy of the node
					const btVector3	v=psb->m_nodes[id[2]].m_v;
					btScalar		m=psb->getMass(id[2]);
					if(m>0) { m*=0.5;psb->m_nodes[id[2]].m_im/=0.5; }
					psb->appendNode(n2,m);
					cnodes[id[2]]=psb->m_nodes.size()-1;
					psb->m_nodes[cnodes[id[2]]].m_v=v;

					// store the indices of the nodes on either side of the cut
					if (fill_cut_nodes) {
						cut_nodes1.push_back(id[2]);
						cut_nodes2.push_back(psb->m_nodes.size()-1);
					}
				}
				n[2] = &psb->m_nodes[cnodes[id[2]]];
				iden += 100;
			}

			switch(iden) {
			case 11:
				psb->appendLink(cnodes[id[0]], cnodes[id[1]], psb->m_faces[i].m_material, true);
				break;
			case 101:
				psb->appendLink(cnodes[id[0]], cnodes[id[2]], psb->m_faces[i].m_material, true);
				break;
			case 110:
				psb->appendLink(cnodes[id[1]], cnodes[id[2]], psb->m_faces[i].m_material, true);
				break;
			}
		}
	}


	/* Clean orphans	*/
	int							nnodes=psb->m_nodes.size();
	btAlignedObjectArray<int>	ranks;
	btAlignedObjectArray<int>	todelete;
	ranks.resize(nnodes,0);
	for(i=0,ni=psb->m_links.size();i<ni;++i)
	{
		for(int j=0;j<2;++j) ranks[int(psb->m_links[i].m_n[j]-nbase)]++;
	}
	for(i=0,ni=psb->m_faces.size();i<ni;++i)
	{
		for(int j=0;j<3;++j) ranks[int(psb->m_faces[i].m_n[j]-nbase)]++;
	}

	for(i=0;i<psb->m_links.size();++i)
	{
		const int	id[]={	int(psb->m_links[i].m_n[0]-nbase),
				int(psb->m_links[i].m_n[1]-nbase)};
		const bool	sg[]={	ranks[id[0]]==1,
				ranks[id[1]]==1};
		if(sg[0]||sg[1])
		{
			--ranks[id[0]];
			--ranks[id[1]];
			btSwap(psb->m_links[i],psb->m_links[psb->m_links.size()-1]);
			psb->m_links.pop_back();--i;
		}  }
	psb->m_bUpdateRtCst=true;
}
