#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include <algorithm>


/** Copying softbodies. */
void copy(btSoftBody *orig, btSoftBody *copy_sb) const {
    const btSoftBody * const orig = softBody.get();
    int i, j;

    // create a new softBody with the data
    btSoftBody * const psb = new btSoftBody(orig->m_worldInfo);

    // materials
    psb->m_materials.reserve(orig->m_materials.size());
    for (i=0;i<orig->m_materials.size();i++) {
        const btSoftBody::Material *mat = orig->m_materials[i];
        btSoftBody::Material *newMat = psb->appendMaterial();
        newMat->m_flags = mat->m_flags;
        newMat->m_kAST = mat->m_kAST;
        newMat->m_kLST = mat->m_kLST;
        newMat->m_kVST = mat->m_kVST;
        f.registerCopy(mat, newMat);
    }

    // nodes
    psb->m_nodes.reserve(orig->m_nodes.size());
    for (i=0;i<orig->m_nodes.size();i++) {
        const btSoftBody::Node *node = &orig->m_nodes[i];
        psb->appendNode(node->m_x, node->m_im ? 1./node->m_im : 0.);
        btSoftBody::Node *newNode = &psb->m_nodes[psb->m_nodes.size()-1];
        newNode->m_area = node->m_area;
        newNode->m_battach = node->m_battach;
        newNode->m_f = node->m_f;
        newNode->m_im = node->m_im;
        newNode->m_n = node->m_n;
        newNode->m_q = node->m_q;
        newNode->m_v = node->m_v;

        newNode->m_material = (btSoftBody::Material *) f.copyOf(node->m_material);
        BOOST_ASSERT(newNode->m_material);

    }

    // links
    psb->m_links.reserve(orig->m_links.size());
    for (i=0;i<orig->m_links.size();i++) {
        const btSoftBody::Link *link = &orig->m_links[i];
        btSoftBody::Material *mat = (btSoftBody::Material *) f.copyOf(link->m_material);
        btSoftBody::Node *n0 = (btSoftBody::Node *) f.copyOf(link->m_n[0]);
        btSoftBody::Node *n1 = (btSoftBody::Node *) f.copyOf(link->m_n[1]);
        BOOST_ASSERT(n0 && n1);
        BOOST_ASSERT( ((btSoftBody::Node*)f.copyOf(link->m_n[0]))->m_x == link->m_n[0]->m_x );
        BOOST_ASSERT( ((btSoftBody::Node*)f.copyOf(link->m_n[1]))->m_x == link->m_n[1]->m_x );
        psb->appendLink(n0, n1, mat);

        btSoftBody::Link *newLink = &psb->m_links[psb->m_links.size() - 1];
        newLink->m_bbending = link->m_bbending;
        newLink->m_rl = link->m_rl;
    }

    // faces
    psb->m_faces.reserve(orig->m_faces.size());
    for (i=0;i<orig->m_faces.size();i++) {
        const btSoftBody::Face *face = &orig->m_faces[i];
        btSoftBody::Material *mat = (btSoftBody::Material *) f.copyOf(face->m_material);
        btSoftBody::Node *n0 = (btSoftBody::Node *) f.copyOf(face->m_n[0]);
        btSoftBody::Node *n1 = (btSoftBody::Node *) f.copyOf(face->m_n[1]);
        btSoftBody::Node *n2 = (btSoftBody::Node *) f.copyOf(face->m_n[2]);
        BOOST_ASSERT(n0 && n1 && n2);

        btSoftBody_appendFace(psb, n0, n1, n2, mat);

        btSoftBody::Face *newFace = &psb->m_faces[psb->m_faces.size()-1];
        newFace->m_normal = face->m_normal;
        newFace->m_ra = face->m_ra;
    }

    // pose
    psb->m_pose.m_bvolume = orig->m_pose.m_bvolume;
    psb->m_pose.m_bframe = orig->m_pose.m_bframe;
    psb->m_pose.m_volume = orig->m_pose.m_volume;
    COPY_ARRAY(psb->m_pose.m_pos, orig->m_pose.m_pos);
    COPY_ARRAY(psb->m_pose.m_wgh, orig->m_pose.m_wgh);
    psb->m_pose.m_com = orig->m_pose.m_com;
    psb->m_pose.m_rot = orig->m_pose.m_rot;
    psb->m_pose.m_scl = orig->m_pose.m_scl;
    psb->m_pose.m_aqq = orig->m_pose.m_aqq;

    // config
    psb->m_cfg.aeromodel = orig->m_cfg.aeromodel;
    psb->m_cfg.kVCF = orig->m_cfg.kVCF;
    psb->m_cfg.kDP = orig->m_cfg.kDP;
    psb->m_cfg.kDG = orig->m_cfg.kDG;
    psb->m_cfg.kLF = orig->m_cfg.kLF;
    psb->m_cfg.kPR = orig->m_cfg.kPR;
    psb->m_cfg.kVC = orig->m_cfg.kVC;
    psb->m_cfg.kDF = orig->m_cfg.kDF;
    psb->m_cfg.kMT = orig->m_cfg.kMT;
    psb->m_cfg.kCHR = orig->m_cfg.kCHR;
    psb->m_cfg.kKHR = orig->m_cfg.kKHR;
    psb->m_cfg.kSHR = orig->m_cfg.kSHR;
    psb->m_cfg.kAHR = orig->m_cfg.kAHR;
    psb->m_cfg.kSRHR_CL = orig->m_cfg.kSRHR_CL;
    psb->m_cfg.kSKHR_CL = orig->m_cfg.kSKHR_CL;
    psb->m_cfg.kSSHR_CL = orig->m_cfg.kSSHR_CL;
    psb->m_cfg.kSR_SPLT_CL = orig->m_cfg.kSR_SPLT_CL;
    psb->m_cfg.kSK_SPLT_CL = orig->m_cfg.kSK_SPLT_CL;
    psb->m_cfg.kSS_SPLT_CL = orig->m_cfg.kSS_SPLT_CL;
    psb->m_cfg.maxvolume = orig->m_cfg.maxvolume;
    psb->m_cfg.timescale = orig->m_cfg.timescale;
    psb->m_cfg.viterations = orig->m_cfg.viterations;
    psb->m_cfg.piterations = orig->m_cfg.piterations;
    psb->m_cfg.diterations = orig->m_cfg.diterations;
    psb->m_cfg.citerations = orig->m_cfg.citerations;
    psb->m_cfg.collisions = orig->m_cfg.collisions;
    COPY_ARRAY(psb->m_cfg.m_vsequence, orig->m_cfg.m_vsequence);
    COPY_ARRAY(psb->m_cfg.m_psequence, orig->m_cfg.m_psequence);
    COPY_ARRAY(psb->m_cfg.m_dsequence, orig->m_cfg.m_dsequence);
    psb->getCollisionShape()->setMargin(orig->getCollisionShape()->getMargin());

    // solver state
    psb->m_sst = orig->m_sst;

    // clusters
    psb->m_clusters.resize(orig->m_clusters.size());
    for (i=0;i<orig->m_clusters.size();i++) {
        btSoftBody::Cluster *cl = orig->m_clusters[i];
        btSoftBody::Cluster *newcl = psb->m_clusters[i] =
            new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();

        newcl->m_nodes.resize(cl->m_nodes.size());
        for (j = 0; j < cl->m_nodes.size(); ++j)
            newcl->m_nodes[j] = (btSoftBody::Node *) f.copyOf(cl->m_nodes[j]);
        COPY_ARRAY(newcl->m_masses, cl->m_masses);
        COPY_ARRAY(newcl->m_framerefs, cl->m_framerefs);
        newcl->m_framexform = cl->m_framexform;
        newcl->m_idmass = cl->m_idmass;
        newcl->m_imass = cl->m_imass;
        newcl->m_locii = cl->m_locii;
        newcl->m_invwi = cl->m_invwi;
        newcl->m_com = cl->m_com;
        newcl->m_vimpulses[0] = cl->m_vimpulses[0];
        newcl->m_vimpulses[1] = cl->m_vimpulses[1];
        newcl->m_dimpulses[0] = cl->m_dimpulses[0];
        newcl->m_dimpulses[1] = cl->m_dimpulses[1];
        newcl->m_nvimpulses = cl->m_nvimpulses;
        newcl->m_ndimpulses = cl->m_ndimpulses;
        newcl->m_lv = cl->m_lv;
        newcl->m_av = cl->m_av;
        newcl->m_leaf = 0; // soft body code will set this automatically
        newcl->m_ndamping = cl->m_ndamping;
        newcl->m_ldamping = cl->m_ldamping;
        newcl->m_adamping = cl->m_adamping;
        newcl->m_matching = cl->m_matching;
        newcl->m_maxSelfCollisionImpulse = cl->m_maxSelfCollisionImpulse;
        newcl->m_selfCollisionImpulseFactor = cl->m_selfCollisionImpulseFactor;
        newcl->m_containsAnchor = cl->m_containsAnchor;
        newcl->m_collide = cl->m_collide;
        newcl->m_clusterIndex = cl->m_clusterIndex;
    }

    // cluster connectivity
    COPY_ARRAY(psb->m_clusterConnectivity, orig->m_clusterConnectivity);

    psb->updateConstants();

    Ptr p(new BulletSoftObject(psb));
    p->nextAnchorHandle = nextAnchorHandle;
    p->anchormap = anchormap;

    return p;
}












//##############################################################################
/** Function for cutting a planar soft-body, using an implicit function.
 * Uses btSoftBody::refine.*/
void btSoftBody::cutSoftBody(ImplicitFn* ifn,btScalar accurary) {
  printf("Cut body...\n");

  // First node address
  const Node*			nbase = &m_nodes[0];

  // Number of nodes (initial)
  int					ncount = m_nodes.size();

  // Undirected graph representation: represents links and face-edges b/w nodes
  btSymMatrix<int>	edges(ncount,-2);

  int	newnodes=0;
  int i,j,k,ni;

  /* Filter out		*/
  //Remove links connecting nodes on different sides of implicit function boundary
  //if they are bending links.
  for(i=0;i<m_links.size();++i)
    {
      Link&	l=m_links[i];
      if(l.m_bbending)
	{
	  if(!SameSign(ifn->Eval(l.m_n[0]->m_x),ifn->Eval(l.m_n[1]->m_x)))
	    {
	      btSwap(m_links[i],m_links[m_links.size()-1]);
	      m_links.pop_back();--i;
	    }
	}
    }
  printf("1\n");

  /* Fill edges : Build the edge matrix, by going over links and faces*/
  for(i=0;i<m_links.size();++i) { // LINKS
    Link&	l=m_links[i];
    edges(int(l.m_n[0]-nbase),int(l.m_n[1]-nbase))=-1;
  }
  printf("2\n");


  for(i=0;i<m_faces.size();++i) {// FACE-EDGES
    Face&	f=m_faces[i];
    edges(int(f.m_n[0]-nbase),int(f.m_n[1]-nbase))=-1;
    edges(int(f.m_n[1]-nbase),int(f.m_n[2]-nbase))=-1;
    edges(int(f.m_n[2]-nbase),int(f.m_n[0]-nbase))=-1;
  }
  printf("3\n");


  /* Intersect		*/
  for(i=0;i<ncount;++i) {
    for(j=i+1;j<ncount;++j) {
      // For all pairs of nodes:
      if(edges(i,j)==-1)  // if there is an edge b/w them
	{
	  Node&			a=m_nodes[i];  // a <-- the first node
	  Node&			b=m_nodes[j];  // b <-- the second node
	  const btScalar	t=ImplicitSolve(ifn,a.m_x,b.m_x,accurary);  // t : ta + (1-t)b is on the edge if 0<t, else -1
	  if(t>0)   // if the nodes are on the opposite sides of the boundary
	    {
	      const btVector3	x=Lerp(a.m_x,b.m_x,t); // x <-- position of the node at the boundary: linear interpolation of the positions of a and b
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
	      appendNode(x,m);
	      edges(i,j)=m_nodes.size()-1;  // edges(i,j) <-- index of the interpolated new node ( > 0)
	      m_nodes[edges(i,j)].m_v=v;
	      ++newnodes;
	    }
	}
    }
  }
  printf("3\n");

  nbase=&m_nodes[0];  // <-- address of the first node
  /* Refine links
   *
   *  a------->b |==> a------->t---->b
   *  t : interpolated node
   **/
  for(i=0,ni=m_links.size();i<ni;++i)
    {
      Link&		feat=m_links[i];
      const int	idx[]={	int(feat.m_n[0]-nbase),
			int(feat.m_n[1]-nbase)};
      if((idx[0]<ncount)&&(idx[1]<ncount))
	{
	  const int ni=edges(idx[0],idx[1]);
	  if(ni>0)
	    {
	      appendLink(i);
	      Link*		pft[]={	&m_links[i],
					&m_links[m_links.size()-1]};
	      pft[0]->m_n[0]=&m_nodes[idx[0]];
	      pft[0]->m_n[1]=&m_nodes[ni];
	      pft[1]->m_n[0]=&m_nodes[ni];
	      pft[1]->m_n[1]=&m_nodes[idx[1]];
	    }
	}
    }
  printf("4\n");


  /* Refine faces:
   * Similar to "edge-breaking" for links [above]
   * */
  for(i=0;i<m_faces.size();++i)
    {
      const Face&	feat=m_faces[i];
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
		  appendFace(i);
		  const int	l=(k+1)%3;
		  Face*		pft[]={	&m_faces[i],
					&m_faces[m_faces.size()-1]};
		  pft[0]->m_n[0]=&m_nodes[idx[l]];
		  pft[0]->m_n[1]=&m_nodes[idx[j]];
		  pft[0]->m_n[2]=&m_nodes[ni];
		  pft[1]->m_n[0]=&m_nodes[ni];
		  pft[1]->m_n[1]=&m_nodes[idx[k]];
		  pft[1]->m_n[2]=&m_nodes[idx[l]];
		  appendLink(ni,idx[l],pft[0]->m_material);
		  --i;break;
		}
	    }
	}
    }
  printf("5\n");


  /* Cut	: Deletes the links/ faces/ nodes/ anchors. */
  /* Deletion metric:
   *
   * Define In(node) = true if ImplicitFn's eval(node) <= 0 and false otherwise.
   *
   * Delete link l iff (In(l.m_n[0]) && In(l.m_n[1]))
   * Delete face f iff (In(f.m_n[0]) && In(f.m_n[1]) && In(f.m_n[2])
   * Delete node n iff In(n)
   *
   * */

  /* Delete the faces */
  std::cout<<"Deleting faces.."<<std::cout;
  for(i=0,ni=m_faces.size();i<ni;++i)
    {
      Node**	n =	m_faces[i].m_n;
      if(	(ifn->Eval(n[0]->m_x)< -accurary) ||
		(ifn->Eval(n[1]->m_x)< -accurary) ||
		(ifn->Eval(n[2]->m_x)< -accurary))
	{
	  btSwap(m_faces[i],m_faces[m_faces.size()-1]);
	  m_faces.pop_back();
	  --i;
	}
    }
  printf("6\n");


  /* Delete the links */
  for(i=0,ni=m_links.size(); i < ni;++i)
    {
      Node**	n =	m_links[i].m_n;
      if(	(ifn->Eval(n[0]->m_x)< -accurary) ||
		(ifn->Eval(n[1]->m_x)< -accurary)) {
	btSwap(m_links[i],m_links[m_links.size()-1]);
	m_links.pop_back();
	--i;
      }
    }
  printf("7\n");


  /* Calculate the rank of the nodes: */
  nbase = &m_nodes[0];
  int	nnodes=m_nodes.size();
  btAlignedObjectArray<int>	ranks;
  ranks.resize(nnodes,0);

  for(i=0,ni=m_links.size();i<ni;++i)  // link ranks
    for(int j=0;j<2;++j) ranks[int(m_links[i].m_n[j]-nbase)]++;

  for(i=0,ni=m_faces.size();i<ni;++i) // face-edge ranks
    for(int j=0;j<3;++j) ranks[int(m_faces[i].m_n[j]-nbase)] ++;

  printf("8\n");


  // Delete the links which end on orphan nodes:
  for(i=0;i<m_links.size();++i)  {
    const int	id[]={	int(m_links[i].m_n[0]-nbase),
			int(m_links[i].m_n[1]-nbase)  };
    const bool	sg[]={	ranks[id[0]]==1,
			ranks[id[1]]==1 };
    if(sg[0]||sg[1]) {
      --ranks[id[0]];
      --ranks[id[1]];
      btSwap(m_links[i],m_links[m_links.size()-1]);
      m_links.pop_back();
      --i;
    }
  }
  printf("9\n");


  /* Delete the anchors which attach to the nodes with 0 rank */
  for(i=0; i< m_anchors.size(); ++i) {
    Anchor a = m_anchors[i];
    Node * n = m_anchors[i].m_node;
    // find the node index
    int c;
    for(c=0; c<m_nodes.size(); c++) {
      if (&m_nodes[i] == n) break;
    }
    if (ranks[c] == 0) {
      btSwap(m_anchors[i], m_anchors[m_anchors.size()-1]);
      m_anchors.pop_back();
      --i;
    }
  }
  printf("10\n");

//  std::vector<int> exclusionInds;


#if 0
  for(i=nnodes-1;i>=0;--i){
	  if(!ranks[i]) todelete.push_back(i);
  }
  if(todelete.size()) {
	  btAlignedObjectArray<int>&	map=ranks;
	  for(int i=0;i<nnodes;++i) map[i]=i;
//	  PointersToIndices(this);
	  for(int i=0,ni=todelete.size();i<ni;++i)
	  {
		  int		j=todelete[i];
		  int&	a=map[j];
		  int&	b=map[--nnodes];
		  m_ndbvt.remove(m_nodes[a].m_leaf);m_nodes[a].m_leaf=0;
		  btSwap(m_nodes[a],m_nodes[b]);
		  j=a;a=b;b=j;
	  }
//	  IndicesToPointers(this,&map[0]);
	  m_nodes.resize(nnodes);
  }
#endif

  /* Delete the nodes with ranks <= 0 */
  /*for(i=0; i<m_nodes.size();++i) {
    if (ranks[i] == 0) {
      //m_ndbvt.remove(m_nodes[i].m_leaf);m_nodes[i].m_leaf=0;

      --i;
    }
    }*/
  printf("11\n");
  m_bUpdateRtCst=true;
}

///

struct btSoftBody::cutPlane {
	std::vector<btVector3> 	c_corners;
	btVector3				c_origin,c_xax,c_yax,c_zax;
	btScalar 				c_f1,c_f2,c_length;

	cutPlane (btVector3 p1, btVector3 p2, btVector3 p3, btVector3 p4,
				btScalar f1, btScalar f2)	{
		assert(0<f1 && f1<f2 && f2<1);
		c_f1 = f1;
		c_f2 = f2;
		c_corners.push_back(p1);
		c_corners.push_back(p2);
		c_corners.push_back(p3);
		c_corners.push_back(p4);
		c_xax = (p3-p2).normalized();
		c_yax = (p3-p4).normalized();
		c_zax = c_xax.cross(c_yax).normalized();
		c_origin = 0.5*(p2+p1)+f1*(p4-p1);
		c_length = (f2-f1)*(p4-p1).length();
	}

	bool shouldConsider (const btVector3& p) {
		btScalar x = (p-c_origin).dot(c_xax);
		return 0 <= x && x <= c_length;
	}

	btScalar Eval(btVector3& p) {
		return (p-c_origin).dot(c_yax);
	}
};

static btScalar				ImplicitSolve(	btSoftBody::cutPlane* inf,
										  	btVector3& a,
										  	btVector3& b,
										  const btScalar accuracy,
										  const int maxiterations=256)
{
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

//##############################################################################
/** Function for cutting a planar soft-body, using an implicit function.
 * Uses btSoftBody::refine.*/
void btSoftBody::cutPlaneSoftBody(btSoftBody::cutPlane* info, btScalar accuracy) {
  printf("Cut body...\n");

  // First node address
  const Node*			nbase = &m_nodes[0];

  // Number of nodes (initial)
  int					ncount = m_nodes.size();

  // Undirected graph representation: represents links and face-edges b/w nodes
  btSymMatrix<int>	edges(ncount,-2);

  int	newnodes=0;
  int i,j,k,ni;

  /* Filter out		*/
  //Remove links connecting nodes on different sides of implicit function boundary
  //if they are bending links.
  for(i=0;i<m_links.size();++i)
    {
      Link&	l=m_links[i];
      if(l.m_bbending)
      {
    	  btScalar t = ImplicitSolve(info, l.m_n[0]->m_x, l.m_n[1]->m_x,accuracy);
    	  if (t<0) continue;
    	  const btVector3 x = Lerp(l.m_n[0]->m_x, l.m_n[1]->m_x,t);
    	  if (info->shouldConsider(x)){
    		  if(!SameSign(info->Eval(l.m_n[0]->m_x),info->Eval(l.m_n[1]->m_x)))
    		  {
    			  btSwap(m_links[i],m_links[m_links.size()-1]);
    			  m_links.pop_back();--i;
    		  }
    	  }

	}
    }
  printf("1\n");

  /* Fill edges : Build the edge matrix, by going over links and faces*/
  for(i=0;i<m_links.size();++i) { // LINKS
    Link&	l=m_links[i];
    edges(int(l.m_n[0]-nbase),int(l.m_n[1]-nbase))=-1;
  }
  printf("2\n");


  for(i=0;i<m_faces.size();++i) {// FACE-EDGES
    Face&	f=m_faces[i];
    edges(int(f.m_n[0]-nbase),int(f.m_n[1]-nbase))=-1;
    edges(int(f.m_n[1]-nbase),int(f.m_n[2]-nbase))=-1;
    edges(int(f.m_n[2]-nbase),int(f.m_n[0]-nbase))=-1;
  }
  printf("3\n");


  /* Intersect		*/
  for(i=0;i<ncount;++i) {
    for(j=i+1;j<ncount;++j) {
      // For all pairs of nodes:
      if(edges(i,j)==-1)  // if there is an edge b/w them
	{
	  Node&			a=m_nodes[i];  // a <-- the first node
	  Node&			b=m_nodes[j];  // b <-- the second node
	  const btScalar	t=ImplicitSolve(info,a.m_x,b.m_x,accuracy);  // t : ta + (1-t)b is on the edge if 0<t, else -1
	  if(t>0)   // if the nodes are on the opposite sides of the boundary
	    {
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
	      appendNode(x,m);
	      edges(i,j)=m_nodes.size()-1;  // edges(i,j) <-- index of the interpolated new node ( > 0)
	      m_nodes[edges(i,j)].m_v=v;
	      ++newnodes;
	    }
	}
    }
  }
  printf("3\n");

  nbase=&m_nodes[0];  // <-- address of the first node
  /* Refine links
   *
   *  a------->b |==> a------->t---->b
   *  t : interpolated node
   **/
  for(i=0,ni=m_links.size();i<ni;++i)
    {
      Link&		feat=m_links[i];
      const int	idx[]={	int(feat.m_n[0]-nbase),
			int(feat.m_n[1]-nbase)};
      if((idx[0]<ncount)&&(idx[1]<ncount))
	{
	  const int ni=edges(idx[0],idx[1]);
	  if(ni>0)
	    {
	      appendLink(i);
	      Link*		pft[]={	&m_links[i],
					&m_links[m_links.size()-1]};
	      pft[0]->m_n[0]=&m_nodes[idx[0]];
	      pft[0]->m_n[1]=&m_nodes[ni];
	      pft[1]->m_n[0]=&m_nodes[ni];
	      pft[1]->m_n[1]=&m_nodes[idx[1]];
	    }
	}
    }
  printf("4\n");


  /* Refine faces:
   * Similar to "edge-breaking" for links [above]
   * */
  for(i=0;i<m_faces.size();++i)
    {
      const Face&	feat=m_faces[i];
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
		  appendFace(i);
		  const int	l=(k+1)%3;
		  Face*		pft[]={	&m_faces[i],
					&m_faces[m_faces.size()-1]};
		  pft[0]->m_n[0]=&m_nodes[idx[l]];
		  pft[0]->m_n[1]=&m_nodes[idx[j]];
		  pft[0]->m_n[2]=&m_nodes[ni];
		  pft[1]->m_n[0]=&m_nodes[ni];
		  pft[1]->m_n[1]=&m_nodes[idx[k]];
		  pft[1]->m_n[2]=&m_nodes[idx[l]];
		  appendLink(ni,idx[l],pft[0]->m_material);
		  --i;break;
		}
	    }
	}
    }
  printf("5\n");


  /* Cut	: Deletes the links/ faces/ nodes/ anchors. */
  btAlignedObjectArray<int>	cnodes;
  const int			 pcount=ncount;
  int				             i;
  ncount=m_nodes.size();
  cnodes.resize(ncount,0);

  nbase=&m_nodes[0];
  /* Links		*/
  for(i=0,ni=m_links.size();i<ni;++i)  {
	  const int		id[]={	int(m_links[i].m_n[0]-nbase),
			  	  	  	  	int(m_links[i].m_n[1]-nbase) };
	  btVector3 n1 = m_nodes[id[0]].m_x;
	  btVector3 n2 = m_nodes[id[1]].m_x;

	  btScalar eval1 = info->Eval(n1);
	  btScalar eval2 = info->Eval(n2);
	  bool   should1 = info->shouldConsider(n1);
	  bool   should2 = info->shouldConsider(n2);

	  if (btFabs(eval1) < accuracy
		  && btFabs (eval2) < accuracy
		  && eval1*eval2 < 0
		  && should1 && should2) { // throw out the link.
		  btSwap(m_links[i],m_links[m_links.size()-1]);
		  m_links.pop_back();--i;
	  } else {
		if (btFabs(eval1) < accuracy
			&& should1) {
			if (eval2 > accuracy) { // (2) in far +ve. (1) is within accuracy and is shouldConsider
				if(!cnodes[id[0]]) {// create a copy of the node
					const btVector3	v=m_nodes[id[0]].m_v;
					btScalar		m=getMass(id[0]);
					if(m>0) { m*=0.5;m_nodes[id[0]].m_im/=0.5; }
					appendNode(n1,m);
					cnodes[id[0]]=m_nodes.size()-1;
					m_nodes[cnodes[id[0]]].m_v=v;
				}
				m_links[i].m_n[0] = m_nodes[cnodes[id[0]]];
 			}
		} else if (btFabs(eval2) < accuracy
				&& should2) {
			if (eval1 > accuracy) { // (1) in far +ve. (2) is within accuracy and is shouldConsider
				if(!cnodes[id[1]]) {// create a copy of the node
					const btVector3	v=m_nodes[id[1]].m_v;
					btScalar		m=getMass(id[1]);
					if(m>0) { m*=0.5;m_nodes[id[1]].m_im/=0.5; }
					appendNode(n2,m);
					cnodes[id[1]]=m_nodes.size()-1;
					m_nodes[cnodes[id[1]]].m_v=v;
				}
				m_links[i].m_n[1] = m_nodes[cnodes[id[1]]];

			}
		}
	  }
  }

  /* Faces : split faces 	*/
  for(i=0,ni=m_faces.size();i<ni;++i) {
	  Node**	n  = m_faces[i].m_n;
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
				  const btVector3	v=m_nodes[id[0]].m_v;
				  btScalar		m=getMass(id[0]);
				  if(m>0) { m*=0.5;m_nodes[id[0]].m_im/=0.5; }
				  appendNode(n0,m);
				  cnodes[id[0]]=m_nodes.size()-1;
				  m_nodes[cnodes[id[0]]].m_v=v;
			  }
			  n[0] = &m_nodes[cnodes[id[0]]];
			  iden += 1;
		  }

		  // for node 1:
		  if (btFabs(eval1) < accuracy && should1 ) {
			  if(!cnodes[id[1]]) {// create a copy of the node
				  const btVector3	v=m_nodes[id[1]].m_v;
				  btScalar		m=getMass(id[1]);
				  if(m>0) { m*=0.5;m_nodes[id[1]].m_im/=0.5; }
				  appendNode(n1,m);
				  cnodes[id[1]]=m_nodes.size()-1;
				  m_nodes[cnodes[id[1]]].m_v=v;
			  }
			  n[1] = &m_nodes[cnodes[id[1]]];
			  iden += 10;
		  }

		  // for node 2:
		  if (btFabs(eval2) < accuracy && should2 ) {
			  if(!cnodes[id[2]]) {// create a copy of the node
				  const btVector3	v=m_nodes[id[2]].m_v;
				  btScalar		m=getMass(id[2]);
				  if(m>0) { m*=0.5;m_nodes[id[2]].m_im/=0.5; }
				  appendNode(n2,m);
				  cnodes[id[2]]=m_nodes.size()-1;
				  m_nodes[cnodes[id[2]]].m_v=v;
			  }
			  n[2] = &m_nodes[cnodes[id[2]]];
			  iden += 100;
		  }

		  switch(iden) {
		  case 11:
			  appendLink(cnodes[id[0]], cnodes[id[1]], m_faces[i].m_material, true);
			  break;
		  case 101:
			  appendLink(cnodes[id[0]], cnodes[id[2]], m_faces[i].m_material, true);
			  break;
		  case 110:
			  appendLink(cnodes[id[1]], cnodes[id[2]], m_faces[i].m_material, true);
			  break;
		  }
	  }

	  /* Clean orphans	*/
  int							nnodes=m_nodes.size();
  btAlignedObjectArray<int>	ranks;
  btAlignedObjectArray<int>	todelete;
  ranks.resize(nnodes,0);
  for(i=0,ni=m_links.size();i<ni;++i)
  {
	  for(int j=0;j<2;++j) ranks[int(m_links[i].m_n[j]-nbase)]++;
  }
  for(i=0,ni=m_faces.size();i<ni;++i)
  {
	  for(int j=0;j<3;++j) ranks[int(m_faces[i].m_n[j]-nbase)]++;
  }
  for(i=0;i<m_links.size();++i)
  {
	  const int	id[]={	int(m_links[i].m_n[0]-nbase),
			  int(m_links[i].m_n[1]-nbase)};
	  const bool	sg[]={	ranks[id[0]]==1,
			  ranks[id[1]]==1};
	  if(sg[0]||sg[1])
	  {
		  --ranks[id[0]];
		  --ranks[id[1]];
		  btSwap(m_links[i],m_links[m_links.size()-1]);
		  m_links.pop_back();--i;
	  }
  }
  printf("11\n");
  m_bUpdateRtCst=true;
}




























//// Important function to create new psb

btSoftBody*	CreateFromSoftBodyExcludeNodes(btSoftBody* softBody, std::vector<int> exclude_nodes_idx)
{
	btSoftBody::tNodeArray& nodes = softBody->m_nodes;
	btSoftBody::tFaceArray& faces = softBody->m_faces;
	btSoftBody::tLinkArray& links = softBody->m_links;

	// compute nodes to exclude (sorted)
	std::sort(exclude_nodes_idx.begin(), exclude_nodes_idx.end());

	// compute face to nodes indices
	std::vector<std::vector<int> > face2nodes(faces.size(), std::vector<int>(3,-1));
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
	std::vector<std::vector<int> > link2nodes(links.size(), std::vector<int>(2,-1));
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
	std::vector<int> oldNodes2newNodes(nodes.size());
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
//##############################################################################



////////////////////////cutsoftbody2////////////////////////////
btSoftBody* btSoftBody::cutSoftBody2(ImplicitFn* ifn,btScalar accuracy) {

		// Determine the nodes that are inside the polygon contour
		std::vector<int> exclude_nodes_idx;
		for (int j=0, ji = m_nodes.size(); j<ji; j++)
			if (ifn->Eval(m_nodes[j].m_x) < -accuracy) exclude_nodes_idx.push_back(j);

		// Create a new btSoftBody containing only the nodes, faces and links inside the polygon contour
		btSoftBody* psb = CreateFromSoftBodyExcludeNodes(this, exclude_nodes_idx);

		return psb;

}
////////////////////////////////////////////////////////////////





















// I've only tested this on the PR2 model
class PR2SoftBodyGripperAction : public Action {
	RaveRobotObject::Manipulator::Ptr manip;
	dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.03f, OPEN_VAL = 0.54f;

    KinBody::LinkPtr leftFinger, rightFinger;
    const btTransform origLeftFingerInvTrans, origRightFingerInvTrans;

    // the point right where the fingers meet when the gripper is closed
    // (in the robot's initial pose)
    const btVector3 centerPt;

    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    const btVector3 closingNormal;

    // points straight down in the PR2 initial position (manipulator frame)
    const btVector3 toolDirection;

    // the target softbody
    btSoftBody *psb;
    BulletSoftObject::Ptr softBody;

    // appended anchors
    vector<BulletSoftObject::AnchorHandle> anchors;

    btTransform getManipRot() const {
        btTransform trans(manip->getTransform());
        trans.setOrigin(btVector3(0, 0, 0));
        return trans;
    }

    // Returns the direction that the specified finger will move when closing
    // (manipulator frame)
    btVector3 getClosingDirection(bool left) const {
        return (left ? 1 : -1) * toolDirection.cross(closingNormal);
    }

    // Finds some innermost point on the gripper
    btVector3 getInnerPt(bool left) const {
        btTransform trans(manip->robot->getLinkTransform(left ? leftFinger : rightFinger));
        // this assumes that the gripper is symmetric when it is closed
        // we get an innermost point on the gripper by transforming a point
        // on the center of the gripper when it is closed
        const btTransform &origInv = left ? origLeftFingerInvTrans : origRightFingerInvTrans;
        return trans * origInv * centerPt;
        // actually above, we can just cache origInv * centerPt
    }

    // Returns true is pt is on the inner side of the specified finger of the gripper
    bool onInnerSide(const btVector3 &pt, bool left) const {
        // then the innerPt and the closing direction define the plane
        return (getManipRot() * getClosingDirection(left)).dot(pt - getInnerPt(left)) > 0;
    }

    // Fills in the rcontacs array with contact information between psb and pco
    static void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts) {
        // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
        struct Custom_CollideSDF_RS : btDbvt::ICollide {
            Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_) : rcontacts(rcontacts_) { }

            void Process(const btDbvtNode* leaf) {
                btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
                DoNode(*node);
            }

            void DoNode(btSoftBody::Node& n) {
                const btScalar m=n.m_im>0?dynmargin:stamargin;
                btSoftBody::RContact c;
                if (!n.m_battach && psb->checkContact(m_colObj1,n.m_x,m,c.m_cti)) {
                    const btScalar  ima=n.m_im;
                    const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                    const btScalar  ms=ima+imb;
                    if(ms>0) {
                        // there's a lot of extra information we don't need to compute
                        // since we just want to find the contact points
#if 0
                        const btTransform&      wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1->getWorldTransform();
                        static const btMatrix3x3        iwiStatic(0,0,0,0,0,0,0,0,0);
                        const btMatrix3x3&      iwi=m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
                        const btVector3         ra=n.m_x-wtr.getOrigin();
                        const btVector3         va=m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra)*psb->m_sst.sdt : btVector3(0,0,0);
                        const btVector3         vb=n.m_x-n.m_q; 
                        const btVector3         vr=vb-va;
                        const btScalar          dn=btDot(vr,c.m_cti.m_normal);
                        const btVector3         fv=vr-c.m_cti.m_normal*dn;
                        const btScalar          fc=psb->m_cfg.kDF*m_colObj1->getFriction();
#endif
                        c.m_node        =       &n;
#if 0
                        c.m_c0          =       ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
                        c.m_c1          =       ra;
                        c.m_c2          =       ima*psb->m_sst.sdt;
                        c.m_c3          =       fv.length2()<(btFabs(dn)*fc)?0:1-fc;
                        c.m_c4          =       m_colObj1->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
#endif
                        rcontacts.push_back(c);
#if 0
                        if (m_rigidBody)
                                m_rigidBody->activate();
#endif
                    }
                }
            }
            btSoftBody*             psb;
            btCollisionObject*      m_colObj1;
            btRigidBody*    m_rigidBody;
            btScalar                dynmargin;
            btScalar                stamargin;
            btSoftBody::tRContactArray &rcontacts;
        };

        Custom_CollideSDF_RS  docollide(rcontacts);              
        btRigidBody*            prb1=btRigidBody::upcast(pco);
        btTransform     wtr=pco->getWorldTransform();

        const btTransform       ctr=pco->getWorldTransform();
        const btScalar          timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
        const btScalar          basemargin=psb->getCollisionShape()->getMargin();
        btVector3                       mins;
        btVector3                       maxs;
        ATTRIBUTE_ALIGNED16(btDbvtVolume)               volume;
        pco->getCollisionShape()->getAabb(      pco->getWorldTransform(),
                mins,
                maxs);
        volume=btDbvtVolume::FromMM(mins,maxs);
        volume.Expand(btVector3(basemargin,basemargin,basemargin));             
        docollide.psb           =       psb;
        docollide.m_colObj1 = pco;
        docollide.m_rigidBody = prb1;

        docollide.dynmargin     =       basemargin+timemargin;
        docollide.stamargin     =       basemargin;
        psb->m_ndbvt.collideTV(psb->m_ndbvt.m_root,volume,docollide);
    }

    // Checks if psb is touching the inside of the gripper fingers
    // If so, attaches anchors to every contact point
    void attach(bool left) {
        btRigidBody *rigidBody =
            manip->robot->associatedObj(left ? leftFinger : rightFinger)->rigidBody.get();
        btSoftBody::tRContactArray rcontacts;
        getContactPointsWith(psb, rigidBody, rcontacts);
        cout << "got " << rcontacts.size() << " contacts\n";
        for (int i = 0; i < rcontacts.size(); ++i) {
            const btSoftBody::RContact &c = rcontacts[i];
            KinBody::LinkPtr colLink = manip->robot->associatedObj(c.m_cti.m_colObj);
            if (!colLink) continue;
            const btVector3 &contactPt = c.m_node->m_x;
            if (onInnerSide(contactPt, left)) {
                cout << "\tappending anchor\n";
                anchors.push_back(softBody->addAnchor(c.m_node, rigidBody));
            }
        }
    }

public:
    typedef boost::shared_ptr<PR2SoftBodyGripperAction> Ptr;
    PR2SoftBodyGripperAction(RaveRobotObject::Manipulator::Ptr manip_,
                  const string &leftFingerName,
                  const string &rightFingerName,
                  float time) :
            Action(time), manip(manip_), vals(1, 0),
            leftFinger(manip->robot->robot->GetLink(leftFingerName)),
            rightFinger(manip->robot->robot->GetLink(rightFingerName)),
            origLeftFingerInvTrans(manip->robot->getLinkTransform(leftFinger).inverse()),
            origRightFingerInvTrans(manip->robot->getLinkTransform(rightFinger).inverse()),
            centerPt(manip->getTransform().getOrigin()),
            indices(manip->manip->GetGripperIndices()),
            closingNormal(manip->manip->GetClosingDirection()[0],
                          manip->manip->GetClosingDirection()[1],
                          manip->manip->GetClosingDirection()[2]),
            toolDirection(util::toBtVector(manip->manip->GetLocalToolDirection())) // don't bother scaling
    {
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
        setCloseAction();
    }

    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    dReal getCurrDOFVal() const {
        vector<dReal> v;
        manip->robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), OPEN_VAL); } // 0.54 is the max joint value for the pr2
    void setCloseAction() { setEndpoints(getCurrDOFVal(), CLOSED_VAL); }
    void toggleAction() {
        if (endVal == CLOSED_VAL)
            setOpenAction();
        else if (endVal == OPEN_VAL)
            setCloseAction();
    }

    // Must be called before the action is run!
    void setTarget(BulletSoftObject::Ptr sb) {
        softBody = sb;
        psb = sb->softBody.get();
    }

    void releaseAllAnchors() {
        for (int i = 0; i < anchors.size(); ++i)
            softBody->removeAnchor(anchors[i]);
        anchors.clear();
    }

    void reset() {
        Action::reset();
        releaseAllAnchors();
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        manip->robot->setDOFValues(indices, vals);

        if (vals[0] == CLOSED_VAL) {
            attach(true);
            attach(false);
        }
    }
};

struct CustomScene : public Scene {
    PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
    BulletInstance::Ptr bullet2;
    OSGInstance::Ptr osg2;
    Fork::Ptr fork;
    RaveRobotObject::Ptr origRobot, tmpRobot;
    PR2Manager pr2m;

    CustomScene() : pr2m(*this) { }

    BulletSoftObject::Ptr createCloth(btScalar s,
				      const btVector3 &center);
    void createFork();
    void swapFork();
    void run();
};

class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,osgGA::GUIActionAdapter &) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'a':
            scene.leftAction->reset();
            scene.leftAction->toggleAction();
            scene.runAction(scene.leftAction, BulletConfig::dt);
            break;
        case 's':
            scene.rightAction->reset();
            scene.rightAction->toggleAction();
            scene.runAction(scene.rightAction, BulletConfig::dt);
            break;
        case 'f':
            scene.createFork();
            break;
        case 'g':
            scene.swapFork();
            break;
        }
        break;
    }
    return false;
}

BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, const btVector3 &center) {
    const int divs = 45;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        *env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.1;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

/*    for (int i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }*/

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}


void CustomScene::createFork() {
    bullet2.reset(new BulletInstance);
    bullet2->setDefaultGravity();
    osg2.reset(new OSGInstance);
    osg->root->addChild(osg2->root.get());

    fork.reset(new Fork(env, bullet2, osg2));
    registerFork(fork);

    cout << "forked!" << endl;

    origRobot = pr2m.pr2;
    EnvironmentObject::Ptr p = fork->forkOf(pr2m.pr2);
    if (!p) {
        cout << "failed to get forked version of robot!" << endl;
        return;
    }
    tmpRobot = boost::static_pointer_cast<RaveRobotObject>(p);
    cout << (tmpRobot->getEnvironment() == env.get()) << endl;
    cout << (tmpRobot->getEnvironment() == fork->env.get()) << endl;
}

void CustomScene::swapFork() {
    // swaps the forked robot with the real one
    cout << "swapping!" << endl;
    int leftidx = pr2m.pr2Left->index;
    int rightidx = pr2m.pr2Right->index;
    origRobot.swap(tmpRobot);
    pr2m.pr2 = origRobot;
    pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
    pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);

/*    vector<int> indices; vector<dReal> vals;
    for (int i = 0; i < tmpRobot->robot->GetDOF(); ++i) {
        indices.push_back(i);
        vals.push_back(0);
    }
    tmpRobot->setDOFValues(indices, vals);*/
}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = BulletConfig::dt;
    const float table_height = .5;
    const float table_thickness = .05;
    BoxObject::Ptr table(
        new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.2, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(10);

    BulletSoftObject::Ptr cloth(
            createCloth(GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
    btSoftBody * const psb = cloth->softBody.get();
    pr2m.pr2->ignoreCollisionWith(psb);

    env->add(table);
    env->add(cloth);

    leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left, "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(cloth);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right, "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 1));
    rightAction->setTarget(cloth);

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    /*
    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);
    */

    startFixedTimestepLoop(dt);
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);


    CustomScene().run();
    return 0;
}
