#include "make_bodies.h"

#include "simulation/bullet_io.h"
#include "utils/config.h"
#include "utils/logging.h"

#include <boost/shared_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <BulletSoftBody/btSoftBodyHelpers.h>

using boost::shared_ptr;

BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness) {
  btVector3 origin = (corners[0] + corners[2])/2;
  origin[2] -= thickness/2;
  btVector3 halfExtents = (corners[2] - corners[0]).absolute()/2;
  halfExtents[2] = thickness/2;

  cout << "origin: " << origin << endl;
  cout << "halfExtents: " << halfExtents << endl;

  return BulletObject::Ptr(new BoxObject(0,halfExtents,btTransform(btQuaternion(0,0,0,1),origin)));
}

BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, btSoftBodyWorldInfo& worldInfo) {
  btVector3 offset(0,0,.01*METERS);
  btSoftBody* psb=btSoftBodyHelpers::CreatePatch(worldInfo,
						 points[0]+offset,
						 points[1]+offset,
						 points[3]+offset,
						 points[2]+offset,
						 45, 31,
						 0/*1+2+4+8*/, true);
  cout << "points[0] " << points[0].x() << " " << points[0].y() << " " << points[0].z() << endl;
  psb->getCollisionShape()->setMargin(.01*METERS);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST		=	5*0.1;
  pm->m_kAST = 5*0.1;
  //	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(1);

  return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

btSoftBody *makeSelfCollidingTowel(const btVector3& center, btScalar lenx, btScalar leny, int resx, int resy, btSoftBodyWorldInfo& worldInfo) {
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        worldInfo,
        center + btVector3(-lenx/2,-leny/2,0),
        center + btVector3(lenx/2,-leny/2,0),
        center + btVector3(-lenx/2,leny/2,0),
        center + btVector3(lenx/2,leny/2,0),
        resx, resy,
        0, true);

    psb->m_cfg.piterations = 8;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 0.9;
    //psb->m_cfg.kDF = 0.1;
    psb->m_cfg.kAHR = 1; // anchor hardness
    psb->m_cfg.kSSHR_CL = 1.0; // so the cloth doesn't penetrate itself
    psb->m_cfg.kSRHR_CL = 0.7;
    psb->m_cfg.kSKHR_CL = 0.7;
//    psb->m_cfg.kDP = 0.01;
    psb->m_cfg.kDP = 0.1;

    psb->getCollisionShape()->setMargin(0.05);

    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.001;
    pm->m_kAST = 0.5;

    psb->generateBendingConstraints(2, pm);

    // weaken links that span 2 nodes
    /*
    btScalar avgDist = 0;
    for (int i = 0; i < psb->m_links.size(); ++i) {
        btSoftBody::Link &l = psb->m_links[i];
        avgDist += l.m_n[0]->m_x.distance(l.m_n[1]->m_x);
    }
    avgDist /= psb->m_links.size();
    for (int i = 0; i < psb->m_links.size(); ++i) {
        btSoftBody::Link &l = psb->m_links[i];
        btScalar d = l.m_n[0]->m_x.distance(l.m_n[1]->m_x);
        if (d > avgDist)
            l.m_material->m_kLST = 0.005;
    }*/

    psb->randomizeConstraints();

    psb->setTotalMass(100, true);
    psb->generateClusters(0);

    /*for (int i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_ndamping = 1;
    }*/

    LOG_TRACE("number of clusters: " << psb->m_clusters.size());

    return psb;
}

// adapted from btSoftBodyHelpers::CreatePatch
btSoftBody* createBox(btSoftBodyWorldInfo& worldInfo,
                      const btVector3& corner000,
                      const btVector3& corner100,
                      const btVector3& corner010,
                      const btVector3& corner110,
                      const btVector3& corner001,
                      const btVector3& corner101,
                      const btVector3& corner011,
                      const btVector3& corner111,
                      int resx,
                      int resy,
                      int resz,
                      bool gendiags) {
#define IDX(_x_,_y_,_z_) ((_z_)*ry*rx + (_y_)*rx + (_x_))
	/* Create nodes	*/
	if((resx<2)||(resy<2)||(resz<2)) return(0);
	const int rx=resx;
	const int ry=resy;
    const int rz=resz;
	const int tot=rx*ry*rz;
	btVector3 *x=new btVector3[tot];
	btScalar *m=new btScalar[tot];
	int iy, iz;

    for (iz=0;iz<rz;++iz)
    {
        const btScalar	tz=iz/(btScalar)(rz-1);
        const btVector3 corner00=lerp(corner000, corner001, tz);
        const btVector3 corner10=lerp(corner100, corner101, tz);
        const btVector3 corner01=lerp(corner010, corner011, tz);
        const btVector3 corner11=lerp(corner110, corner111, tz);
        for(iy=0;iy<ry;++iy)
        {
            const btScalar	ty=iy/(btScalar)(ry-1);
            const btVector3	py0=lerp(corner00,corner01,ty);
            const btVector3	py1=lerp(corner10,corner11,ty);
            for(int ix=0;ix<rx;++ix)
            {
                const btScalar	tx=ix/(btScalar)(rx-1);
                x[IDX(ix,iy,iz)]=lerp(py0,py1,tx);
                m[IDX(ix,iy,iz)]=1;
            }
        }
    }
	btSoftBody*		psb=new btSoftBody(&worldInfo,tot,x,m);
	delete[] x;
	delete[] m;
	/* Create links	and faces */
    for(iz=0;iz<rz;++iz)
    {
        for(iy=0;iy<ry;++iy)
        {
            for(int ix=0;ix<rx;++ix)
            {
                const int	idx=IDX(ix,iy,iz);
                const bool	mdx=(ix+1)<rx;
                const bool	mdy=(iy+1)<ry;
                const bool	mdz=(iz+1)<rz;
                if(mdx) psb->appendLink(idx,IDX(ix+1,iy,iz));
                if(mdy) psb->appendLink(idx,IDX(ix,iy+1,iz));
                if(mdz) psb->appendLink(idx,IDX(ix,iy,iz+1));

                if(mdx&&mdy)
                {
                    if((ix+iy)&1)
                    {
                        psb->appendFace(IDX(ix,iy,iz),IDX(ix+1,iy,iz),IDX(ix+1,iy+1,iz));
                        psb->appendFace(IDX(ix,iy,iz),IDX(ix+1,iy+1,iz),IDX(ix,iy+1,iz));
                        if(gendiags)
                        {
                            psb->appendLink(IDX(ix,iy,iz),IDX(ix+1,iy+1,iz));
                        }
                    }
                    else
                    {
                        psb->appendFace(IDX(ix,iy+1,iz),IDX(ix,iy,iz),IDX(ix+1,iy,iz));
                        psb->appendFace(IDX(ix,iy+1,iz),IDX(ix+1,iy,iz),IDX(ix+1,iy+1,iz));
                        if(gendiags)
                        {
                            psb->appendLink(IDX(ix+1,iy,iz),IDX(ix,iy+1,iz));
                        }
                    }
                }
                if(mdy&&mdz)
                {
                    if((iy+iz)&1)
                    {
                        psb->appendFace(IDX(ix,iy,iz),IDX(ix,iy+1,iz),IDX(ix,iy+1,iz+1));
                        psb->appendFace(IDX(ix,iy,iz),IDX(ix,iy+1,iz+1),IDX(ix,iy,iz+1));
                        if(gendiags)
                        {
                            psb->appendLink(IDX(ix,iy,iz),IDX(ix,iy+1,iz+1));
                        }
                    }
                    else
                    {
                        psb->appendFace(IDX(ix,iy,iz+1),IDX(ix,iy,iz),IDX(ix,iy+1,iz));
                        psb->appendFace(IDX(ix,iy,iz+1),IDX(ix,iy+1,iz),IDX(ix,iy+1,iz+1));
                        if(gendiags)
                        {
                            psb->appendLink(IDX(ix,iy+1,iz),IDX(ix,iy,iz+1));
                        }
                    }
                }
                if(mdx&&mdz)
                {
                    if((ix+iz)&1)
                    {
                        psb->appendFace(IDX(ix,iy,iz),IDX(ix+1,iy,iz),IDX(ix+1,iy,iz+1));
                        psb->appendFace(IDX(ix,iy,iz),IDX(ix+1,iy,iz+1),IDX(ix,iy,iz+1));
                        if(gendiags)
                        {
                            psb->appendLink(IDX(ix,iy,iz),IDX(ix+1,iy,iz+1));
                        }
                    }
                    else
                    {
                        psb->appendFace(IDX(ix,iy,iz+1),IDX(ix,iy,iz),IDX(ix+1,iy,iz));
                        psb->appendFace(IDX(ix,iy,iz+1),IDX(ix+1,iy,iz),IDX(ix+1,iy,iz+1));
                        if(gendiags)
                        {
                            psb->appendLink(IDX(ix+1,iy,iz),IDX(ix,iy,iz+1));
                        }
                    }
                }
            }
        }
    }
	/* Finished		*/
#undef IDX
	return(psb);
}

BulletSoftObject::Ptr makeBoxFromGrid(const vector<btVector3>& points, const btVector3 &thickness, int resx, int resy, int resz, btSoftBodyWorldInfo& worldInfo) {
  btSoftBody *psb = createBox(worldInfo,
      points[0], points[1], points[3], points[2],
      points[0] + thickness, points[1] + thickness, points[3] + thickness, points[2] + thickness,
      resx, resy, resz, true);

  psb->setTotalMass(100);
  psb->m_cfg.piterations=2;
  psb->generateClusters(0);
  psb->getCollisionShape()->setMargin(0.01);
  psb->m_cfg.collisions	= btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;// + btSoftBody::fCollision::CL_SELF;
//  psb->m_cfg.collisions	= btSoftBody::fCollision::SDF_RS + btSoftBody::fCollision::VF_SS;
  psb->m_materials[0]->m_kLST		=	0.08;
  psb->m_materials[0]->m_kAST		=	0.1;
  psb->m_materials[0]->m_kVST		=	0.1;
  //psb->randomizeConstraints();
  return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}
