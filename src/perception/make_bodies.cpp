#include "make_bodies.h"
#include "bullet_io.h"
#include "utils/config.h"
#include <boost/shared_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <tetgen.h>

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

BulletSoftObject::Ptr makeSelfCollidingTowel(const vector<btVector3>& points, btSoftBodyWorldInfo& worldInfo) {
  btVector3 offset(0,0,.01*METERS);
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        worldInfo,
        points[0]+offset,
        points[1]+offset,
        points[3]+offset,
        points[2]+offset,
        45, 31,
        0, true);

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.1;
    pm->m_kAST = 0.1;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}



static char *readWholeFile(const char *filename) {
  cout << "reading " << filename << endl;
  FILE *f = fopen(filename, "r");
  if (!f) return NULL;

  fseek(f, 0, SEEK_END);
  size_t size = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *buf = new char[size];
  size_t s = fread(buf, 1, size, f);
  fclose(f);

  if (s != size) {
    cout << "ERROR: read " << s << " bytes instead of " << size << endl;
    delete[] buf;
    return NULL;
  }
  return buf;
}

#define FILE_PREFIX "tetra"
#define NODE_FILE (FILE_PREFIX ".node")
#define ELE_FILE (FILE_PREFIX ".ele")
#define FACE_FILE (FILE_PREFIX ".face")

// adapted from http://wias-berlin.de/software/tetgen/files/tetcall.cxx
static void runTetgen(const btVector3 &dims) {
  // TODO: deallocate memory!
  tetgenio in, out;
  tetgenio::facet *f;
  tetgenio::polygon *p;
  int i;

  // All indices start from 1.
  in.firstnumber = 1;

  in.numberofpoints = 8;
  in.pointlist = new REAL[in.numberofpoints * 3];
  in.pointlist[0]  = 0;  // node 1.
  in.pointlist[1]  = 0;
  in.pointlist[2]  = 0;
  in.pointlist[3]  = dims.x();  // node 2.
  in.pointlist[4]  = 0;
  in.pointlist[5]  = 0;
  in.pointlist[6]  = dims.x();  // node 3.
  in.pointlist[7]  = dims.y();
  in.pointlist[8]  = 0;
  in.pointlist[9]  = 0;  // node 4.
  in.pointlist[10] = dims.y();
  in.pointlist[11] = 0;
  // Set node 5, 6, 7, 8.
  for (i = 4; i < 8; i++) {
    in.pointlist[i * 3]     = in.pointlist[(i - 4) * 3];
    in.pointlist[i * 3 + 1] = in.pointlist[(i - 4) * 3 + 1];
    in.pointlist[i * 3 + 2] = dims.z();
  }

  in.numberoffacets = 6;
  in.facetlist = new tetgenio::facet[in.numberoffacets];
  in.facetmarkerlist = new int[in.numberoffacets];

  // Facet 1. The leftmost facet.
  f = &in.facetlist[0];
  f->numberofpolygons = 1;
  f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
  f->numberofholes = 0;
  f->holelist = NULL;
  p = &f->polygonlist[0];
  p->numberofvertices = 4;
  p->vertexlist = new int[p->numberofvertices];
  p->vertexlist[0] = 1;
  p->vertexlist[1] = 2;
  p->vertexlist[2] = 3;
  p->vertexlist[3] = 4;
  
  // Facet 2. The rightmost facet.
  f = &in.facetlist[1];
  f->numberofpolygons = 1;
  f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
  f->numberofholes = 0;
  f->holelist = NULL;
  p = &f->polygonlist[0];
  p->numberofvertices = 4;
  p->vertexlist = new int[p->numberofvertices];
  p->vertexlist[0] = 5;
  p->vertexlist[1] = 6;
  p->vertexlist[2] = 7;
  p->vertexlist[3] = 8;

  // Facet 3. The bottom facet.
  f = &in.facetlist[2];
  f->numberofpolygons = 1;
  f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
  f->numberofholes = 0;
  f->holelist = NULL;
  p = &f->polygonlist[0];
  p->numberofvertices = 4;
  p->vertexlist = new int[p->numberofvertices];
  p->vertexlist[0] = 1;
  p->vertexlist[1] = 5;
  p->vertexlist[2] = 6;
  p->vertexlist[3] = 2;

  // Facet 4. The back facet.
  f = &in.facetlist[3];
  f->numberofpolygons = 1;
  f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
  f->numberofholes = 0;
  f->holelist = NULL;
  p = &f->polygonlist[0];
  p->numberofvertices = 4;
  p->vertexlist = new int[p->numberofvertices];
  p->vertexlist[0] = 2;
  p->vertexlist[1] = 6;
  p->vertexlist[2] = 7;
  p->vertexlist[3] = 3;

  // Facet 5. The top facet.
  f = &in.facetlist[4];
  f->numberofpolygons = 1;
  f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
  f->numberofholes = 0;
  f->holelist = NULL;
  p = &f->polygonlist[0];
  p->numberofvertices = 4;
  p->vertexlist = new int[p->numberofvertices];
  p->vertexlist[0] = 3;
  p->vertexlist[1] = 7;
  p->vertexlist[2] = 8;
  p->vertexlist[3] = 4;

  // Facet 6. The front facet.
  f = &in.facetlist[5];
  f->numberofpolygons = 1;
  f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
  f->numberofholes = 0;
  f->holelist = NULL;
  p = &f->polygonlist[0];
  p->numberofvertices = 4;
  p->vertexlist = new int[p->numberofvertices];
  p->vertexlist[0] = 4;
  p->vertexlist[1] = 8;
  p->vertexlist[2] = 5;
  p->vertexlist[3] = 1;

  // Set 'in.facetmarkerlist'

  in.facetmarkerlist[0] = -1;
  in.facetmarkerlist[1] = -2;
  in.facetmarkerlist[2] = 0;
  in.facetmarkerlist[3] = 0;
  in.facetmarkerlist[4] = 0;
  in.facetmarkerlist[5] = 0;

  // Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
  //   do quality mesh generation (q) with a specified quality bound
  //   (1.414), and apply a maximum volume constraint (a0.1).

  tetrahedralize("zpq1.414a0.1", &in, &out);
//  tetrahedralize("zp", &in, &out);

  // Output mesh to files 'barout.node', 'barout.ele' and 'barout.face'.
  out.save_nodes(FILE_PREFIX);
  out.save_elements(FILE_PREFIX);
  out.save_faces(FILE_PREFIX);
}

static void cleanupTetgen() {
  cout << "deleting " << ELE_FILE << endl;
  unlink(ELE_FILE);
  cout << "deleting " << FACE_FILE << endl;
  unlink(FACE_FILE);
  cout << "deleting " << NODE_FILE << endl;
  unlink(NODE_FILE);
}

namespace TetraCube {

}

btSoftBody *generateTetraBox(const btVector3 &dims, btSoftBodyWorldInfo &worldInfo) {
  runTetgen(dims);
  boost::scoped_array<char> eleStr(readWholeFile(ELE_FILE));
  boost::scoped_array<char> nodeStr(readWholeFile(NODE_FILE));
  btSoftBody *psb = btSoftBodyHelpers::CreateFromTetGenData(worldInfo,
    eleStr.get(), 0, nodeStr.get(),
    false, true, true);
  cleanupTetgen();
  return psb;
}

BulletSoftObject::Ptr makeTetraBox(const vector<btVector3>& points, btScalar thickness, btSoftBodyWorldInfo& worldInfo) {
  btVector3 dims(points[0].distance(points[1]), points[0].distance(points[2]), thickness);
  //btSoftBody *psb = generateTetraBox(dims, worldInfo);
  btSoftBody *psb = 0;//btSoftBodyHelpers::CreateFromTetGenData(worldInfo,
    // TetraCube::getElements(), 0, TetraCube::getNodes(),
    // false, true, true);
//  psb->scale(btVector3(0.9, 0.9, 0.9));
  psb->transform(btTransform(btQuaternion(0, 0, 0, 1), points[0]));
  psb->setVolumeMass(1);
  psb->m_cfg.piterations=1;
  psb->generateClusters(16);
  psb->getCollisionShape()->setMargin(0.01);
  psb->m_cfg.collisions	= btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS
  //+ btSoftBody::fCollision::CL_SELF
  		;
  //psb->m_materials[0]->m_kLST=0.8;
  psb->m_materials[0]->m_kLST		=	0.1;
  psb->m_materials[0]->m_kAST		=	0.1;
  psb->m_materials[0]->m_kVST		=	0.1;
  return BulletSoftObject::Ptr(new BulletSoftObject(psb));
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
