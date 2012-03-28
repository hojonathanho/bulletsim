#include "clothutil.h"

#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "nodeactions.h"

ClothSpec::ClothSpec(btSoftBody *psb_,
        int resx_, int resy_,
        btScalar lenx_, btScalar leny_) :
            psb(psb_), resx(resx_), resy(resy_), lenx(lenx_), leny(leny_),
            cloud(new pcl::PointCloud<pcl::PointXYZ>()),
            kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {

    cloud->points.resize(resx * resy);
    kdtree->setInputCloud(cloud);
}

void ClothSpec::updateAccel() {
    // fill in cloud with cloth points
    BOOST_ASSERT(cloud->points.size() == resx * resy);
    for (int i = 0; i < cloud->points.size(); ++i) {
        const btVector3 &p = psb->m_nodes[i].m_x;
        cloud->points[i].x = p.x();
        cloud->points[i].y = p.y();
        cloud->points[i].z = p.z();
    }

    // update kdtree
    kdtree->setInputCloud(cloud);
}

void liftClothEdges(Scene &scene, ClothSpec &cs, bool disableDrawing) {
    bool d = scene.drawingOn;
    scene.setDrawing(!disableDrawing);

    const int steps = 20;
//    const int steps = 50;
    const btVector3 force(0, 0, 0.5);
    for (int i = 0; i < steps; ++i) {
        for (int y = 0; y < cs.resy; ++y) {
            cs.psb->addForce(force, y*cs.resx + 0);
            cs.psb->addForce(force, y*cs.resx + cs.resx-1);
        }
        scene.step(BulletConfig::dt);
    }

    // let the cloth stabilize
    const int restingsteps = 400;
    for (int i = 0; i < restingsteps; ++i) {
        scene.step(BulletConfig::dt);
    }
    //scene.stepFor(BulletConfig::dt, 10);

    // clear velocities
    for (int i = 0; i < cs.psb->m_nodes.size(); ++i) {
        cs.psb->m_nodes[i].m_v.setZero();
    }

    scene.setDrawing(d);
}

void liftClothMiddle(Scene &scene, ClothSpec &cs, bool disableDrawing) {
    bool d = scene.drawingOn;
    scene.setDrawing(!disableDrawing);

    // add forces
//    const int steps = 30;
    const int steps = 60;
//    const btVector3 force(0, 0, 0.5);
    const btVector3 force(0, 0, 0.5);
    for (int i = 0; i < steps; ++i) {
        for (int x = 0; x < cs.resx; ++x)
            cs.psb->addForce(force, (cs.resy / 2)*cs.resx + x);
        scene.step(BulletConfig::dt);
    }

    // let the cloth stabilize
    const int restingsteps = 400;
    for (int i = 0; i < restingsteps; ++i) {
        scene.step(BulletConfig::dt);
    }

    // clear velocities
    for (int i = 0; i < cs.psb->m_nodes.size(); ++i) {
        cs.psb->m_nodes[i].m_v.setZero();
    }
    scene.setDrawing(d);

    //liftClothMiddle2(scene, cs, disableDrawing);
}

#if 0
// runs some random actions on a cloth
void randomizeCloth(Scene &scene, btSoftBody *psb, int resx, int resy, int numSteps) {
    NodeActionList actions;
    genSpecsForCloth(actions, resx, resy);
    for (int i = 0; i < numSteps; ++i) {
        NodeMoveAction &a = actions[rand() % actions.size()];
        a.setSoftBody(scene.env, psb);
        scene.runAction(a, BulletConfig::dt);
    }
}
#endif

static btScalar rayFromToTriangle(const btVector3& rayFrom,
               const btVector3& rayTo,
               const btVector3& rayNormalizedDirection,
               const btVector3& a,
               const btVector3& b,
               const btVector3& c,
               btScalar maxt) {
  static const btScalar ceps=-SIMD_EPSILON*10;
  static const btScalar teps=SIMD_EPSILON*10;

  const btVector3 n=btCross(b-a,c-a);
  const btScalar d=btDot(a,n);
  const btScalar den=btDot(rayNormalizedDirection,n);
  if(!btFuzzyZero(den)) {
    const btScalar num=btDot(rayFrom,n)-d;
    const btScalar t=-num/den;
    if((t>teps)&&(t<maxt)) {
      const btVector3 hit=rayFrom+rayNormalizedDirection*t;
      if((btDot(n,btCross(a-hit,b-hit))>ceps)
     && (btDot(n,btCross(b-hit,c-hit))>ceps)
     && (btDot(n,btCross(c-hit,a-hit))>ceps)) {
    return(t);
      }
    }
  }
  return(-1);
}

int softBody_facesCrossed(const btSoftBody* psb, const btVector3& rayFrom, const btVector3& rayTo) {
  int count = 0;
  btVector3 dir = rayTo - rayFrom;
  vector<btSoftBody::Node *> nodes;
  for(int i = 0,ni = psb->m_faces.size(); i < ni; ++i) {
    const btSoftBody::Face& f = psb->m_faces[i];
    const btScalar t = rayFromToTriangle(rayFrom,
                     rayTo,
                     dir,
                     f.m_n[0]->m_x,
                     f.m_n[1]->m_x,
                     f.m_n[2]->m_x,
                     1.f);
    if(t > 0) {
      bool nodeFound = false;
      for (int j = 0; j < nodes.size(); ++j) {
    if (f.m_n[0] == nodes[j] || f.m_n[1] == nodes[j] || f.m_n[2] == nodes[j]) {
      nodeFound = true;
      break;
    }
      }
      if (!nodeFound) {
    nodes.push_back(f.m_n[0]);
    nodes.push_back(f.m_n[1]);
    nodes.push_back(f.m_n[2]);
    ++count;
      }
    }
  }
  return count;
}
