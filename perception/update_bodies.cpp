#include "update_bodies.h"
#include "dist_math.h"
#include "utils_perception.h"
#include "config_perception.h"

PlotLines::Ptr plots::linesAB;
PlotLines::Ptr plots::linesBA;


void initTrackingPlots() {
  plots::linesAB.reset(new PlotLines(3));
  plots::linesAB->setDefaultColor(1,1,0,1);
  plots::linesBA.reset(new PlotLines(3));
  plots::linesBA->setDefaultColor(0,1,1,1);
}

vector<btVector3> getSoftBodyNodes(BulletSoftObject::Ptr psb) {
  btAlignedObjectArray<btSoftBody::Node> m_nodes = psb->softBody->m_nodes;
  vector<btVector3> out(m_nodes.size());
  for (int i=0; i < m_nodes.size(); i++) out[i] = m_nodes[i].m_x;
  return out;
}



vector<int> getNNInds(vector<btVector3> from, vector<btVector3> to) {
  MatrixXf eigenFrom = toEigenMatrix(from);
  MatrixXf eigenTo = toEigenMatrix(to);
  return argminAlongRows(pairwiseSquareDist(eigenFrom, eigenTo));

}

vector<btVector3> clothOptImpulses(BulletSoftObject::Ptr psb, const vector<btVector3>& obs) {
  const vector<btVector3> est = getSoftBodyNodes(psb);
  int nObs = obs.size();
  int nEst = est.size();

  vector<btVector3> impulses(nEst);
  vector<btVector3> abLineStarts;
  vector<btVector3> abLineEnds;

  vector<int> indObsFromEst = getNNInds(est, obs);
  for (int iEst=0; iEst < nEst; iEst++) {
    const btVector3 estPt = est[iEst];
    const btVector3 obsPt = obs[indObsFromEst[iEst]];
    impulses[iEst] = TrackingConfig::fAB * (obsPt - estPt);
    abLineStarts.push_back(obsPt);
    abLineEnds.push_back(estPt);
  }
  plots::linesAB->setPoints(abLineStarts,abLineEnds);


  vector<btVector3> baLineStarts;
  vector<btVector3> baLineEnds;

  vector<int> indEstFromObs = getNNInds(obs, est);
  for (int iObs=0; iObs < nObs; iObs++) {
    int iEst = indEstFromObs[iObs];
    const btVector3 obsPt = obs[iObs];
    const btVector3 estPt = obs[iEst];
    impulses[iEst] += TrackingConfig::fBA*(obsPt - estPt);
    baLineStarts.push_back(obsPt);
    baLineEnds.push_back(estPt);
  }

  plots::linesBA->setPoints(baLineStarts,baLineEnds);

  return impulses;

}


void applyImpulses(const vector<btVector3>& impulses, BulletSoftObject::Ptr psb) {
  for (int i=0; i<impulses.size(); i++)
    psb->softBody->addForce(impulses[i],i);
}
