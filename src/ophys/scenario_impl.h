#pragma once

#include "scenario.h"

class RopeLiftScenario : public Scenario {
public:
  RopeLiftScenario()
    : m_initPositions(OPhysConfig::N, 3),
      m_initManipPos(Vector7d::Zero())
  {
    for (int i = 0; i < OPhysConfig::N; ++i) {
      m_initPositions.row(i) <<
        OPhysConfig::tableDistFromRobot+0.2,
        (-0.25 + 0.5*i/(OPhysConfig::N-1.0)),
        OPhysConfig::tableHeight;
    }

    if (!OPhysConfig::useRobot) {
      m_initManipPos.head<3>() = centroid(m_initPositions) + Vector3d(0, 0, 2);
    } else {
      m_initManipPos = toEigVec(pr2RightNeutralPos());
    }

    m_destPos0 = m_initPositions.row(0).transpose() + Vector3d(1, 0, 1);
  }

  void setDestPos0(const Vector3d &p) { m_destPos0 = p; }

  virtual const MatrixX3d &getInitialRopePoints() { return m_initPositions; }

  virtual double getRopeLinkLen() {
    return (m_initPositions.row(0) - m_initPositions.row(1)).norm();
  }

  virtual const Vector7d &getInitManipDofs() { return m_initManipPos; }

  virtual double goalCost(const OptRopeState &s) {
    return (s.atTime.back().x.row(0).transpose() - m_destPos0).squaredNorm();
  }

private:
  MatrixX3d m_initPositions;
  Vector7d m_initManipPos;
  Vector3d m_destPos0;
};

class RopeDragScenario : public Scenario {
public:
  RopeDragScenario()
    : m_initPositions(OPhysConfig::N, 3),
      m_initManipPos(Vector7d::Zero())
  {
    for (int i = 0; i < OPhysConfig::N; ++i) {
      m_initPositions.row(i) <<
        OPhysConfig::tableDistFromRobot+0.35,
        0.25 + (-0.25 + 0.5*i/(OPhysConfig::N-1.0)),
        OPhysConfig::tableHeight;
    }

    if (!OPhysConfig::useRobot) {
      m_initManipPos.head<3>() = centroid(m_initPositions) + Vector3d(0, 0, 2);
    } else {
      m_initManipPos = toEigVec(pr2RightNeutralPos());
    }

    m_destPos0 = m_initPositions.row(0).transpose() + Vector3d(0, -.5, 0.2);
  }


  virtual const MatrixX3d &getInitialRopePoints() { return m_initPositions; }

  virtual double getRopeLinkLen() {
    return (m_initPositions.row(0) - m_initPositions.row(1)).norm();
  }

  virtual const Vector7d &getInitManipDofs() { return m_initManipPos; }

  virtual double goalCost(const OptRopeState &s) {
    return (s.atTime.back().x.row(0).transpose() - m_destPos0).squaredNorm();
  }

private:
  MatrixX3d m_initPositions;
  Vector7d m_initManipPos;
  Vector3d m_destPos0;
};

class PointManipScenario : public Scenario {
public:
  PointManipScenario()
    : m_initPositions(2, 3),
      m_initManipPos(Vector7d::Zero()),
      m_destPositions(2, 3)
  {
    for (int i = 0; i < OPhysConfig::N; ++i) {
      m_initPositions.row(i) <<
        OPhysConfig::tableDistFromRobot+0.2,
        -0.25 + (-0.25 + 0.5*i/(OPhysConfig::N-1.0)),
        OPhysConfig::tableHeight;
    }

    if (!OPhysConfig::useRobot) {
      m_initManipPos.head<3>() = centroid(m_initPositions) + Vector3d(0, 0, 2);
    } else {
      m_initManipPos = toEigVec(pr2RightNeutralPos());
    }

    m_destPositions.row(0) = m_initPositions.row(1) + Vector3d(0.3, 0, 0).transpose();
    m_destPositions.row(1) = m_initPositions.row(0) + Vector3d(0.3, 0, 0).transpose();
  }


  virtual const MatrixX3d &getInitialRopePoints() { return m_initPositions; }

  virtual double getRopeLinkLen() {
    return (m_initPositions.row(0) - m_initPositions.row(1)).norm();
  }

  virtual const Vector7d &getInitManipDofs() { return m_initManipPos; }

  virtual double goalCost(const OptRopeState &s) {
    return (s.atTime.back().x - m_destPositions).squaredNorm();
  }

  virtual bool disableRopeCnt() { return true; }

private:
  MatrixX3d m_initPositions, m_destPositions;
  Vector7d m_initManipPos;
};


class PointManipScenario2 : public Scenario {
public:
  PointManipScenario2()
    : m_initPositions(2, 3),
      m_initManipPos(Vector7d::Zero()),
      m_destPositions(2, 3)
  {
    for (int i = 0; i < OPhysConfig::N; ++i) {
      m_initPositions.row(i) <<
        OPhysConfig::tableDistFromRobot+0.2,
        -0.25 + (-0.25 + 0.5*i/(OPhysConfig::N-1.0)),
        OPhysConfig::tableHeight;
    }

    if (!OPhysConfig::useRobot) {
      m_initManipPos.head<3>() = centroid(m_initPositions) + Vector3d(0, 0, 2);
    } else {
      m_initManipPos = toEigVec(pr2RightNeutralPos());
    }

    m_destPositions.row(0) = m_initPositions.row(0) + Vector3d(0.3, 0, 0).transpose();
    m_destPositions.row(1) = m_initPositions.row(0);
  }


  virtual const MatrixX3d &getInitialRopePoints() { return m_initPositions; }

  virtual double getRopeLinkLen() {
    return (m_initPositions.row(0) - m_initPositions.row(1)).norm();
  }

  virtual const Vector7d &getInitManipDofs() { return m_initManipPos; }

  virtual double goalCost(const OptRopeState &s) {
    return (s.atTime.back().x - m_destPositions).squaredNorm();
  }

  virtual bool disableRopeCnt() { return true; }

private:
  MatrixX3d m_initPositions, m_destPositions;
  Vector7d m_initManipPos;
};
