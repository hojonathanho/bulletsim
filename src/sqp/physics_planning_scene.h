#pragma once
#include "simulation/simplescene.h"

struct PhysicsPlanningScene : public Scene {
  /*
  maintains two environments: one with padded shapes, used for planning,
  one with normal shapes, used for visualization and simulation
  should be able to update planning scene's rave environment based on physics scene
  */
  ScenePtr m_planScene;
  int m_planVisible;
    

  PhysicsPlanningScene(EnvironmentBasePtr rave);
  void step(float dt, int maxsteps, float internaldt);
  void updatePlanningScene();
  void updateRaveFromBullet();
  void toggleVisible();
};
