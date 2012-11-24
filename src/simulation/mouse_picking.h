/*
 * mouse_picking.h
 *
 *  Created on: Aug 4, 2012
 *      Author: alex
 */

#ifndef MOUSE_PICKING_H_
#define MOUSE_PICKING_H_

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

class PickingMouseHandler : public osgGA::GUIEventHandler {
public:
	Scene &m_scene;

	bool m_use6Dof;
	btDynamicsWorld*		m_dynamicsWorld;
	btTypedConstraint*		m_pickConstraint;
	int m_mouseOldX;
	int m_mouseOldY;

	float m_gOldPickingDist;
	btRigidBody* m_pickedBody;//for deactivation state

	int									m_lastmousepos[2];
	btVector3							m_impact;
	btSoftBody::sRayCast				m_results;
	btSoftBody::Node*					m_node;
	btVector3							m_goal;
	bool								m_drag;

	PickingMouseHandler(Scene &scene);
	~PickingMouseHandler();
	void shootBox(const btVector3& destination);
	btVector3	getRayTo(int x, int y);
	void removePickingConstraint();
	bool processMouseInput(const osgGA::GUIEventAdapter &ea);
	bool processMouseMotionInput(const osgGA::GUIEventAdapter &ea);

  bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) {
  	if (processMouseInput(ea)) return true;
  	if (processMouseMotionInput(ea)) return true;
  	return false;
  }
};

#endif /* MOUSE_PICKING_H_ */
