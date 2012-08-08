/*
 * mouse_picking.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: alex
 */

#include "mouse_picking.h"

#include <iostream>
#include "utils/conversions.h"

void pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep);

PickingMouseHandler::PickingMouseHandler(Scene &scene) :
	m_scene(scene),
	m_use6Dof(false),
	m_pickConstraint(0),
	m_mouseOldX(0),
	m_mouseOldY(0),

	m_gOldPickingDist(0),
	m_pickedBody(0),

	m_node(0),
	m_drag(false)
{
	m_dynamicsWorld = m_scene.env->bullet->dynamicsWorld;
	m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback, this, true);
}

void PickingMouseHandler::shootBox(const btVector3& destination)
{
	if (m_scene.env->bullet->dynamicsWorld)
	{
		float mass = 1.f;

		osg::ref_ptr<osg::Camera> camera = m_scene.viewer.getCamera();
		osg::Vec3f eye, center, up;
		camera->getViewMatrixAsLookAt(eye, center, up);

		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = toBulletVector(eye);
		startTransform.setOrigin(camPos);

    BoxObject::Ptr box(new BoxObject(mass, btVector3(0.05*METERS, 0.05*METERS, 0.05*METERS), btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0))));
    box->setColor(0,1,0,1);
    m_scene.env->add(box);

    btRigidBody* body = box->rigidBody.get();
		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=5.0*METERS;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setCcdMotionThreshold(0.5);
		body->setCcdSweptSphereRadius(0.9f);
//		printf("shootBox uid=%d\n", body->getBroadphaseHandle()->getUid());
//		printf("camPos=%f,%f,%f\n",camPos.getX(),camPos.getY(),camPos.getZ());
//		printf("destination=%f,%f,%f\n",destination.getX(),destination.getY(),destination.getZ());

	}
}

btVector3	PickingMouseHandler::getRayTo(int x, int y) {
	osg::ref_ptr<osg::Camera> camera = m_scene.viewer.getCamera();

	// compute model to window transform
	// Model*View*Projection*WindowMatrix
	osg::Matrixd matrix;
	matrix.postMult(m_scene.env->osg->root->getWorldMatrices()[0]);
	matrix.postMult(camera->getViewMatrix());
	matrix.postMult(camera->getProjectionMatrix());
	matrix.postMult(camera->getViewport()->computeWindowMatrix());

	osg::Matrixd inverse;
	inverse.invert(matrix);

	// Transform ray from window to model coordinates
	osg::Vec3d startRay = osg::Vec3d(x,y,0) * inverse;
	osg::Vec3d endRay = osg::Vec3d(x,y,1) * inverse;

	return toBulletVector(endRay);
}

void PickingMouseHandler::removePickingConstraint()
{
	if (m_pickConstraint && m_dynamicsWorld)
	{
		m_dynamicsWorld->removeConstraint(m_pickConstraint);
		delete m_pickConstraint;
		//printf("removed constraint");
		m_pickConstraint = 0;
		m_pickedBody->forceActivationState(ACTIVE_TAG);
		m_pickedBody->setDeactivationTime( 0.f );
		m_pickedBody = 0;
	}
}

bool PickingMouseHandler::processMouseInput(const osgGA::GUIEventAdapter &ea) {
	int x = ea.getX();
	int y = ea.getY();
	m_mouseOldX = x;
	m_mouseOldY = y;
	btVector3 rayTo = getRayTo(x, y);

	if (ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
		if ((ea.getEventType() == osgGA::GUIEventAdapter::PUSH) &&
				(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {
					shootBox(rayTo);
					return true;
		}


	} else if (ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
		if  ((ea.getEventType() == osgGA::GUIEventAdapter::PUSH) &&
				(ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {

			//add a point to point constraint for picking
			if (m_dynamicsWorld)	{
				osg::Vec3f eye, center, up;	m_scene.viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
				btVector3 rayFrom = toBulletVector(eye);

				btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
				m_dynamicsWorld->rayTest(rayFrom,rayTo,rayCallback);
				if (rayCallback.hasHit())
				{

					btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
					if (body)
					{
						//other exclusions?
						if (!(body->isStaticObject() || body->isKinematicObject()))
						{
							m_pickedBody = body;
							m_pickedBody->setActivationState(DISABLE_DEACTIVATION);

							btVector3 pickPos = rayCallback.m_hitPointWorld;
							//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());

							btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;

							if (m_use6Dof)
							{
								btTransform tr;
								tr.setIdentity();
								tr.setOrigin(localPivot);
								btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*body, tr,false);
								dof6->setLinearLowerLimit(btVector3(0,0,0));
								dof6->setLinearUpperLimit(btVector3(0,0,0));
								dof6->setAngularLowerLimit(btVector3(0,0,0));
								dof6->setAngularUpperLimit(btVector3(0,0,0));

								m_dynamicsWorld->addConstraint(dof6);
								m_pickConstraint = dof6;

								dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,0);
								dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,1);
								dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,2);
								dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,3);
								dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,4);
								dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8,5);

								dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,0);
								dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,1);
								dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,2);
								dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,3);
								dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,4);
								dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1,5);
							} else
							{
								btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
								m_dynamicsWorld->addConstraint(p2p);
								m_pickConstraint = p2p;
								p2p->m_setting.m_impulseClamp = 30.0;
								//very weak constraint for picking
								p2p->m_setting.m_tau = 0.001f;
/*
								p2p->setParam(BT_CONSTRAINT_CFM,0.8,0);
								p2p->setParam(BT_CONSTRAINT_CFM,0.8,1);
								p2p->setParam(BT_CONSTRAINT_CFM,0.8,2);
								p2p->setParam(BT_CONSTRAINT_ERP,0.1,0);
								p2p->setParam(BT_CONSTRAINT_ERP,0.1,1);
								p2p->setParam(BT_CONSTRAINT_ERP,0.1,2);
								*/
							}
							m_use6Dof = !m_use6Dof;

							m_gOldPickingDist  = (pickPos-rayFrom).length();
							return true;
						}
					}
				}
			}

			m_results.fraction=1.f;
			if(!m_pickConstraint)
			{
				osg::Vec3f eye, center, up;	m_scene.viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
				const btVector3			rayFrom=toBulletVector(eye);
				const btVector3			rayTo=getRayTo(x,y);
				const btVector3			rayDir=(rayTo-rayFrom).normalized();
				btSoftBodyArray&		sbs= ((btSoftRigidDynamicsWorld*) m_dynamicsWorld)->getSoftBodyArray();
				for(int ib=0;ib<sbs.size();++ib)
				{
					btSoftBody*				psb=sbs[ib];
					btSoftBody::sRayCast	res;
					if(psb->rayTest(rayFrom,rayTo,res))
					{
						m_results=res;
					}
				}
				if(m_results.fraction<1.f)
				{
					m_impact			=	rayFrom+(rayTo-rayFrom)*m_results.fraction;
					m_drag				=	true;
					m_lastmousepos[0]	=	x;
					m_lastmousepos[1]	=	y;
					m_node				=	0;
					switch(m_results.feature)
					{
					case btSoftBody::eFeature::Tetra:
						{
							btSoftBody::Tetra&	tet=m_results.body->m_tetras[m_results.index];
							m_node=tet.m_n[0];
							for(int i=1;i<4;++i)
							{
								if(	(m_node->m_x-m_impact).length2()>
									(tet.m_n[i]->m_x-m_impact).length2())
								{
									m_node=tet.m_n[i];
								}
							}
							break;
						}
					case	btSoftBody::eFeature::Face:
						{
							btSoftBody::Face&	f=m_results.body->m_faces[m_results.index];
							m_node=f.m_n[0];
							for(int i=1;i<3;++i)
							{
								if(	(m_node->m_x-m_impact).length2()>
									(f.m_n[i]->m_x-m_impact).length2())
								{
									m_node=f.m_n[i];
								}
							}
						}
						break;
					}
					if(m_node) m_goal=m_node->m_x;
					return true;
				}
			}


		}
	} else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE) {
		m_results.fraction=1.f;
		m_drag=false;
		removePickingConstraint();
		return true;
	}

	return false;
}


bool PickingMouseHandler::processMouseMotionInput(const osgGA::GUIEventAdapter &ea) {
	int x = ea.getX();
	int y = ea.getY();

	if(m_node&&(m_results.fraction<1.f))
	{
		if(!m_drag)
		{
#define SQ(_x_) (_x_)*(_x_)
			if((SQ(x-m_lastmousepos[0])+SQ(y-m_lastmousepos[1]))>6)
			{
				m_drag=true;
			}
#undef SQ
		}
		if(m_drag)
		{
			m_lastmousepos[0]	=	x;
			m_lastmousepos[1]	=	y;
		}
		return true;
	}

	if (m_pickConstraint)
	{
		//move the constraint pivot

		if (m_pickConstraint->getConstraintType() == D6_CONSTRAINT_TYPE)
		{
			btGeneric6DofConstraint* pickCon = static_cast<btGeneric6DofConstraint*>(m_pickConstraint);
			if (pickCon)
			{
				//keep it at the same picking distance

				btVector3 newRayTo = getRayTo(x,y);
				btVector3 rayFrom;
				btVector3 oldPivotInB = pickCon->getFrameOffsetA().getOrigin();

				osg::Vec3f eye, center, up;	m_scene.viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
				rayFrom = toBulletVector(eye);
				btVector3 dir = newRayTo-rayFrom;
				dir.normalize();
				dir *= m_gOldPickingDist;
				btVector3 newPivotB = rayFrom + dir;

				pickCon->getFrameOffsetA().setOrigin(newPivotB);
			}

		} else
		{
			btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
			if (pickCon)
			{
				//keep it at the same picking distance

				btVector3 newRayTo = getRayTo(x,y);
				btVector3 rayFrom;
				btVector3 oldPivotInB = pickCon->getPivotInB();

				osg::Vec3f eye, center, up;	m_scene.viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
				rayFrom = toBulletVector(eye);
				btVector3 dir = newRayTo-rayFrom;
				dir.normalize();
				dir *= m_gOldPickingDist;
				btVector3 newPivotB = rayFrom + dir;

				pickCon->setPivotB(newPivotB);
			}
		}
		return true;
	}

	float dx, dy;
    dx = btScalar(x) - m_mouseOldX;
    dy = btScalar(y) - m_mouseOldY;

	m_mouseOldX = x;
  m_mouseOldY = y;

  return false;
}

///for mouse picking
void pickingPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	PickingMouseHandler* handler = (PickingMouseHandler*)world->getWorldUserInfo();
	if(handler->m_drag)
	{
		const int				x=handler->m_lastmousepos[0];
		const int				y=handler->m_lastmousepos[1];
		osg::Vec3f eye, center, up;	handler->m_scene.viewer.getCamera()->getViewMatrixAsLookAt(eye, center, up);
		const btVector3			rayFrom = toBulletVector(eye);
		const btVector3			rayTo=handler->getRayTo(x,y);
		const btVector3			rayDir=(rayTo-rayFrom).normalized();
		const btVector3			N=(toBulletVector(center)-toBulletVector(eye)).normalized();
		const btScalar			O=btDot(handler->m_impact,N);
		const btScalar			den=btDot(N,rayDir);
		if((den*den)>0)
		{
			const btScalar			num=O-btDot(N,rayFrom);
			const btScalar			hit=num/den;
			if((hit>0)&&(hit<1500))
			{
				handler->m_goal=rayFrom+rayDir*hit;
			}
		}
		btVector3				delta=handler->m_goal-handler->m_node->m_x;
		static const btScalar	maxdrag=0.1*METERS;
		if(delta.length2()>(maxdrag*maxdrag))
		{
			delta=delta.normalized()*maxdrag;
		}
		handler->m_node->m_v+=delta/timeStep;
	}

}
