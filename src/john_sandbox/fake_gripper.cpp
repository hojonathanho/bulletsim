#include "simulation/plotting.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "boost/foreach.hpp"
#include "utils/conversions.h"
#include <iostream>
#include <openrave/openrave.h>
#include <osg/io_utils>
#include "simulation/fake_manipulator.h"
using namespace std;
using namespace OpenRAVE;
using boost::shared_ptr;
using osg::ref_ptr;



class TelekineticGripper : public CompoundObject<BulletObject> {
public:
  typedef boost::shared_ptr<FakeGripper> Ptr;
  RaveRobotObject::Manipulator::Ptr m_manip;
  vector<BulletObject::Ptr> m_origs;
  btTransform m_tf;

  void prePhysics() {
    btTransform tf_manip = m_manip->getTransform();
    btTransform tf_obj;
    for (int i=0; i < m_origs.size(); i++) {
      m_origs[i]->motionState->getWorldTransform(tf_obj);
      children[i]->motionState->setKinematicPos(m_tf * tf_manip.inverse()*tf_obj);
    }

  }

  void setTransform(const btTransform& tf) {m_tf = tf;}

  TelekineticGripper(RaveRobotObject::Manipulator::Ptr manip) : m_manip(manip) {
    m_tf.setIdentity();

    vector<KinBody::LinkPtr> links; manip->manip->GetChildLinks(links);
    RaveRobotObject* robot = m_manip->robot;

    BOOST_FOREACH(KinBody::LinkPtr link, links) {
      BulletObject::Ptr obj = robot->associatedObj(link);
      if (obj) {
	children.push_back(BulletObject::Ptr(new BulletObject(*obj)));
	m_origs.push_back(obj);
      }
    }
  }
};

int main(int argc, char* argv[]) {

  SceneConfig::enableIK = 1;
  SceneConfig::enableHaptics = 1;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);


  Scene s;
  PR2Manager pr2m(s);

  TelekineticGripper::Ptr fake(new TelekineticGripper(pr2m.pr2Left));
  fake->setTransform(btTransform(btQuaternion(0,0,0,1), btVector3(1,1,1)));
  s.env->add(fake);

  s.startViewer();
  s.startLoop();

}
