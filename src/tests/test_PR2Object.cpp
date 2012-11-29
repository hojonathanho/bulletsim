/*
 * test_multihyp_tracker.cpp
 *
 *  Created on: Nov 7, 2012
 *      Author: alex
 */

#include <ros/ros.h>
#include <boost/assign.hpp>

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "robots/PR2Object.h"

Fork::Ptr forked;
PR2Object::Ptr forked_pr2;
ros::Subscriber forked_jointSub;
void forkSceneEnvironment(Scene* scene, PR2Object::Ptr pr2) {
  forked.reset(new Fork(scene->env));
  forked_pr2 = boost::static_pointer_cast<PR2Object>(forked->forkOf(pr2));

  ros::NodeHandle nh;
  forked_jointSub = nh.subscribe("/joint_states", 5, &PR2Object::setJointState, forked_pr2.get());

  // for moving the forked PR2 even if the forked environment is not in the active Scene
	scene->addVoidKeyCallback('n', boost::bind(&PR2Object::drive, forked_pr2.get(), 0,.05, 0));
	scene->addVoidKeyCallback('m', boost::bind(&PR2Object::drive, forked_pr2.get(), 0,-.05, 0));

	scene->addVoidKeyCallback('s', boost::bind(&Scene::swapEnvironment, scene, forked->env), "swap the active environment");
}

struct LocalConfig: Config {
  static float node_density;
  static float surface_density;
  LocalConfig() : Config() {
    params.push_back(new Parameter<float>("node_density", &node_density, "# nodes per unit distance. (resolution-1)/length"));
    params.push_back(new Parameter<float>("surface_density", &surface_density, "surface density for towel. (total mass)/(total area)"));
  }
};
float LocalConfig::node_density = 1/0.02; // 1 node per 2 cm
float LocalConfig::surface_density = 0.4; // 0.4 kg per m2 (100 grams for a 0.5m x 0.5m towel

int main(int argc, char *argv[]) {
    SceneConfig::enableIK = true;
    SceneConfig::enableRobot = true;
    SceneConfig::enableHaptics = true;
    GeneralConfig::scale = 100;
    SceneConfig::mouseDragScale = 10;
    BulletConfig::kinematicPolicy = 0;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(SceneConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(LocalConfig());
    parser.read(argc, argv);

    ros::init(argc, argv,"multihyp_tracker");
    ros::NodeHandle nh;

    // construct the scene
    Scene scene;

		BoxObject::Ptr table(new BoxObject(0, btVector3(1.3,1.1,0.07)*METERS, btTransform(btQuaternion(0, 0, 0, 1), btVector3(1.4, 0, 0.7)*METERS)));
//		table->rigidBody->setFriction(LocalConfig::friction);
		table->setColor(0,.8,0,.5);
		scene.env->add(table);
//		Load(scene.env, scene.rave, EXPAND(BULLETSIM_DATA_DIR)"/xml/table.xml");
//		RaveObject::Ptr table = getObjectByName(scene.env, scene.rave, "table");
//		table->setColor(0,.8,0,.5);
		float table_height = 0.7+0.07;

		BulletSoftObject::Ptr cloth = makeCloth(0.25*METERS, 0.25*METERS, btVector3(0.38,0,table_height+0.01)*METERS, LocalConfig::node_density/METERS, LocalConfig::surface_density/(METERS*METERS));
		cloth->setColor(1,1,1,1);
		scene.env->add(cloth);

  	PR2Object::Ptr pr2 = PR2Object::Ptr(new PR2Object(scene.rave)); // it seems to also be ok to pass in empty RaveInstance::Ptr()
		pr2->setColor(1,0,0,0.5);
  	scene.env->add(pr2);

//  	// Wait for first joint state message and set the PR2's initial pose
//  	sensor_msgs::JointStateConstPtr init_joint_state_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
//  	pr2->setJointState(*init_joint_state_msg.get());

  	// Set the PR2's initial pose from predefined joint state
  	sensor_msgs::JointState init_joint_state_msg;
  	const vector<double> init_joint_state_position = boost::assign::list_of(37.6985)(-33.5569)(9.9989)(6.28336)(157.197)(164.179)(9.42541)(6.66752)(7.74144)(40.8411)(-96.1862)(-51.7114)(0.276999)(-2.08061e-05)(1.26083)(-0.405239)(-1.01151)(-1.82956)(-0.3302)(-7.38323)(-1.43635)(-2.00076)(-78.4722)(0.0805294)(0.466481)(0.466481)(0.466481)(0.466481)(2.31529)(0.317579)(-0.0574898)(-12.4258)(-0.146335)(-1.25459)(69.2608)(0.0805117)(0.466369)(0.466369)(0.466369)(0.466369);
  	init_joint_state_msg.position = init_joint_state_position;
  	const vector<string> init_joint_state_name = boost::assign::list_of("fl_caster_rotation_joint")("fl_caster_l_wheel_joint")("fl_caster_r_wheel_joint")("fr_caster_rotation_joint")("fr_caster_l_wheel_joint")("fr_caster_r_wheel_joint")("bl_caster_rotation_joint")("bl_caster_l_wheel_joint")("bl_caster_r_wheel_joint")("br_caster_rotation_joint")("br_caster_l_wheel_joint")("br_caster_r_wheel_joint")("torso_lift_joint")("head_pan_joint")("head_tilt_joint")("laser_tilt_mount_joint")("r_upper_arm_roll_joint")("r_shoulder_pan_joint")("r_shoulder_lift_joint")("r_forearm_roll_joint")("r_elbow_flex_joint")("r_wrist_flex_joint")("r_wrist_roll_joint")("r_gripper_joint")("r_gripper_l_finger_joint")("r_gripper_r_finger_joint")("r_gripper_r_finger_tip_joint")("r_gripper_l_finger_tip_joint")("l_upper_arm_roll_joint")("l_shoulder_pan_joint")("l_shoulder_lift_joint")("l_forearm_roll_joint")("l_elbow_flex_joint")("l_wrist_flex_joint")("l_wrist_roll_joint")("l_gripper_joint")("l_gripper_l_finger_joint")("l_gripper_r_finger_joint")("l_gripper_r_finger_tip_joint")("l_gripper_l_finger_tip_joint");
  	init_joint_state_msg.name = init_joint_state_name;
  	pr2->setJointState(init_joint_state_msg);

    ros::Subscriber jointSub = nh.subscribe("/joint_states", 5, &PR2Object::setJointState, pr2.get());

    // Forking callback
    scene.addVoidKeyCallback('f', boost::bind(forkSceneEnvironment, &scene, pr2), "fork the Scene's Environment");

  	// start the simulation
    scene.startViewer();

    while (ros::ok()) {
    	// Draw the gripper palm transform at every iteration
      scene.addDrawOnce(PlotAxes::Ptr(new PlotAxes(pr2->getLinkTransform(pr2->robot->GetLink("l_gripper_palm_link")), 0.1*METERS)));
      scene.addDrawOnce(PlotAxes::Ptr(new PlotAxes(pr2->getLinkTransform(pr2->robot->GetLink("r_gripper_palm_link")), 0.1*METERS)));

      scene.env->step(.03,2,.015);
      scene.draw();

      if (forked)
      	forked->env->step(.03,2,.015);

      ros::spinOnce();
    }

    return 0;
}
