#include "simulation/simplescene.h"
#include "clouds/cloud_ops.h"
#include "clouds/utils_pcl.h"
#include <boost/foreach.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/unordered_map.hpp>
#include "utils/clock.h"
#include "sqp/sqp_algorithm.h"
#include "robots/pr2.h"
#include "gurobi_c++.h"
#include <bulletsim_msgs/PlanTraj.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "utils/logging.h"
#include <pcl/ros/conversions.h>
#include "pcl/io/pcd_io.h"
#include "collision_map_tools.h"
using boost::shared_ptr;
using namespace std;
using namespace Eigen;
using namespace bulletsim_msgs;


struct LocalConfig : Config {
  static int nSteps;
  static int nIter;
  static int plotDecimation;
  static string cloudTopic;
  static bool test;
  static float voxelSize;

  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("nSteps", &nSteps, "n samples of trajectory"));
    params.push_back(new Parameter<int>("nIter", &nIter, "num iterations"));
    params.push_back(new Parameter<int>("plotDecimation", &plotDecimation, "plot every k grippers"));
    params.push_back(new Parameter<string>("cloudTopic", &cloudTopic, "topic with point clouds"));
    params.push_back(new Parameter<bool>("test", &test, "run test case"));
    params.push_back(new Parameter<float>("voxelSize", &voxelSize, "voxel size"));
  }
};
int LocalConfig::nSteps = 100;
int LocalConfig::nIter = 100;
int LocalConfig::plotDecimation=1;
string LocalConfig::cloudTopic = "/camera/depth_registered/points";
bool LocalConfig::test = false;
float LocalConfig::voxelSize = .02;

class PlanningServer {
  
  ros::NodeHandle* m_nh;
  Scene& m_scene;
  PR2Manager& m_pr2m;
  CollisionBoxes::Ptr m_collisionBoxes;
  
public:
  PlanningServer(ros::NodeHandle* nh, Scene& scene, PR2Manager& pr2m) :
    m_nh(nh),
    m_scene(scene),
    m_pr2m(pr2m) {}
  
  bool callback(PlanTraj::Request& request, PlanTraj::Response& response) {
    m_pr2m.pr2->setColor(1,1,1,.4);

    ColorCloudPtr dsCloud;
    if (LocalConfig::test) {
      dsCloud = readPCD("/home/joschu/Data/scp/three_objs.pcd");
      dsCloud = downsampleCloud(dsCloud, LocalConfig::voxelSize);
    }
    else {
      std::string topicName(LocalConfig::cloudTopic);
      sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topicName, *m_nh, ros::Duration(1));
      if (!msg) {
        ROS_ERROR("Planner couldn't get a point point clouds");
        return false;
      }
      ColorCloudPtr fullCloud(new ColorCloud());
      pcl::fromROSMsg(*msg, *fullCloud);
      dsCloud = downsampleCloud(fullCloud, .02);
    }
    LOG_WARN("todo: actually filter out robot");
    dsCloud = filterX(dsCloud, .2, 100);

    if (m_collisionBoxes) m_scene.env->remove(m_collisionBoxes);
    m_collisionBoxes = collisionBoxesFromPointCloud(dsCloud, .02);
    m_scene.env->add(m_collisionBoxes);

    RaveRobotObject::Manipulator::Ptr arm;
    if (request.side == "r") arm = m_pr2m.pr2Right;
    else if (request.side == "l") arm = m_pr2m.pr2Left;
    else {
      ROS_ERROR("side must be 'r' or 'l'");
      return false;
    }

    Eigen::VectorXd startJoints = toVectorXd(request.start_joints);
    Eigen::VectorXd endJoints = toVectorXd(request.end_joints);

#if 0
    GetArmToJointGoal planner;
    planner.setup(arm, m_pr2m.pr2, m_scene.env->bullet->dynamicsWorld);
    planner.setProblem(startJoints, endJoints, LocalConfig::nSteps);
#else
    ComponentizedArmPlanningProblem planner;
    planner.setup(arm, m_pr2m.pr2, m_scene.env->bullet->dynamicsWorld);
    MatrixXd initTraj = makeTraj(startJoints, endJoints, LocalConfig::nSteps);
    shared_ptr<LengthConstraintAndCost> lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj),.5));
    shared_ptr<CollisionCost> cc(new CollisionCost(true, true, planner.m_cce.get(), -BulletConfig::linkPadding/2, 5));
    shared_ptr<JointBounds> jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj)/5, arm->manip));
    planner.addComponent(lcc);
    planner.addComponent(cc);
    planner.addComponent(jb);
    planner.initialize(initTraj);
#endif

    TrajPlotter::Ptr plotter(new ArmPlotter(arm, &m_scene, *planner.m_cce, LocalConfig::plotDecimation));
    planner.setPlotter(plotter);

    planner.optimize(LocalConfig::nIter);
    plotter->clear();

    m_pr2m.pr2->setColor(1,1,1,1);


    if (LocalConfig::test) {
      interactiveTrajPlot(planner.getTraj(), arm, &planner.m_cce->m_syncher, &m_scene);
    }


    MatrixXd result = planner.getTraj();
    response.trajectory = vector<double>();
    response.trajectory.assign(result.data(), result.data() + result.rows()*result.cols());

    return true;
  }  
};


int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.read(argc, argv);

	if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

	Scene scene;
	PR2Manager pr2m(scene);
	RaveRobotObject::Ptr pr2 = pr2m.pr2;
	RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;
	RaveRobotObject::Manipulator::Ptr larm = pr2m.pr2Left;
	BOOST_FOREACH(BulletObject::Ptr child, pr2->children) {
		if	(child) {
			btRigidBody* rb = child->rigidBody.get();
			scene.env->bullet->dynamicsWorld->removeRigidBody(rb);
		}
	}
  scene.startViewer();  

  
  if (LocalConfig::test) {
    PlanTraj::Request req;
    PlanTraj::Response resp;
    pr2m.pr2->setDOFValues(vector<int>(1,12), vector<double>(1,.287));
    PlanningServer server(NULL, scene, pr2m);
//    double start_joints_a[] = {0.197,  0.509,  1.905, -2.026,  3.482, -1.945, -4.851};
//    double end_joints_a[] = {1.516, -0.206,  0.923, -1.906,  1.981, -0.999, -3.585};

    double start_joints_a[] = {1.27082, -0.181098, 1.6, -2.3189, 3.1123, -1.04331, 1.40386};
    double end_joints_a[] = {-0.258096, -0.114419, 2.8, -2.0454, 2.83046, -2.007, 0.218691};
    req.start_joints = std::vector<double>(&start_joints_a[0],&start_joints_a[0]+7);
    req.end_joints = std::vector<double>(&end_joints_a[0],&end_joints_a[0]+7);
    req.side = "l";
    server.callback(req, resp   );
    scene.idle(true);


  }
  else {
    ros::init(argc, argv, "planning_server");
    ros::NodeHandle nh;
    PlanningServer server(&nh, scene, pr2m);
    ros::ServiceServer service = nh.advertiseService("plan_traj", &PlanningServer::callback, &server);

    while (ros::ok()) {
      scene.idleFor(.05);
      ros::spinOnce();
    }
  }

}
