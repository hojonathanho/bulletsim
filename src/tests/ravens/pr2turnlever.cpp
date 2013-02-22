/** \example orpr2turnlever.cpp
    \author Rosen Diankov

    Shows how to set a workspace trajectory for the hand and have a robot plan it.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/format.hpp>
#include <iostream>

#include "orexample.h"
#include <openrave/plannerparameters.h>
#include <openrave/planningutils.h>

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {


std::vector<dReal> mirror_arm_joints(const std::vector<dReal> &x) {
    //mirror image of joints (r->l or l->r)
	assert(("Mirror Joints: Expecting 7 values. Not found.", x.size()==7));
	dReal vec[] = {-1*x[0],x[1],-1*x[2],x[3],-1*x[4],x[5],-1*x[6]};
	std::vector<dReal> mirrored;
	mirrored.assign(vec,vec+7);
    return mirrored;
}


class PR2 {
private:
	EnvironmentBasePtr env;
	RobotBase::ManipulatorPtr rmanip, lmanip;

public:
	PR2(EnvironmentBasePtr penv) : env(penv) {}
};



class PR2TurnLevelExample : public OpenRAVEExample
{

private:
	//handles for plotting
	vector<GraphHandlePtr> plot_handles;

	// arm postures
    const dReal _up[7];  //=
    std::vector<dReal> arm_up;

    const dReal _side[7];
    std::vector<dReal> arm_side;

public:

    PR2TurnLevelExample() :
    		_up({0.33, -0.35, 2.59, -0.15, 0.59, -1.41, -0.27}),
    		_side({1.832, -0.332, 1.011, -1.437, 1.1 , -2.106, 3.074}) {
    	arm_up.assign(_up, _up+7);
    	arm_side.assign(_side, _side+7);
    }


    /** Set the joint values of the L('l' : left) or R('r' : right)
     * of the robot PROBOT to the JOINT_VALUES. */
    void setArmJointAngles(RobotBasePtr probot,
    					   const vector<dReal> &joint_values, char lr = 'l') {
    	string manip_name = probot->GetActiveManipulator()->GetName();
    	assert(("Set Joint Angles: Expecting 7 values. Not found.",
    			joint_values.size()==7));

    	EnvironmentBasePtr penv = probot->GetEnv();
    	{
    		EnvironmentMutex::scoped_lock lock(penv->GetMutex());
    		if (lr=='l') {
    			probot->SetActiveManipulator("leftarm");
    		} else {
    			probot->SetActiveManipulator("rightarm");
    		}

    		RobotBase::ManipulatorPtr manip = probot->GetActiveManipulator();
    		probot->SetActiveDOFs(manip->GetArmIndices());
    		probot->SetActiveDOFValues(joint_values,true);
    	}

    	probot->SetActiveManipulator(manip_name);
    }

    /** Set the joint values of both the arms to
     *  left joints to JOINTS_LEFT and right to JOINTS_RIGHT.  */
    void setBothArmsJointAngles(RobotBasePtr probot,
    		const vector<dReal> &joint_left,
    		const vector<dReal> &joint_right) {

    	EnvironmentBasePtr penv = probot->GetEnv();
    	string manip_name = probot->GetActiveManipulator()->GetName();
    	{
    		EnvironmentMutex::scoped_lock lock(penv->GetMutex());

    		assert(("Set Both Arms : Expecting 7 values. Not found.",
    				joint_left.size()==7 && joint_right.size()==7));

    		probot->SetActiveManipulator("leftarm");
    		RobotBase::ManipulatorPtr left_manip = probot->GetActiveManipulator();
    		probot->SetActiveDOFs(left_manip->GetArmIndices());
    		probot->SetActiveDOFValues(joint_left,true);

    		probot->SetActiveManipulator("rightarm");
    		RobotBase::ManipulatorPtr right_manip = probot->GetActiveManipulator();
    		probot->SetActiveDOFs(right_manip->GetArmIndices());
    		probot->SetActiveDOFValues(joint_right,true);
    	}
    	probot->SetActiveManipulator(manip_name);
    }


    void setArmPose(RobotBasePtr probot, std::string pose="side", char lrb='l') {
    	vector<dReal> * joints;
    	if (pose=="up") {
    		joints = &arm_side;
    	} else {
    		joints = &arm_up;
    	}

    	if (lrb == 'l') {
    		setArmJointAngles(probot, *joints, 'l');
    	} else if (lrb == 'r') {
    		vector<dReal> right_joints = mirror_arm_joints(*joints);
    		setArmJointAngles(probot, right_joints, 'r');
    	} else {
    		vector<dReal> right_joints = mirror_arm_joints(*joints);
    		setBothArmsJointAngles(probot, *joints, right_joints);
    	}
    }


    /** Sets the torso link VALUE.
     *  Up is value == 1 | Down is value == 0 */
    void setTorso(RobotBasePtr probot, dReal value) {

    	RobotBase::JointPtr torso_joint = probot->GetJoint("torso_lift_joint");
    	int joint_dof_idx = torso_joint->GetDOFIndex();

    	std::vector<int> joint_dof;
    	joint_dof.push_back(joint_dof_idx);
    	std::vector<double> values;
    	values.push_back(value);

    	probot->SetDOFValues(values, 1, joint_dof);
    	// TO GET THE DOF VALUES : probot->GetDOFValues(values, indices);
    }


    PlannerAction PlanCallback(const PlannerBase::PlannerProgress& progress) {
        // plan callback
        return PA_None;
    }

    virtual void WaitRobot(RobotBasePtr probot) {
        // unlock the environment and wait for the robot to finish
        while(!probot->GetController()->IsDone() && IsOk()) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

    virtual void demothread(int argc, char ** argv) {
        string scenefilename = "/home/ankush/sandbox/rave_suture/suture_env.zae";
        RaveSetDebugLevel(Level_Debug);
        penv->Load(scenefilename);

        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);
        RobotBasePtr probot = vrobots.at(0);
        std::cout<<"ROBOT NAME: "<<probot->GetName()<<std::endl;


        vector<KinBodyPtr> vbodies;
        penv->GetBodies(vbodies);
        vbodies[0]->Enable(false);
        vbodies[1]->Enable(false);


        //penv->Remove(penv->GetKinBody("table"));

        RobotBase::ManipulatorPtr pmanip = probot->SetActiveManipulator("rightarm");

        KinBodyPtr target = penv->GetKinBody("lever");
        if( !target ) {
             target = RaveCreateKinBody(penv,"");
             std::vector<AABB> boxes(1);
             boxes[0].pos = Vector(0,0.1,0);
             boxes[0].extents = Vector(0.01,0.1,0.01);
             target->InitFromBoxes(boxes,true);
             target->SetName("lever");
             penv->Add(target);

             Transform t = pmanip->GetEndEffectorTransform();
             t.trans += Vector(-0.3,0.0,-0.2);

             Transform t1;
             t1.trans = t.trans;
             target->SetTransform(t1);
         }


        setArmPose(probot, "up", 'b');
        setTorso(probot, 1);

        // load inverse kinematics using ikfast
        ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
        penv->Add(pikfast,true,"");
        stringstream ssin,ssout;
        vector<dReal> vsolution;
        ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IKP_Transform6D;
        if( !pikfast->SendCommand(ssout,ssin) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("failed to load iksolver", ORE_Assert);
        }


        // create the planner parameters
        PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");

        ModuleBasePtr basemodule = RaveCreateModule(penv,"BaseManipulation");
        penv->Add(basemodule,true,probot->GetName());
        ModuleBasePtr taskmodule = RaveCreateModule(penv,"TaskManipulation");
        penv->Add(taskmodule,true,probot->GetName());

        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
            stringstream ssin, ssout; ssin << "ReleaseFingers";
            taskmodule->SendCommand(ssout,ssin);
        }
        WaitRobot(probot);


        TrajectoryBasePtr workspacetraj;
        Transform Tgrasp0;
        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
            Transform Toffset;
            Toffset.trans = Vector(0,0.2,0);
            Transform Ttarget0 = target->GetTransform();
            Transform Ttarget1 = Ttarget0 * matrixFromAxisAngle(Vector(PI/2,0,0));

            Tgrasp0 = matrixFromAxisAngle(Vector(PI/2,0,0))*matrixFromAxisAngle(Vector(0,PI/2,0));
            Tgrasp0.trans = Ttarget0*Toffset.trans;
            Transform Tgraspoffset = Ttarget0.inverse() * Tgrasp0;
            Transform Tgrasp1 = Ttarget1 * Tgraspoffset;

            ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D,"linear");
            workspacetraj = RaveCreateTrajectory(penv,"");
            vector<dReal> values;
            workspacetraj->Init(spec);

            // vector of points for plotting the trajectory way-points
            vector<RaveVector<float> > points;
            for(size_t i = 0; i < 32; ++i) {
                dReal angle = i*0.05;
                Transform Ttarget = Ttarget0 * matrixFromAxisAngle(Vector(angle,0,0));
                Transform Tgrasp = Ttarget*Tgraspoffset;
                points.push_back(Tgrasp.trans);

                IkParameterization ikparam(Tgrasp,IKP_Transform6D);
                values.resize(ikparam.GetNumberOfValues());
                RAVELOG_INFO("input waypoints size: %d\n", values.size());
                ikparam.GetValues(values.begin());
                workspacetraj->Insert(workspacetraj->GetNumWaypoints(),values);
            }
            //plot the trajectory way-points
            plot_handles.push_back(penv->plot3(&points[0].x, points.size(), sizeof(points[0]), 5.0f));

            std::vector<dReal> maxvelocities(7,1.0);
            std::vector<dReal> maxaccelerations(7,5.0);
            planningutils::RetimeAffineTrajectory(workspacetraj,maxvelocities,maxaccelerations);
            RAVELOG_INFO(str(boost::format("duration=%f, points=%d")%workspacetraj->GetDuration()%workspacetraj->GetNumWaypoints()));
        }


        // see if trajectory can be extracted: Yes it can be!
        TrajectoryBasePtr hand_traj;
        hand_traj = RaveCreateTrajectory(penv,"");
        {
            stringstream ssout, ssin; ssin << "MoveToHandPosition outputtraj execute 0 poses 1  " << Tgrasp0;
            basemodule->SendCommand(ssout,ssin);
            hand_traj->deserialize(ssout);
        }
        WaitRobot(probot);
        RAVELOG_INFO(str(boost::format("****************** MOVE HAND TRAJECTORY*********************\n duration=%f, points=%d")%hand_traj->GetDuration()%hand_traj->GetNumWaypoints()));


        {
        	stringstream ssout, ssin; ssin << "MoveToHandPosition poses 1 " << Tgrasp0;
        	basemodule->SendCommand(ssout,ssin);
        }
        WaitRobot(probot);


        {
            stringstream ssin, ssout; ssin << "CloseFingers";
            taskmodule->SendCommand(ssout,ssin);
        }
        WaitRobot(probot);

        int a;
        std::cout<<"Enter something: ";
        std::cin>>a;

        list<TrajectoryBasePtr> listtrajectories;

       	{
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
            probot->SetActiveDOFs(pmanip->GetArmIndices());
            RAVELOG_INFO("*************** ROBOT DOFs: %d********************\n", probot->GetActiveDOF());
            probot->Grab(target);
            PlannerBasePtr planner = RaveCreatePlanner(penv,"workspacetrajectorytracker");
            WorkspaceTrajectoryParametersPtr params(new WorkspaceTrajectoryParameters(penv));
            params->SetRobotActiveJoints(probot); // set planning configuration space to current active dofs
            params->workspacetraj = workspacetraj;

            RAVELOG_INFO("starting to plan\n");
            if( !planner->InitPlan(probot,params) ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("plan init failed",ORE_Assert);
            }


            // create a new output trajectory
            TrajectoryBasePtr outputtraj = RaveCreateTrajectory(penv,"");
            if( !planner->PlanPath(outputtraj) ) {
            	//continue;
            	throw OPENRAVE_EXCEPTION_FORMAT0("plan failed",ORE_Assert);
            }
            listtrajectories.push_back(outputtraj);
            listtrajectories.push_back(planningutils::ReverseTrajectory(outputtraj));




            vector<dReal> data;
            /*int num_waypoints = outputtraj->GetNumWaypoints();
            RAVELOG_INFO(">>>>>>> There are %d points in the output trajectory.\n", num_waypoints);
            for (int i = 0; i < num_waypoints; i++ ){
            	// returns the i^th waypoint in the trajectory. configurationspace is mentioned to get the values for the appropriate transform;
            	// the waypoint fills up data with 7 values : 3 translation; 4 rotation [quaternion] xyzw.
            	// Instead of just getting the way-points, we can sample the trajectories
            	// sampling trajectory: traj->Sample(vector<dReal> sample, time [float], configurationspec);
            	// sampling interpolates the values of the configuration space.
            	// printing sampled content:
            	//    for (int i=0; i<sample.size(); i++)
            	//        	std::cout<<sample[i]<<std::endl;

            	outputtraj->GetWaypoint(i,data, params->_configurationspecification);
            	RAVELOG_INFO("WAYPOINT. size = %d\n", data.size());
            	for (int i=0; i<data.size(); i++)
            		std::cout<<"   "<<data[i]<<std::endl;
            	std::cout<<"-------"<<std::endl;
            }*/

            //extract the joint values from the output trajectory
            RAVELOG_INFO("Robot number of Active DOFs = %d\n ", probot->GetActiveDOF());
            vector<dReal> joints(probot->GetActiveDOF());
            vector<int> indices = probot->GetActiveDOFIndices();
            vector<dReal> sample;
            outputtraj->Sample(sample, outputtraj->GetDuration());
            outputtraj->GetConfigurationSpecification().ExtractJointValues(joints.begin(),sample.begin(),probot,indices);
            for(int k=0; k<joints.size(); k++)
            	std::cout<<"        "<<joints[k]<<std::endl;
            std::cout<<"-------"<<std::endl;

            RAVELOG_INFO("DURATION + 1\n ");
            outputtraj->Sample(sample, outputtraj->GetDuration()+10);
            outputtraj->GetConfigurationSpecification().ExtractJointValues(joints.begin(),sample.begin(),probot,indices);
            for(int k=0; k<joints.size(); k++)
            	std::cout<<"        "<<joints[k]<<std::endl;
            std::cout<<"-------"<<std::endl;


            // validate against the current joint values
            //std::vector<int> ;
            //joint_dof.push_back(joint_dof_idx);
            std::vector<double> values;
        	probot->GetDOFValues(values, probot->GetActiveDOFIndices());
        	RAVELOG_INFO("Validating against current joint angles....\n");
        	assert(("Unknown number of DOF!!", values.size()==7));
        	for(int k=0; k<values.size(); k++)
        		std::cout<<"        "<<values[k]<<std::endl;
        	std::cout<<"-------"<<std::endl;
        }

        //RAVELOG_INFO(probot->GetController()->GetDescription());

       	//while(IsOk()) {
            for(list<TrajectoryBasePtr>::iterator it = listtrajectories.begin(); it != listtrajectories.end(); ++it) {
            	probot->GetController()->SetPath(*it);
                WaitRobot(probot);
            }
            probot->GetController()->Reset();

        //}

            while(IsOk()) {
            	boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
    }

};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::PR2TurnLevelExample example;
    return example.main(argc,argv);
}

