#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/array.hpp>

#include "openravesupport.h"

using namespace OpenRAVE;
using namespace std;
using namespace boost::interprocess;

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define usleep(micro) Sleep(micro/1000)
#endif

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    penv->AddViewer(viewer);
    viewer->main(true);
}

int main(int argc, char ** argv)
{
    string scenefilename = "robots/pr2-beta-static.zae";
    string viewername = "qtcoin";
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();

    //boost::thread thviewer(boost::bind(SetViewer,penv,viewername)); // create the viewer
    //usleep(200000); // wait for the viewer to init
//    penv->Load(scenefilename);
    //usleep(100000); // wait for the viewer to init

/*    vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    RobotBasePtr probot = vrobots.at(0);*/

    RobotBasePtr probot = penv->ReadRobotURI(scenefilename);
    penv->AddRobot(probot);
    cout << "active dofs: " << probot->GetDOF() << endl;
    probot->SetActiveManipulator("leftarm");
    RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();

    // load inverse kinematics using ikfast
    ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
    penv->AddModule(pikfast,"");
    stringstream ssin,ssout;
        ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
        // if necessary, add free inc for degrees of freedom
        ssin << " " << 0.04f;

    if( !pikfast->SendCommand(ssout,ssin) ) {
        RAVELOG_ERROR("failed to load iksolver\n");
    }
    if( !pmanip->GetIkSolver()) {
        penv->Destroy();
        return 1;
    }

    // initialize the message queue
    message_queue::remove(RAVE_TO_SIMULATION_MQ_NAME);
    const size_t bufSize = 4096;
    const int maxMessages = 100;
    message_queue mq(create_only, RAVE_TO_SIMULATION_MQ_NAME, maxMessages, bufSize); // 1 kb max msg size
    RobotActiveDOFsMessage msg;

    const float a = 1.0f;
    vector<dReal> vsolution;
    int n = 0;
    const Transform origt = pmanip->GetEndEffectorTransform();
    while (true) {
        EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

        Transform t = origt;
        //t.trans += Vector(a*(RaveRandomFloat()-0.5f),a*(RaveRandomFloat()-0.5f),a*(RaveRandomFloat()-0.5f));
        float x = 0.1*cos(n/10.), z = 0.1*sin(n/10.); ++n;
        t.trans += Vector(-0.1, x, z);
        //t.rot = quatMultiply(t.rot,quatFromAxisAngle(Vector(RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f)*0.2f));
        if (!probot->GetActiveManipulator()->FindIKSolution(IkParameterization(t), vsolution, true)) {
            stringstream ss; ss << "failed to get solution for target transform for end effector: " << t << endl;
            RAVELOG_INFO(ss.str());
            continue;
        }

        probot->SetActiveDOFs(pmanip->GetArmIndices());
        probot->SetActiveDOFValues(vsolution);
        penv->UpdatePublishedBodies();

        // now we push the arm indices and the angles out to the main simulation
        msg.activeDOFs = pmanip->GetArmIndices();
        msg.values = vsolution;

        char buffer[bufSize];
        boost::iostreams::stream<boost::iostreams::basic_array<char> > source(buffer, bufSize); 
        boost::archive::binary_oarchive oa(source); 
        oa << msg;  

        if (!mq.try_send(buffer, bufSize, 0)) {
            stringstream ss; ss << "couldn't send message for target transform for end effector: " << t << endl;
            RAVELOG_INFO(ss.str());
        }

        usleep(1000);
    }

    message_queue::remove(RAVE_TO_SIMULATION_MQ_NAME);
    RaveDestroy();
    //thviewer.join(); // wait for the viewer thread to exit
    return 0;
}
