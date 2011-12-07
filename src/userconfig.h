#ifndef __USERCONFIG_H__
#define __USERCONFIG_H__

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <LinearMath/btVector3.h>
#include <osg/Vec3>

/* HOW TO USE THIS:
 *
 * If you want to add a new option called "mysection.myoption", you must:
 * 1. Add a variable in the ConfigData structure to hold the value of the option:
 *        struct ConfigData {
 *            struct {
 *                int myoption;
 *            } mysection;
 *            // etc...
 *        };
 *    This is where you will access the option from elsewhere in the code.
 * 2. Update ConfigData::ConfigData() in userconfig.cpp. You need to specify the name
 *    of the option (if your variable is ConfigData.mysection.myoption, then the name
 *    is just mysection.myoption), the type, its default value, and a short description string.
 *        OPT(mysection.myoption, int, 42, "my cool option")
 *
 *    If the type is a vector, then you must use OPT_MULTI instead of OPT.
 *    (Note: the only type of vector supported is btVector3.)
 *
 * 3. Then elsewhere in the program, you can access your option as:
 *        CFG.mysection.myoption
 */

struct ConfigData {
    bool verbose;

    struct {
        btVector3 gravity;
        int maxSubSteps;
        float internalTimeStep;
    } bullet;

    struct {
        bool enableIK;
        bool enableHaptics;
        bool enableRobot;
        btScalar scale;
    } scene;

    struct {
        btVector3 cameraHomePosition;
        int windowWidth;
        int windowHeight;
    } viewer;


    // set default values in the constructor
    ConfigData();
    void loadFromMap(const po::variables_map &vm);
    po::options_description opts;
};

struct Config {
    static ConfigData data;
    static void read(int argc, char *argv[]);
};

// convenience macros
#define CFG (Config::data)

#endif // __USERCONFIG_H__
