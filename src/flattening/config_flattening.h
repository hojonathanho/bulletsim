#ifndef __FL_CONFIG_H__
#define __FL_CONFIG_H__

#include "utils/config.h"

struct FlatteningConfig : Config {
    static bool useFakeGripper;

    FlatteningConfig() : Config() {
        params.push_back(new Parameter<bool>("useFakeGripper", &useFakeGripper, "use telekinetic gripper, no ik required"));
    }
};

#endif // __FL_CONFIG_H__
