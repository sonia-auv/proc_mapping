//
// Created by coumarc9 on 7/21/17.
//

#include "HydroObjective.h"

namespace proc_mapping
{
    HydroObjective::HydroObjective() {



    }

    HydroObjective::~HydroObjective() {

    }

    void HydroObjective::addPing(proc_hydrophone::PingPoseConstPtr &ping) {

        pings.push_back(ping);

        // TODO Handle matrix

    }
}


