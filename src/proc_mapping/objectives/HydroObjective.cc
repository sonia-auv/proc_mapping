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

    void HydroObjective::addPing(const proc_hydrophone::PingPoseConstPtr &ping) {

        pings.push_back(ping);

        auto function = GetFunction(ping);

        function.print();

        // TODO Handle matrix

    }

    arma::mat HydroObjective::GetFunction(const proc_hydrophone::PingPoseConstPtr &ping) {

        double heading = ping->pose.orientation.z;
        double x = ping->pose.position.x;
        double y = ping->pose.position.y;

        double m = sin(heading) / cos(heading);

        double b = y - m * x;

        arma::mat matrix(2,1);

        matrix(0,0) = m;
        matrix(1,0) = b;

        return matrix;
    }
}


