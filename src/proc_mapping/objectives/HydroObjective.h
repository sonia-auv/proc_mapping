//
// Created by coumarc9 on 7/21/17.
//

#ifndef PROC_MAPPING_HYDROOBJECTIVE_H
#define PROC_MAPPING_HYDROOBJECTIVE_H

#include <proc_hydrophone/PingPose.h>
#include <armadillo>

namespace proc_mapping{
    class HydroObjective {
    public:
        HydroObjective();
        ~HydroObjective();

        void addPing(const proc_hydrophone::PingPoseConstPtr &ping);

    private:

        std::vector<proc_hydrophone::PingPoseConstPtr> pings;
        arma::mat functions;// Matrix with m and b of y = mx+b linear function

        arma::mat GetFunction(const proc_hydrophone::PingPoseConstPtr &ping);

    };
}




#endif //PROC_MAPPING_HYDROOBJECTIVE_H
