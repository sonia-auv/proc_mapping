//
// Created by coumarc9 on 7/21/17.
//

#ifndef PROC_MAPPING_HYDROOBJECTIVE_H
#define PROC_MAPPING_HYDROOBJECTIVE_H

#include <proc_hydrophone/PingPose.h>
#include <geometry_msgs/Pose.h>

namespace proc_mapping{
    class HydroObjective {
    public:
        HydroObjective();
        ~HydroObjective();

        void addPing(const proc_hydrophone::PingPoseConstPtr &ping);
        geometry_msgs::PoseConstPtr getPoint();

    private:

        geometry_msgs::PoseConstPtr point;
        double distance = 1;

    };
}




#endif //PROC_MAPPING_HYDROOBJECTIVE_H
