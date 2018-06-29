//
// Created by coumarc9 on 7/21/17.
//

#ifndef PROC_MAPPING_HYDROOBJECTIVE_H
#define PROC_MAPPING_HYDROOBJECTIVE_H

#include <proc_hydrophone/PingPose.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace proc_mapping{
    class HydroObjective {
    public:
        HydroObjective();
        ~HydroObjective();

        void addPing(const proc_hydrophone::PingPoseConstPtr &ping);

        inline void setOdom(const nav_msgs::OdometryConstPtr &odom){ odom_ = odom; }

        geometry_msgs::PoseConstPtr getPoint();

    private:

        std::list<geometry_msgs::PoseConstPtr> points_;
        double distance = 1;
        nav_msgs::OdometryConstPtr odom_;

    };
}




#endif //PROC_MAPPING_HYDROOBJECTIVE_H
