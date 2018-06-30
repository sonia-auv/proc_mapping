//
// Created by coumarc9 on 7/21/17.
//

#ifndef PROC_MAPPING_HYDROOBJECTIVE_H
#define PROC_MAPPING_HYDROOBJECTIVE_H

#include <ros/node_handle.h>
#include <proc_hydrophone/PingPose.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

namespace proc_mapping{
    class HydroObjective {
    public:
        HydroObjective(double_t distance = 1);
        ~HydroObjective();

        void addPing(const proc_hydrophone::PingPoseConstPtr &ping);

        inline void setOdom(const nav_msgs::OdometryConstPtr &odom){ odom_ = odom; }

        geometry_msgs::PoseConstPtr getPoint();

        inline void setDistance(double_t distance) { ROS_INFO_STREAM("Setting distance to " << distance << "m"); distance_ = distance; };
        void resetQueue();

    private:

        std::list<geometry_msgs::PoseConstPtr> points_;
        double_t distance_;
        nav_msgs::OdometryConstPtr odom_;

    };
}




#endif //PROC_MAPPING_HYDROOBJECTIVE_H
