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

        //ping->pose.position

        geometry_msgs::PosePtr pose(new geometry_msgs::Pose());

        pose->position.x = ping->pose.position.x;// + distance * cos(ping->pose.orientation.z);
        pose->position.y = ping->pose.position.y;// + distance * sin(ping->pose.orientation.z);
        pose->position.z = ping->pose.position.z;


        pose->orientation.z = ping->pose.orientation.z;
        pose->orientation.y = ping->pose.orientation.y;

        this->point = pose;

    }

    geometry_msgs::PoseConstPtr HydroObjective::getPoint() {

        geometry_msgs::PosePtr pose(new geometry_msgs::Pose());

        pose->position.x = odom_->pose.pose.position.x + distance * cos(point->orientation.z);
        pose->position.y = odom_->pose.pose.position.y + distance * sin(point->orientation.z);
        pose->position.z = odom_->pose.pose.position.z;


        pose->orientation.z = point->orientation.z;
        pose->orientation.y = point->orientation.y;


        return pose;
    }




}


