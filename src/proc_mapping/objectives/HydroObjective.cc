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

        geometry_msgs::PointPtr point(new geometry_msgs::Point());

        point->x = ping->pose.position.x + distance * sin(ping->pose.orientation.z);
        point->y = ping->pose.position.y + distance * cos(ping->pose.orientation.z);
        point->z = ping->pose.position.z;

        this->point = point;

    }

    geometry_msgs::PointConstPtr HydroObjective::getPoint() {
        return point;
    }




}


