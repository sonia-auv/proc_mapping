//
// Created by coumarc9 on 7/21/17.
//

#include "HydroObjective.h"

namespace proc_mapping
{
    HydroObjective::HydroObjective(double_t distance) : distance_(distance) {

    }

    HydroObjective::~HydroObjective() {

    }

    void HydroObjective::addPing(const proc_hydrophone::PingPoseConstPtr &ping) {

        //ping->pose.position

        geometry_msgs::PosePtr pose(new geometry_msgs::Pose());
        pose->position.x = ping->pose.position.x;
        pose->position.y = ping->pose.position.y;
        pose->position.z = ping->pose.position.z;


        pose->orientation.z = ping->pose.orientation.z;
        pose->orientation.y = ping->pose.orientation.y;

        points_.push_back(pose);

        while (points_.size() > 5)
        {
            points_.pop_front();
        }

    }

    geometry_msgs::PoseConstPtr HydroObjective::getPoint() {


        if (points_.size() == 0)
            return geometry_msgs::PosePtr(new geometry_msgs::Pose());

        std::vector<float_t > x;
        std::vector<float_t > y;

        for (auto point : points_) {

            x.push_back(cos(point->orientation.z));
            y.push_back(sin(point->orientation.z));
        }

        std::sort(x.begin(), x.begin() + x.size());
        std::sort(y.begin(), y.begin() + y.size());

        float_t medianX;
        float_t medianY;

        if (points_.size() % 2)
        {
            int32_t median = points_.size() / 2;
            medianX = x[median];
            medianY = y[median];
        }
        else
        {
            int32_t median = points_.size() / 2 - 1;
            medianX = (x[median] + x[median + 1]) / 2;
            medianY = (y[median] + y[median + 1]) / 2;

        }

        double_t heading = atan2(medianY, medianX);

        geometry_msgs::PosePtr pose(new geometry_msgs::Pose());

        pose->position.x = odom_->pose.pose.position.x + distance_ * cos(heading);
        pose->position.y = odom_->pose.pose.position.y + distance_ * sin(heading);
        pose->position.z = odom_->pose.pose.position.z;


        pose->orientation.z = heading;
        pose->orientation.y = points_.back()->orientation.y;


        return pose;
    }




}


