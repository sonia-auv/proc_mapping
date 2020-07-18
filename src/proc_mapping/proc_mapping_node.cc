/**
 * \file	proc_mapping_node.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "proc_mapping_node.h"

namespace proc_mapping
{

    const double_t ProcMappingNode::distanceDefaultValue_ = 1;
    const std::string ProcMappingNode::distanceParamName_ = "/proc_mapping/hydro/distance";

    //==============================================================================
    // C / D T O R S   S E C T I O N
    //------------------------------------------------------------------------------
    //
    ProcMappingNode::ProcMappingNode(const ros::NodeHandlePtr &nh)
        : nh_(nh),
          reset_map_sub_(),
          hydro_sub_(nh_->subscribe("/proc_hydrophone/ping", 100, &ProcMappingNode::PingsCallback, this)),
          odom_sub_(nh_->subscribe("/proc_navigation/odom", 100, &ProcMappingNode::OdomCallback, this)),
          pingerLocationPublisher(nh_->advertise<sonia_msgs::PingerLocation>("/proc_mapping/pinger_location", 100)),
          pingerLocationDebugPublisher(nh_->advertise<geometry_msgs::Point>("/proc_mapping/debug/pinger_location", 100)),
          objective_reset_srv_(nh_->advertiseService("/proc_mapping/objective_reset/", &ProcMappingNode::ObjectiveResetCallback, this)),
          pingerLocationService(nh_->advertiseService("/proc_mapping/pinger_location_service", &ProcMappingNode::PingerLocationServiceCallback, this))
    {

        double_t distance;

        if (nh_->hasParam(distanceParamName_))
        {
            nh_->getParam(distanceParamName_, distance);
        }
        else
        {
            distance = distanceDefaultValue_;
            ROS_INFO_STREAM("No param found for " << distanceParamName_ << ". Using default value of " << distanceDefaultValue_);
        }

        hydroObjectives_[25] = HydroObjective(distance);
        hydroObjectives_[30] = HydroObjective(distance);
        hydroObjectives_[35] = HydroObjective(distance);
        hydroObjectives_[40] = HydroObjective(distance);
    }

    //------------------------------------------------------------------------------
    //
    ProcMappingNode::~ProcMappingNode() {}

    //==============================================================================
    // M E T H O D   S E C T I O N
    //------------------------------------------------------------------------------
    //
    void ProcMappingNode::Spin()
    {

        ros::Rate r(15); // 15 hz

        while (ros::ok())
        {

            ros::spinOnce();

            r.sleep();
        }
    }

    bool ProcMappingNode::ObjectiveResetCallback(sonia_msgs::ObjectiveReset::Request &request,
                                                 sonia_msgs::ObjectiveReset::Response &response)
    {

        switch (request.objectiveType)
        {
        case sonia_msgs::ObjectiveReset::Request::ALL:
            break;

        case sonia_msgs::ObjectiveReset::Request::BUOY:
            break;

        case sonia_msgs::ObjectiveReset::Request::FENCE:
            break;

        case sonia_msgs::ObjectiveReset::Request::PINGER:
            for (auto hydroObjective : hydroObjectives_)
            {
                hydroObjective.second.resetQueue();
            }

            break;

        default:
            ROS_WARN("Invalid value for ObjectiveReset::Request::objectiveType");
            break;
        }

        return true;
    }

    bool ProcMappingNode::PingerLocationServiceCallback(sonia_msgs::PingerLocationService::Request &request,
                                                        sonia_msgs::PingerLocationService::Response &response)
    {

        // If key doesn't exist
        if (hydroObjectives_.find(request.frequency) == hydroObjectives_.end())
            return false;

        HydroObjective objective = hydroObjectives_[request.frequency];

        response.pingerLocation.pose = *(objective.getPoint());
        response.pingerLocation.frequency = request.frequency;

        return true;
    }

    void ProcMappingNode::PingsCallback(const sonia_msgs::PingPoseConstPtr &ping)
    {

        uint8_t frequency = ping->frequency;

        // If key doesn't exist
        if (hydroObjectives_.find(frequency) == hydroObjectives_.end())
            return;

        hydroObjectives_[frequency].addPing(ping);

        HydroObjective objective = hydroObjectives_[frequency];

        sonia_msgs::PingerLocation pingerLocation;
        pingerLocation.pose = *(objective.getPoint());
        pingerLocation.frequency = frequency;

        pingerLocationPublisher.publish(pingerLocation);
    }

    void ProcMappingNode::OdomCallback(const nav_msgs::OdometryConstPtr &odom)
    {

        for (auto keyValue : hydroObjectives_)
        {
            hydroObjectives_[keyValue.first].setOdom(odom);
        }
    }

} // namespace proc_mapping
