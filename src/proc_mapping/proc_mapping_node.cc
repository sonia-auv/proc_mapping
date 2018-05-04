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

#include "proc_mapping/proc_mapping_node.h"

namespace proc_mapping {

    //==============================================================================
    // C / D T O R S   S E C T I O N
    //------------------------------------------------------------------------------
    //
    ProcMappingNode::ProcMappingNode(const ros::NodeHandlePtr &nh)
        : nh_(nh),
          reset_map_sub_(),
          hydro_sub_(nh_->subscribe("/proc_hydrophone/ping", 100, &ProcMappingNode::PingsCallback, this)),
          odom_sub_(nh_->subscribe("/proc_navigation/odom", 100, &ProcMappingNode::OdomCallback, this)),
          pingerLocationPublisher(nh_->advertise<PingerLocation>("/proc_mapping/pinger_location", 100)),
          pingerLocationDebugPublisher(nh_->advertise<geometry_msgs::Point>("/proc_mapping/debug/pinger_location", 100)),
          objective_reset_srv_(nh_->advertiseService("/proc_mapping/objective_reset/", &ProcMappingNode::ObjectiveResetCallback, this)),
          pingerLocationService(nh_->advertiseService("/proc_mapping/pinger_location_service", &ProcMappingNode::PingerLocationServiceCallback, this))
    {}

    //------------------------------------------------------------------------------
    //
    ProcMappingNode::~ProcMappingNode() {}

    //==============================================================================
    // M E T H O D   S E C T I O N
    //------------------------------------------------------------------------------
    //
    void ProcMappingNode::Spin() {

        ros::Rate r(15);  // 15 hz

        auto previousStamp = ros::Time::now();

        while (ros::ok()) {

            ros::spinOnce();

//            if ((ros::Time::now() - previousStamp).sec >= 10)
//            {
//
//                auto point = pingObjective.getPoint();
//
//                if (point)
//                {
//                    PingerLocationPtr pingerLocation(new PingerLocation());
//
//                    pingerLocation->point = *point;
//                    pingerLocation->frequency = 40;
//
//                    pingerLocationPublisher.publish(pingerLocation);
//
//                    pingerLocationDebugPublisher.publish(point);
//                }
//
//                previousStamp = ros::Time::now();
//            }

            r.sleep();
        }
    }

    bool ProcMappingNode::ObjectiveResetCallback(proc_mapping::ObjectiveReset::Request &request,
                                                     proc_mapping::ObjectiveReset::Response &response) {

        switch (request.objectiveType)
        {
            case proc_mapping::ObjectiveReset::Request::ALL:
//                buoys_->reset();
//                fence_->reset();
//                pinger_->reset();
                break;

            case proc_mapping::ObjectiveReset::Request::BUOY:
//                buoys_->reset();
                break;

            case proc_mapping::ObjectiveReset::Request::FENCE:
//                fence_->reset();
                break;

            case proc_mapping::ObjectiveReset::Request::PINGER:
//                pinger_->reset();
                break;

            default:
                ROS_WARN("Invalid value for ObjectiveReset::Request::objectiveType");
                break;

        }

        return true;

    }

    bool ProcMappingNode::PingerLocationServiceCallback(proc_mapping::PingerLocationService::Request &request,
                                       proc_mapping::PingerLocationService::Response &response)
    {
        // TODO Manage frequency. To test

        response.pingerLocation.point = *(pingObjective.getPoint());
        response.pingerLocation.frequency = request.frequency;

        return true;
    }

    void ProcMappingNode::PingsCallback(const proc_hydrophone::PingPoseConstPtr &ping) {

        //std::cout << "Ping callback" << std::endl;

        pingObjective.addPing(ping);

    }

    void ProcMappingNode::OdomCallback(const nav_msgs::OdometryConstPtr &odom) {

        // TODO Prepare a reset-method
        pingObjective.setOdom(odom);

    }

}  // namespace proc_mapping
