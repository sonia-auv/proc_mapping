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
          map_pub_(),
          markers_pub_(),
          reset_map_sub_(),
          position_(nh),
          buoys_("buoys", 3),
          fence_("fence", 1),
          pinger_("pinger", 1)

    {

        // TODO Use attributes initialisation

        markers_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/markers", 100);


        hydro_sub_ = nh_->subscribe("/provider_hydrophone/markers", 100, &ProcMappingNode::MarkersCallback, this);

        proc_image_sub_ = nh_->subscribe("/proc_image_processing/markers",100, &ProcMappingNode::MarkersCallback, this);

        provider_activation_request_sub_ = nh_->subscribe("/proc_mapping/provider_activation_request/", 100, &ProcMappingNode::ProviderActivationRequestCallback, this);

        mapping_request_sub_ = nh_->subscribe("/proc_mapping/mapping_request", 100, &ProcMappingNode::MappingRequestCallback, this);
        mapping_response_pub_ = nh_->advertise<proc_mapping::MappingResponse>("/proc_mapping/mapping_response", 100);

        // TODO Param
        if (true)
        {
            debug = new Debug(nh_);
        }

    }

    //------------------------------------------------------------------------------
    //
    ProcMappingNode::~ProcMappingNode() {

        if (debug)
        {
            delete debug;
        }

    }

    //==============================================================================
    // M E T H O D   S E C T I O N

    //------------------------------------------------------------------------------
    //
    //void ProcMappingNode::ResetMapCallback(
    //    const sonia_msgs::ResetMap::ConstPtr &msg) {
    //  ROS_INFO("Resetting the mappers object.");
    //  sonar_mapper_.ResetMapper();
    //}

    //------------------------------------------------------------------------------
    //
    void ProcMappingNode::Spin() {
      ros::Rate r(15);  // 15 hz
            //int id=0;



      while (ros::ok()) {
        ros::spinOnce();

          markers_pub_.publish(markers);

        if (debug)
        {
            debug->sendDebugData();
        }

        r.sleep();
      }
    }


    void ProcMappingNode::MarkersCallback(const visualization_msgs::MarkerArray::ConstPtr &markers) {

        std::vector<visualization_msgs::Marker> buoysMarkers;
        std::vector<visualization_msgs::Marker> fenceMarkers;
        std::vector<visualization_msgs::Marker> pingerMarkers;

        for (unsigned int i = 0; i < markers->markers.size(); i++) {

            auto marker = markers->markers[i];

            switch (marker.type)
            {
                case visualization_msgs::Marker::SPHERE:
                    buoysMarkers.push_back(visualization_msgs::Marker(marker));
                    break;
                case visualization_msgs::Marker::CUBE:
                    fenceMarkers.push_back(visualization_msgs::Marker(marker));
                    break;
                case visualization_msgs::Marker::CYLINDER:
                    pingerMarkers.push_back(visualization_msgs::Marker(marker));

                    break;
                default:
                    ROS_ERROR("Type of marker not supported");
                    break;
            }

            marker.header.frame_id = "NED";
            marker.header.stamp = ros::Time::now();

            marker.ns = std::to_string(marker.id);

            marker.id = 0;
            marker.action = visualization_msgs::Marker::ADD;

            marker.color.a = 1;

            marker.color.b = 1;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.lifetime = ros::Duration();

            this->markers.markers.push_back(marker);

        }

        if (!buoysMarkers.empty())
        {
            buoys_.addMarkers(buoysMarkers);
            //buoys_.getObjectives();
        }
        if (!fenceMarkers.empty())
        {
            fence_.addMarkers(fenceMarkers);
            //fence_.getObjectives();
        }
        if (!pingerMarkers.empty())
        {
            pinger_.addMarkers(pingerMarkers);
            //pinger_.getObjectives();
        }

    }

    void ProcMappingNode::MappingRequestCallback(const proc_mapping::MappingRequest::ConstPtr &request)
    {

        proc_mapping::MappingResponse response;

        response.mapping_request = *request;

        std::vector<visualization_msgs::Marker> objectives;

        switch (request->object_type)
        {
            case MappingRequest::BUOY:
                objectives = buoys_.getObjectives();
                break;

            case MappingRequest::FENCE:
                objectives = fence_.getObjectives();
                break;

            case MappingRequest::PINGER:
                objectives = pinger_.getObjectives();
                break;
        }

        for (visualization_msgs::Marker marker : objectives)
        {
            response.data.markers.push_back(marker);
        }

        mapping_response_pub_.publish(response);
    }

    void ProcMappingNode::ProviderActivationRequestCallback(const proc_mapping::ProviderActivationRequest::ConstPtr &request) {

        // TODO Handle callback

        ROS_INFO("Callback received on '/proc_mapping/provider_activation_request'");


    }

}  // namespace proc_mapping
