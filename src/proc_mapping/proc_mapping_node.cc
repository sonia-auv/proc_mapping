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
          buoys_(new Objective("buoys", 3)),
          fence_(new Objective("fence", 1)),
          pinger_(new Objective("pinger", 1))

    {

        // TODO Use attributes initialisation

        markers_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/markers", 100);

        global_mapping_request_sub_ = nh_->subscribe("/proc_mapping/global_mapping_request", 100, &ProcMappingNode::GlobalMappingRequestCallback, this);
        local_mapping_request_sub_ = nh_->subscribe("/proc_mapping/local_mapping_request", 100, &ProcMappingNode::LocalMappingRequestCallback, this);

        global_mapping_response_pub_ = nh_->advertise<proc_mapping::GlobalMappingResponse>("/proc_mapping/global_mapping_response", 100);
        local_mapping_response_pub_ = nh_->advertise<proc_mapping::LocalMappingResponse>("/proc_mapping/local_mapping_response", 100);

        hydro_sub_ = nh_->subscribe("/proc_hydrophone/ping", 100, &ProcMappingNode::PingsCallback, this);
        proc_image_sub_ = nh_->subscribe("/proc_image_processing/markers",100, &ProcMappingNode::MarkersCallback, this);
        objective_reset_srv_ = nh_->advertiseService("/proc_mapping/objective_reset/", &ProcMappingNode::ObjectiveResetCallback, this);

        pingerLocationPublisher = nh_->advertise<PingerLocation>("/proc_mapping/pinger_location", 100);

        bool debug;

        if (nh_->getParam("/proc_mapping/debug", debug) && debug)
        {
            this->debug = new Debug(nh_, buoys_, fence_, pinger_);
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
    void ProcMappingNode::Spin() {
      ros::Rate r(15);  // 15 hz
            //int id=0;

        auto previousStamp = ros::Time();

      while (ros::ok()) {
        ros::spinOnce();

          markers_pub_.publish(markers);

        if (debug)
        {
            debug->sendDebugData();
        }

          if ((ros::Time() - previousStamp).sec >= 10)
          {

              auto point = pingObjective.getPoint();

                PingerLocationPtr pingerLocation(new PingerLocation());


              pingerLocation->point = *point;
                pingerLocation->frequency = 40;
              //PingPose pingPose;

              pingerLocationPublisher.publish(pingerLocation);

            previousStamp = ros::Time();
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

            marker.ns = std::to_string(marker.id);

            switch (marker.type)
            {
                case visualization_msgs::Marker::SPHERE:
                    buoysMarkers.push_back(marker);
                    break;
                case visualization_msgs::Marker::CUBE:
                    fenceMarkers.push_back(marker);
                    break;
                case visualization_msgs::Marker::CYLINDER:
                    pingerMarkers.push_back(marker);

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
            buoys_->addMarkers(buoysMarkers);
            //buoys_.getObjectives();
        }
        if (!fenceMarkers.empty())
        {
            fence_->addMarkers(fenceMarkers);
            //fence_.getObjectives();
        }
        if (!pingerMarkers.empty())
        {
            pinger_->addMarkers(pingerMarkers);
            //pinger_.getObjectives();
        }

    }

    void ProcMappingNode::GlobalMappingRequestCallback(const proc_mapping::GlobalMappingRequest::ConstPtr &request)
    {

        proc_mapping::GlobalMappingResponsePtr response(new proc_mapping::GlobalMappingResponse());

        response->request = *request;

        switch (request->object_type) {
            case proc_mapping::GlobalMappingRequest::BUOY:
                response->point = *buoys_->getGlobalMapping();
                break;

            case proc_mapping::GlobalMappingRequest::PINGER:
                response->point = *pinger_->getGlobalMapping();
                break;

            case proc_mapping::GlobalMappingRequest::FENCE:
                response->point = *fence_->getGlobalMapping();
                break;

            default:
                // TODO Handle with logs
                break;
        }

        global_mapping_response_pub_.publish(response);

    }

    void ProcMappingNode::LocalMappingRequestCallback(const proc_mapping::LocalMappingRequest::ConstPtr &request)
    {
        proc_mapping::LocalMappingResponsePtr response(new proc_mapping::LocalMappingResponse());

        response->request = *request;

        switch (request->object_type) {
            case proc_mapping::LocalMappingRequest::BUOY:
                response->point = *buoys_->getLocalMapping(request->color);
                break;

            case proc_mapping::LocalMappingRequest::PINGER:
                response->point = *pinger_->getLocalMapping(request->color);
                break;

            case proc_mapping::LocalMappingRequest::FENCE:
                response->point = *fence_->getLocalMapping(request->color);
                break;

            default:
                // TODO Handle with logs
                break;
        }

        local_mapping_response_pub_.publish(response);
    }

    bool ProcMappingNode::ObjectiveResetCallback(proc_mapping::ObjectiveReset::Request &request,
                                                     proc_mapping::ObjectiveReset::Response &response) {


        switch (request.objectiveType)
        {
            case proc_mapping::ObjectiveReset::Request::ALL:
                buoys_->reset();
                fence_->reset();
                pinger_->reset();
                break;

            case proc_mapping::ObjectiveReset::Request::BUOY:
                buoys_->reset();
                break;

            case proc_mapping::ObjectiveReset::Request::FENCE:
                fence_->reset();
                break;

            case proc_mapping::ObjectiveReset::Request::PINGER:
                pinger_->reset();
                break;

            default:
                ROS_WARN("Invalid value for ObjectiveReset::Request::objectiveType");
                break;

        }

        return true;

    }

    void ProcMappingNode::PingsCallback(const proc_hydrophone::PingPoseConstPtr &ping) {

        //std::cout << "Ping callback" << std::endl;

        pingObjective.addPing(ping);

    }

}  // namespace proc_mapping
