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
#include <functional>
#include "proc_mapping/config.h"

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
          submarine_position_(),
          buoys_(3)
    {


        markers_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/markers", 100);

        hydro_sub_ = nh_->subscribe("/provider_hydrophone/markers", 100, &ProcMappingNode::MarkersCallback, this);

        proc_image_sub_ = nh_->subscribe("/proc_image_processing/markers",100, &ProcMappingNode::MarkersCallback, this);

    }

    //------------------------------------------------------------------------------
    //
    ProcMappingNode::~ProcMappingNode() {}

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

        r.sleep();
      }
    }


    void ProcMappingNode::MarkersCallback(const visualization_msgs::MarkerArray::ConstPtr &markers) {

        //visualization_msgs::Marker marker;

        //marker.header.frame_id = "NED";
        //marker.header.stamp = ros::Time::now();
        //marker.type = visualization_msgs::Marker::SPHERE;

        //marker.id = 0;
        //marker.action = visualization_msgs::Marker::ADD;

        //marker.pose.position.x=0;
        //marker.pose.position.y=0;
        //marker.pose.position.z=0;


        //marker.pose.orientation.x = 0.0;
        //marker.pose.orientation.y = 0.0;
        //marker.pose.orientation.z = 0.0;
        //marker.pose.orientation.w = 1.0;

        //marker.scale.x = 1.0;
        //marker.scale.y = 1.0;
        //marker.scale.z = 1.0;


        //marker.color.r = 1.0f;
        //marker.color.g = 1.0f;
        //marker.color.b = 0.0f;
        //marker.color.a = 1.0;

        //marker.lifetime = ros::Duration();

        std::vector<visualization_msgs::Marker> markersObjectives;

        for (unsigned int i = 0; i < markers->markers.size(); i++) {

            // TODO Check marker type (buoy, fens, ...) then add it to the objective



            auto marker = markers->markers[i];

            marker.header.frame_id = "NED";
            marker.header.stamp = ros::Time::now();

            marker.ns = std::to_string(marker.id);

            marker.id = 0;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.lifetime = ros::Duration();

            this->markers.markers.push_back(marker);

            markersObjectives.push_back(marker);

        }

        buoys_.addMarkers(markersObjectives);


    }

}  // namespace proc_mapping
