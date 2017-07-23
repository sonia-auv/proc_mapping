/**
 * \file	proc_mapping_node.h
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

#ifndef PROC_MAPPING_PROC_MAPPING_NODE_H_
#define PROC_MAPPING_PROC_MAPPING_NODE_H_

#include <ros/node_handle.h>
#include <memory>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <proc_hydrophone/PingPose.h>


#include "proc_mapping/objectives/Objective.h"
#include <proc_mapping/objectives/HydroObjective.h>

#include "proc_mapping/GlobalMappingRequest.h"
#include "proc_mapping/GlobalMappingResponse.h"
#include "proc_mapping/LocalMappingRequest.h"
#include "proc_mapping/LocalMappingResponse.h"

#include "proc_mapping/ObjectiveReset.h"
#include "proc_mapping/position.h"
#include "proc_mapping/debug.h"

namespace proc_mapping {

class ProcMappingNode {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ProcMappingNode(const ros::NodeHandlePtr &nh);

  ~ProcMappingNode();

  /// Taking care of the spinning of the ROS thread.
  /// Each iteration of the loop, this will take the objects in the object
  /// registery, empty it and publish the objects.
  void Spin();
  //  void ResetMapCallback(const sonia_msgs::ResetMap::ConstPtr &msg);



 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

    ros::NodeHandlePtr nh_;

    ros::Publisher map_pub_;
    ros::Publisher markers_pub_;

    ros::Subscriber global_mapping_request_sub_;
    ros::Subscriber local_mapping_request_sub_;
    ros::Publisher global_mapping_response_pub_;
    ros::Publisher local_mapping_response_pub_;

    ros::Subscriber reset_map_sub_;

    ros::ServiceServer objective_reset_srv_;

    ros::Subscriber hydro_sub_;
    ros::Subscriber proc_image_sub_;

    visualization_msgs::MarkerArray markers;

    const Position position_;

    Objective::Ptr buoys_;
    Objective::Ptr fence_;
    Objective::Ptr pinger_;

    HydroObjective pingObjective;

    void PingsCallback(const proc_hydrophone::PingPoseConstPtr &ping);

    void MarkersCallback(const visualization_msgs::MarkerArray::ConstPtr &markers);
    void GlobalMappingRequestCallback(const proc_mapping::GlobalMappingRequest::ConstPtr &request);
    void LocalMappingRequestCallback(const proc_mapping::LocalMappingRequest::ConstPtr &request);
    //void MappingRequestCallback(const proc_mapping::MappingRequest::ConstPtr &request);
    bool ObjectiveResetCallback(proc_mapping::ObjectiveReset::Request &request,
                                    proc_mapping::ObjectiveReset::Response &response);

    Debug * debug = 0;

};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_MAPPING_NODE_H_
