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
#include <sonia_msgs/PingPose.h>
#include <proc_mapping/objectives/HydroObjective.h>
#include "proc_mapping/ObjectiveReset.h"
#include <proc_mapping/PingerLocationService.h>
#include <nav_msgs/Odometry.h>

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

    ros::Subscriber reset_map_sub_;
    ros::Subscriber hydro_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher pingerLocationPublisher;
    ros::Publisher pingerLocationDebugPublisher;

    ros::ServiceServer objective_reset_srv_;
    ros::ServiceServer pingerLocationService;

    std::map<uint8_t , HydroObjective> hydroObjectives_;

//    HydroObjective pingObjective25khz_;
//    HydroObjective pingObjective30khz_;
//    HydroObjective pingObjective35khz_;
//    HydroObjective pingObjective40khz_;

    void PingsCallback(const sonia_msgs::PingPoseConstPtr &ping);

    bool ObjectiveResetCallback(proc_mapping::ObjectiveReset::Request &request,
                                    proc_mapping::ObjectiveReset::Response &response);
    bool PingerLocationServiceCallback(proc_mapping::PingerLocationService::Request &request,
                                proc_mapping::PingerLocationService::Response &response);

    void OdomCallback(const nav_msgs::OdometryConstPtr &odom);

    static const double_t distanceDefaultValue_;// = 1;
    static const std::string distanceParamName_;// = "/proc_mapping/hydro/distance";


};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_MAPPING_NODE_H_
