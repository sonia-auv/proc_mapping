//
// Created by coumarc9 on 5/26/17.
//

#ifndef PROC_MAPPING_POSITION_H
#define PROC_MAPPING_POSITION_H

#include <ros/node_handle.h>

#include "proc_mapping/KeyValueIdPose.h"
#include "proc_mapping/PoseSavedRequest.h"

#include "geometry_msgs/Pose.h"

namespace proc_mapping
{
    class Position {

    public:

        Position(const ros::NodeHandlePtr &nh);

    private:

        const ros::NodeHandlePtr nh_;

        // Subscribers
        ros::Subscriber save_position_sub_;
        const ros::Subscriber saved_position_request_sub_;


        // Publishers
        const ros::Publisher saved_position_response_pub_;


        // Callbacks
        void SavePositionCallback(const proc_mapping::KeyValueIdPose::ConstPtr &request);

        std::map<uint8_t , geometry_msgs::Pose> positions_map;

    };
}




#endif //PROC_MAPPING_POSITION_H
