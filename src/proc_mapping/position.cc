//
// Created by coumarc9 on 5/26/17.
//

#include "proc_mapping/position.h"

namespace proc_mapping
{
    Position::Position(const ros::NodeHandlePtr &nh) :
        nh_(nh)
    {
        // TODO Check to use initialization
        this->save_position_sub_ = nh_->subscribe("/proc_mapping/SavePose", 100, &Position::SavePositionCallback, this);



    }

    void Position::SavePositionCallback(const proc_mapping::KeyValueIdPose::ConstPtr &request)
    {

        if (positions_map.count(request->id) == 0)
        {
            ROS_INFO("Position id doesn't exist yet");
        } else
        {

            ROS_INFO("Position id already exist. Saved position : {id = %d, p.x = %f, p.y = %f, p.z = %f, o.x = %f, o.y = %f, o.z = %f,o.w = %f}",
                     request->id, positions_map[request->id].position.x, positions_map[request->id].position.y, positions_map[request->id].position.z,
                     positions_map[request->id].orientation.x, positions_map[request->id].orientation.y, positions_map[request->id].orientation.z,
                     positions_map[request->id].orientation.w);


        }

        positions_map[request->id] = request->pose;

        ROS_INFO("Position saved");

        geometry_msgs::Pose position = positions_map[request->id];

        ROS_INFO("Saved position : {id = %d, p.x = %f, p.y = %f, p.z = %f, o.x = %f, o.y = %f, o.z = %f,o.w = %f}",
                 request->id, position.position.x, position.position.y, position.position.z,
                 position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w);

    }

}
