//
// Created by coumarc9 on 5/26/17.
//

#include "proc_mapping/position.h"

namespace proc_mapping
{
    Position::Position(const ros::NodeHandlePtr &nh) :
        nh_(nh),
        save_position_sub_(nh_->subscribe("/proc_mapping/SavePose", 100, &Position::SavePositionCallback, this)),
        saved_position_request_sub_(nh_->subscribe("/proc_mapping/GetSavedPoseRequest", 100, &Position::GetSavedPoseRequestCallback, this)),
        saved_position_response_pub_(nh_->advertise<KeyValueIdPose>("/proc_mapping/GetSavedPoseResponse", 100))
    {
    }

    void Position::SavePositionCallback(const proc_mapping::KeyValueIdPose::ConstPtr &request)
    {

        ROS_DEBUG("SavePosition called ");

        if (positions_map.count(request->id) == 0)
        {
            ROS_INFO("Position id doesn't exist yet");
        } else
        {
            geometry_msgs::Pose position = positions_map[request->id];

            ROS_INFO("Position id already exist. Saved position : {id = %d, p.x = %f, p.y = %f, p.z = %f, o.x = %f, o.y = %f, o.z = %f,o.w = %f}",
                     request->id, position.position.x, position.position.y, position.position.z,
                     position.orientation.x, position.orientation.y, position.orientation.z,
                     position.orientation.w);


        }

        positions_map[request->id] = request->pose;

        ROS_INFO("Position saved");

        geometry_msgs::Pose position = positions_map[request->id];

        ROS_INFO("Saved position : {id = %d, p.x = %f, p.y = %f, p.z = %f, o.x = %f, o.y = %f, o.z = %f,o.w = %f}",
                 request->id, position.position.x, position.position.y, position.position.z,
                 position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w);

    }

    void Position::GetSavedPoseRequestCallback(const proc_mapping::PoseSavedRequest::ConstPtr &request)
    {
        ROS_DEBUG("SavedPoseRequest called with id : %d", request->id);

        geometry_msgs::Pose position;

        if (positions_map.count(request->id) == 1)
        {
            ROS_INFO("Position id exist. Saved position : {id = %d, p.x = %f, p.y = %f, p.z = %f, o.x = %f, o.y = %f, o.z = %f,o.w = %f}",
                     request->id, positions_map[request->id].position.x, positions_map[request->id].position.y, positions_map[request->id].position.z,
                     positions_map[request->id].orientation.x, positions_map[request->id].orientation.y, positions_map[request->id].orientation.z,
                     positions_map[request->id].orientation.w);

            position = positions_map[request->id];

        }
        else
        {
            ROS_WARN("Position id doesn't exist. Default position will be send");
        }

        KeyValueIdPose response;

        response.id = request->id;
        response.pose = position;

        this->saved_position_response_pub_.publish(response);

    }


}
