//
// Created by coumarc9 on 5/26/17.
//

#include "debug.h"

namespace proc_mapping
{
    Debug::Debug(const ros::NodeHandlePtr &nh) :
        nh_(nh),
        all_buoys_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/all_buoys", 100)),
        all_fences_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/all_fences", 100)),
        all_pingers_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/all_pingers", 100)),
        buoys_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/buoys", 100)),
        fence_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/fence", 100)),
        pinger_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/pinger", 100))
    {

    }

    void Debug::sendDebugData()
    {

    }

}