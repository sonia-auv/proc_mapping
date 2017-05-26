//
// Created by coumarc9 on 5/26/17.
//

#include "debug.h"

namespace proc_mapping
{
    Debug::Debug(const ros::NodeHandlePtr &nh, Objective::Ptr &buoys, Objective &fence, Objective &pinger) :
        nh_(nh),
        all_buoys_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/all_buoys", 100)),
        all_fences_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/all_fences", 100)),
        all_pingers_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/all_pingers", 100)),
        buoys_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/buoys", 100)),
        fence_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/fence", 100)),
        pinger_pub_(nh_->advertise<visualization_msgs::MarkerArray>("/proc_mapping/debug/pinger", 100)),
        buoys_(buoys),
        fence_(fence),
        pinger_(pinger)
    {
        // TODO Clean class
    }

    void Debug::sendDebugData()
    {
        sendBuoysMarkers();
        //sendDebugMarkers(buoys_, all_buoys_pub_, buoys_pub_);

    }

    void Debug::sendBuoysMarkers()
    {
        visualization_msgs::MarkerArray markers;

        for (visualization_msgs::Marker marker : buoys_->getObjectives())
        {
            marker.header.frame_id = "NED";
            marker.header.stamp = ros::Time::now();

            marker.ns = std::to_string(marker.id);

            marker.id = 0;
            marker.action = visualization_msgs::Marker::ADD;

            marker.color.a = 1;
            marker.color.g = 1;

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.lifetime = ros::Duration();

            marker.type = visualization_msgs::Marker::SPHERE;

            markers.markers.push_back(marker);
        }

        buoys_pub_.publish(markers);

        visualization_msgs::MarkerArray allMarkers;

        for (visualization_msgs::Marker marker : buoys_->getAllMarkers())
        {
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

            marker.type = visualization_msgs::Marker::SPHERE;

            allMarkers.markers.push_back(marker);
        }

        all_buoys_pub_.publish(allMarkers);
    }

//    void Debug::sendDebugMarkers(Objective objective, ros::Publisher all_publisher, ros::Publisher publisher)
//    {
//        visualization_msgs::MarkerArray markers;
//
//        for (visualization_msgs::Marker marker : objective.getObjectives())
//        {
//            markers.markers.push_back(marker);
//        }
//
//        publisher.publish(markers);
//
//        visualization_msgs::MarkerArray allMarkers;
//
//        for (visualization_msgs::Marker marker : objective.getAllMarkers())
//        {
//            markers.markers.push_back(marker);
//        }
//
//        publisher.publish(markers);
//
//    }

}