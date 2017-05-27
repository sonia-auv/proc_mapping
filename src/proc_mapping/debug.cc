//
// Created by coumarc9 on 5/26/17.
//

#include "debug.h"

namespace proc_mapping
{
    Debug::Debug(const ros::NodeHandlePtr &nh, Objective::Ptr &buoys, Objective::Ptr &fence, Objective::Ptr &pinger) :
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
    }

    void Debug::sendDebugData()
    {
        sendBuoysMarkers();
        sendFenceMarkers();
        sendPingerMarkers();
    }

    void Debug::sendBuoysMarkers()
    {
        visualization_msgs::MarkerArray markers;

        for (visualization_msgs::Marker marker : buoys_->getObjectives())
        {
            setupMarker(marker);

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.g = 1;

            marker.type = visualization_msgs::Marker::SPHERE;

            markers.markers.push_back(marker);
        }

        buoys_pub_.publish(markers);

        visualization_msgs::MarkerArray allMarkers;

        for (visualization_msgs::Marker marker : buoys_->getAllMarkers())
        {
            setupMarker(marker);

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.b = 1;

            marker.type = visualization_msgs::Marker::SPHERE;

            allMarkers.markers.push_back(marker);
        }

        all_buoys_pub_.publish(allMarkers);
    }

    void Debug::sendFenceMarkers()
    {
        visualization_msgs::MarkerArray markers;

        for (visualization_msgs::Marker marker : fence_->getObjectives())
        {
            setupMarker(marker);

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.b = 0.5;
            marker.color.g = 0.5;

            marker.type = visualization_msgs::Marker::CUBE;

            markers.markers.push_back(marker);
        }

        fence_pub_.publish(markers);

        visualization_msgs::MarkerArray allMarkers;

        for (visualization_msgs::Marker marker : fence_->getAllMarkers())
        {
            setupMarker(marker);

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.b = 0.5;
            marker.color.r = 0.5;

            marker.type = visualization_msgs::Marker::CUBE;

            allMarkers.markers.push_back(marker);
        }

        all_fences_pub_.publish(allMarkers);
    }

    void Debug::sendPingerMarkers()
    {
        visualization_msgs::MarkerArray markers;

        for (visualization_msgs::Marker marker : pinger_->getObjectives())
        {
            setupMarker(marker);

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.r = 0.5;
            marker.color.g = 0.5;

            marker.type = visualization_msgs::Marker::CYLINDER;

            markers.markers.push_back(marker);
        }

        pinger_pub_.publish(markers);

        visualization_msgs::MarkerArray allMarkers;

        for (visualization_msgs::Marker marker : pinger_->getAllMarkers())
        {
            setupMarker(marker);

            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.b = 1;
            marker.color.r = 1;
            marker.color.g = 1;

            marker.type = visualization_msgs::Marker::CYLINDER;

            allMarkers.markers.push_back(marker);
        }

        all_pingers_pub_.publish(allMarkers);
    }

    void Debug::setupMarker(visualization_msgs::Marker &marker) {

        marker.header.frame_id = "NED";
        marker.header.stamp = ros::Time::now();

        marker.ns = std::to_string(marker.id);

        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;

        marker.color.a = 1;

        marker.lifetime = ros::Duration();

    }

}