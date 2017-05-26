//
// Created by coumarc9 on 5/26/17.
//

#ifndef PROC_MAPPING_DEBUG_H
#define PROC_MAPPING_DEBUG_H

#include <ros/node_handle.h>
#include "visualization_msgs/MarkerArray.h"

#include "proc_mapping/objectives/Objective.h"

namespace proc_mapping
{
    class Debug {
    public:
        Debug(const ros::NodeHandlePtr &nh, Objective::Ptr &buoys, Objective &fence, Objective &pinger);

        void sendDebugData();

    private:
        const ros::NodeHandlePtr nh_;
        const ros::Publisher all_buoys_pub_;
        const ros::Publisher all_fences_pub_;
        const ros::Publisher all_pingers_pub_;
        const ros::Publisher buoys_pub_;
        const ros::Publisher fence_pub_;
        const ros::Publisher pinger_pub_;

        Objective::Ptr buoys_;
        Objective fence_;
        Objective pinger_;

        //void static sendDebugMarkers(Objective objective, ros::Publisher all_publisher, ros::Publisher publisher);

        void sendBuoysMarkers();

    };
}



#endif //PROC_MAPPING_DEBUG_H
