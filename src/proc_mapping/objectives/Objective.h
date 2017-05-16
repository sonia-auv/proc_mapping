//
// Created by coumarc9 on 5/13/17.
//

#ifndef PROC_MAPPING_OBJECTIVE_H
#define PROC_MAPPING_OBJECTIVE_H

#include "visualization_msgs/Marker.h"
#include "mlpack/methods/kmeans/kmeans.hpp"
#include "armadillo"

#include <ros/ros.h>

namespace proc_mapping
{
    class Objective {
    public:

        Objective(uint8_t nbObjects);
        ~Objective();


        // Add markers to the list of markers
        void addMarkers(std::vector<visualization_msgs::Marker> markers);

        void setNbObjects(uint8_t nbObjects);
        uint8_t getNbObjects();

    private:

        std::vector<visualization_msgs::Marker> markers;
        uint8_t nbObjects;

        arma::mat kmean_mat;

        // The centroids will be stored in this matrix.
        arma::mat centroids;

        int const NB_ROW = 3;

    };
}




#endif //PROC_MAPPING_OBJECTIVE_H
