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

        Objective(std::string id, uint8_t nbObjects);
        ~Objective();


        // Add markers to the list of markers
        void addMarkers(std::vector<visualization_msgs::Marker> markers);

        void setNbObjects(uint8_t nbObjects);
        uint8_t getNbObjects();

    private:

        std::vector<visualization_msgs::Marker> markers;

        const std::string id;
        uint8_t nbObjects;

        arma::mat kmean_mat;

        // The centroids will be stored in this matrix.
        arma::mat centroids;

        static const arma::uword NB_ROWS = 3;

    };
}




#endif //PROC_MAPPING_OBJECTIVE_H
