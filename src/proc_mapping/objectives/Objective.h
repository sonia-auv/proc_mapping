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

        using Ptr = std::shared_ptr<Objective>;

        Objective(std::string id, uint8_t nbObjects);
        ~Objective();


        // Add markers to the list of markers
        void addMarkers(std::vector<visualization_msgs::Marker> markers);

        std::vector<visualization_msgs::Marker> getObjectives();
        std::vector<visualization_msgs::Marker> getAllMarkers();


        void setNbObjects(uint8_t nbObjects);
        uint8_t getNbObjects();

        std::string getId();

        void reset();

    private:

        //int idTest=0;

        // x, y and z
        static const arma::uword NB_ROWS = 3;

        // Id of the objective (its name)
        const std::string id;

        // Nb of clusters (objective item)
        uint8_t nbObjects;

        // All markers
        std::vector<visualization_msgs::Marker> markers;

        // Markers that have to be added to kmean_mat
        std::vector<visualization_msgs::Marker> untreatedMarkers;

        // KMean algorithm matrix
        arma::mat kmean_mat;

        // The centroids matrix
        arma::mat centroids;

        // The centroids list
        std::vector<visualization_msgs::Marker> centroidsList;

        // Resize the kmean matrix with new markers
        void resizeKmeanMatrix();

        // Add marker at the end of the matrix
        void addUntreadedMarkersToMatrix();

        // Run KMean algorithm
        bool runAlgorithm();

        void fillCentroidsList();

        void printCentroids();

    };
}




#endif //PROC_MAPPING_OBJECTIVE_H
