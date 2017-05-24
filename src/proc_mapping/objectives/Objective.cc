//
// Created by coumarc9 on 5/13/17.
//

#include "Objective.h"

namespace proc_mapping
{

    Objective::Objective(uint8_t nbObjects)
        : nbObjects(nbObjects),
          kmean_mat(),
          centroids(NB_ROWS, nbObjects, arma::fill::zeros)
    {
    }

    Objective::~Objective() {}

    void Objective::addMarkers(std::vector<visualization_msgs::Marker> markers) {


        ROS_INFO("Markers received size : %lu", markers.size());

        if(markers.empty()) return;

        arma::uword nbCol = kmean_mat.n_cols;

        ROS_DEBUG("Initial nbCol : %lld", nbCol);

        kmean_mat.resize(NB_ROWS, nbCol + markers.size());

        int i = 0;

        for(auto marker : markers)
        {
            this->markers.push_back(marker);

            arma::uword noCol = nbCol + i;

            kmean_mat(0,noCol) = marker.pose.position.x;
            kmean_mat(1,noCol) = marker.pose.position.y;
            kmean_mat(2,noCol) = marker.pose.position.z;

            i++;

        }

        if (kmean_mat.n_cols >= nbObjects)
        {

            ROS_DEBUG("Begin clustering");


            mlpack::kmeans::KMeans<> kmeans;

            // true => initial guess for centroids
            kmeans.Cluster(kmean_mat, nbObjects, centroids, true);

            ROS_DEBUG("End Clustering");

            ROS_INFO("Beginning of centroids");

            centroids.print();

            for (int j = 0; j < this->nbObjects; ++j) {

                ROS_INFO("Centroid #%d, { x = %f, y = %f, z = %f}", j, centroids(0,j), centroids(1,j), centroids(2,j));

            }

            ROS_INFO("End of centroids");

        }
        else
        {
            ROS_INFO("Not enough object to run algorithm");
        }

    }

    void Objective::setNbObjects(uint8_t nbObjects) {
        this->nbObjects = nbObjects;
    }

    uint8_t Objective::getNbObjects() {
        return nbObjects;
    }
}