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

        // TODO Remove std:cout
        // TODO If total makers < nbObjects, then resize

        std::cout << "Markers size : " << markers.size() << std::endl;

        centroids.print("Centroids");

        if(markers.empty()) return;

        long long int nbCol = kmean_mat.n_cols;

        std::cout << "Initial nbCol : " << nbCol << std::endl;

        std::cout << "NB_ROWS : " << NB_ROWS << std::endl;

        kmean_mat.print("Initial Matrix");

        kmean_mat.resize(NB_ROWS, nbCol + markers.size());

        kmean_mat.print("Reshaped Matrix");

        int i = 0;

        for(auto marker : markers)
        {
            this->markers.push_back(marker);

            long long int noCol = nbCol + i;


            kmean_mat(0,noCol) = marker.pose.position.x;
            kmean_mat(1,noCol) = marker.pose.position.y;
            kmean_mat(2,noCol) = marker.pose.position.z;

            kmean_mat.print("After set column");

            i++;

        }

        kmean_mat.print("Final matrix");


        centroids.print("Centroids before treatment");

        // TODO Check for better
        if (kmean_mat.n_cols >= nbObjects)
        {
            std::cout << "Begin Clustering" << std::endl;

            mlpack::kmeans::KMeans<> kmeans;

            // true => initial guess for centroids
            kmeans.Cluster(kmean_mat, nbObjects, centroids, true);

            std::cout << "End Clustering" << std::endl;

            kmean_mat.print("Matrix after clusters");

            centroids.print("Centroids after clusters");
        }
        else
        {
            std::cout << "Not enough object to run algorithm" << std::endl;
        }

    }

    void Objective::setNbObjects(uint8_t nbObjects) {
        this->nbObjects = nbObjects;
    }

    uint8_t Objective::getNbObjects() {
        return nbObjects;
    }
}