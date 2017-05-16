//
// Created by coumarc9 on 5/13/17.
//

#include "Objective.h"

namespace proc_mapping
{

    Objective::Objective(uint8_t nbObjects)
        : nbObjects(nbObjects),
          kmean_mat(),
          centroids(NB_ROW, nbObjects, arma::fill::zeros)
    {

        // TODO Temporaire
        this->nbObjects = 1;
        centroids = arma::mat(NB_ROW, this->nbObjects);

        // TODO Décommenter la ligne suivante amène l'erreur
        //kmeans.Cluster(kmean_mat, this->nbObjects, assignments, centroids);

    }

    Objective::~Objective() {

    }

    void Objective::addMarkers(std::vector<visualization_msgs::Marker> markers) {

        // TODO Remove std:cout
        // TODO If total makers < nbObjects, then resize

        std::cout << "Markers size : " << markers.size() << std::endl;

        centroids.print("Centroids");

        if(markers.empty()) return;

        long long int nbCol = kmean_mat.n_cols;

        std::cout << "Initial nbCol : " << nbCol << std::endl;

        std::cout << "NB_ROW : " << NB_ROW << std::endl;

        kmean_mat.print("Initial Matrix");

        kmean_mat.resize(NB_ROW, nbCol + markers.size());

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

        std::cout << "Begin Clustering" << std::endl;

        mlpack::kmeans::KMeans<> kmeans;

        // true => initial guess for centroids
        kmeans.Cluster(kmean_mat, nbObjects, centroids, true);

        std::cout << "End Clustering" << std::endl;

        kmean_mat.print("Matrix after clusters");

        centroids.print("Centroids after clusters");

    }

    void Objective::setNbObjects(uint8_t nbObjects) {
        this->nbObjects = nbObjects;
    }

    uint8_t Objective::getNbObjects() {
        return nbObjects;
    }
}