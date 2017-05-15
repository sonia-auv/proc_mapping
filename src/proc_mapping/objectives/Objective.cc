//
// Created by coumarc9 on 5/13/17.
//

#include "Objective.h"

namespace proc_mapping
{

    Objective::Objective(uint8_t nbObjects)
        : nbObjects(nbObjects)
    {

        //arma::mat data;

        arma::mat data;
// The number of clusters we are getting.
        size_t clusters;
// The assignments will be stored in this vector.
        arma::Row<size_t> assignments;
// The centroids will be stored in this matrix.
        arma::mat centroids;


        // TODO Décommenter la ligne suivante amène l'erreur
        //kmeans.Cluster(data, clusters, assignments, centroids);

    }

    Objective::~Objective() {

    }

    void Objective::addMarkers(std::vector<visualization_msgs::Marker> markers) {

        for(auto marker : markers)
        {
            this->markers.push_back(marker);

            // TODO Add to matrix list for traitment
            //std::iterator<visualization_msgs::Marker, void> test = this->markers;
        }

    }

    void Objective::setNbObjects(uint8_t nbObjects) {
        this->nbObjects = nbObjects;
    }

    uint8_t Objective::getNbObjects() {
        return nbObjects;
    }
}