//
// Created by coumarc9 on 5/13/17.
//

#include "Objective.h"

namespace proc_mapping
{

    Objective::Objective(uint8_t nbObjects)
        : nbObjects(nbObjects)
    {


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