//
// Created by coumarc9 on 5/13/17.
//

#include "Objective.h"

namespace proc_mapping
{

    Objective::Objective(std::string id, uint8_t nbObjects)
        : id(id),
          nbObjects(nbObjects),
          untreatedMarkers(),
          kmean_mat(),
          centroids(NB_ROWS, nbObjects, arma::fill::zeros),
          centroidsList()

    {
        // TODO Log using ROS_LOG_STREAM for matrix
    }

    Objective::~Objective() {}

    void Objective::addMarkers(std::vector<visualization_msgs::Marker> markers) {

        if(markers.empty())
        {
            ROS_INFO("No markers received in %s objective", id.data());
            return;
        }

        ROS_INFO("Start adding markers for %s", id.data());

        ROS_INFO("Markers received size : %lu", markers.size());

        for (auto marker : markers)
        {
            untreatedMarkers.push_back(marker);
        }

        ROS_INFO("Markers added to list. Markers are ready to be treated");

        ROS_INFO("End adding markers for %s", id.data());

    }

    std::vector<visualization_msgs::Marker> Objective::getObjectives()
    {
        if (untreatedMarkers.empty())
        {
            ROS_INFO("No markers have been added to the untreatedMarkers list, return same centroids as last run");
            return centroidsList;
        }

        resizeKmeanMatrix();

        addUntreadedMarkersToMatrix();

        // If algorithm failed
        if (!runAlgorithm())
            return std::vector<visualization_msgs::Marker>(); // return empty vector

        fillCentroidsList();

        return centroidsList;

    }

    void Objective::fillCentroidsList()
    {
        centroidsList.clear();

        for (unsigned int i = 0; i < centroids.n_cols; ++i) {

            visualization_msgs::Marker marker;

            marker.pose.position.x  = centroids(0,i);
            marker.pose.position.y  = centroids(1,i);
            marker.pose.position.z  = centroids(2,i);

            centroidsList.push_back(marker);

        }

    }

    void Objective::addUntreadedMarkersToMatrix()
    {
        int i = kmean_mat.n_cols - untreatedMarkers.size();

        for(auto marker : untreatedMarkers)
        {
            //this->markers.push_back(marker);

            kmean_mat(0,i) = marker.pose.position.x;
            kmean_mat(1,i) = marker.pose.position.y;
            kmean_mat(2,i) = marker.pose.position.z;

            i++;

        }

        untreatedMarkers.clear();

    }

    void Objective::setNbObjects(uint8_t nbObjects) {
        this->nbObjects = nbObjects;
        // TODO Modify numbers of clusters
    }

    uint8_t Objective::getNbObjects() {
        return nbObjects;
    }

    std::string Objective::getId()
    {
        return this->id;
    }

    void Objective::resizeKmeanMatrix()
    {
        arma::uword nbCol = kmean_mat.n_cols;

        ROS_DEBUG("Initial nbCol : %lld", nbCol);

        kmean_mat.resize(NB_ROWS, nbCol + untreatedMarkers.size());

        nbCol = kmean_mat.n_cols;

        ROS_DEBUG("New nbCol : %lld", nbCol);
    }

    bool Objective::runAlgorithm()
    {


        if (kmean_mat.n_cols < nbObjects)
        {
            ROS_INFO("Not enough object to run algorithm");
            return false;
        }

        mlpack::kmeans::KMeans<> kmeans;

        try {
            ROS_DEBUG("Begin clustering");

            // true => initial guess for centroids
            kmeans.Cluster(kmean_mat, nbObjects, centroids, true);
        } catch (...) {
            ROS_ERROR("An error occured when trying to run kmean algorithm");
            ROS_DEBUG("End Clustering");
            return false;
        }

        ROS_DEBUG("End Clustering");

        return true;

    }

    void Objective::printCentroids()
    {

        ROS_INFO("Beginning of centroids");

        for (int j = 0; j < this->nbObjects; ++j) {

            ROS_INFO("Centroid #%d, { x = %f, y = %f, z = %f}", j, centroids(0,j), centroids(1,j), centroids(2,j));

        }

        ROS_INFO("End of centroids");

    }

}