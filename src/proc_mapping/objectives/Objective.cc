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
    }

    Objective::~Objective() {

    }

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
            this->markers.push_back(marker);
        }

        ROS_INFO("Markers added to list. Markers are ready to be treated");

        ROS_INFO("End adding markers for %s", id.data());

    }

    geometry_msgs::PointConstPtr Objective::getGlobalMapping()
    {

        ROS_INFO("Creating GlobalMapping for %s", id.data());

        // The centroids matrix
        arma::mat centroids(NB_ROWS, 1, arma::fill::zeros);

        mlpack::kmeans::KMeans<> kmeans;

        try {
            kmeans.Cluster(this->centroids, 1, centroids);

            geometry_msgs::PointPtr point(new geometry_msgs::Point);

            point->x = centroids(0,0);
            point->y = centroids(1,0);
            point->z = centroids(2,0);

            ROS_DEBUG("Returning GlobalMapping point { x = %f, y = %f, z = %f", point->x, point->y, point->z);

            return point;

        } catch (...) {
            ROS_ERROR("An error occur while trying to generate GlobalMapping for %s", id.data());
        }

        ROS_INFO("Returning default value for Point");
        return geometry_msgs::PointConstPtr();
    }

    geometry_msgs::PointConstPtr Objective::getLocalMapping(std_msgs::ColorRGBA color)
    {

        // TODO Change point to return the great one

        for (auto marker : centroidsList)
        {
            bool nearBlue = marker.color.b < color.b + blueUpperTolerance && marker.color.b > color.b - blueLowerTolerance;
            bool nearRed = marker.color.r < color.r + redUpperTolerance && marker.color.r > color.r - redLowerTolerance;
            bool nearGreen = marker.color.g < color.g + greenUpperTolerance && marker.color.g > color.g - greenLowerTolerance;

            if (nearBlue && nearRed && nearGreen)
            {
                ROS_DEBUG("Returning LocalMapping point { x = %f, y = %f, z = %f", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                geometry_msgs::PointPtr point(new geometry_msgs::Point);

                point->x = marker.pose.position.x;
                point->y = marker.pose.position.y;
                point->z = marker.pose.position.z;

                return point;

            }

        }

        // Sending first centroid (didnt find the right color)
        geometry_msgs::PointPtr point(new geometry_msgs::Point);

        point->x = centroids(0,0);
        point->y = centroids(1,0);
        point->z = centroids(2,0);

        ROS_DEBUG("Returning first centroids point");
        ROS_DEBUG("Returning LocalMapping point { x = %f, y = %f, z = %f", point->x, point->y, point->z);

        return point;
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

    std::vector<visualization_msgs::Marker> Objective::getAllMarkers()
    {
        return this->markers;
    }

    void Objective::reset() {

        ROS_DEBUG("Reset markers in objective %s", id.data());

        markers.clear();
        untreatedMarkers.clear();

        kmean_mat.resize(0,0);

    }

    void Objective::fillCentroidsList()
    {
        centroidsList.clear();

        for (unsigned int i = 0; i < centroids.n_cols; ++i) {

            visualization_msgs::Marker marker;

            marker.pose.position.x  = centroids(0,i);
            marker.pose.position.y  = centroids(1,i);
            marker.pose.position.z  = centroids(2,i);

            marker.id = i;

            std_msgs::ColorRGBA color;

            unsigned int coloredMarker = 0;

            for (auto j = 0; j < assignments.size() ; j++)
            {

                auto assignment = assignments[j];

                if (assignment != i)
                    continue;

                auto associatedMaker = markers.at(j);

                if (associatedMaker.color.b != 0 || associatedMaker.color.g != 0 || associatedMaker.color.r != 0)
                {
                    color.g += associatedMaker.color.b;
                    color.b += associatedMaker.color.b;
                    color.r += associatedMaker.color.r;
                    coloredMarker++;
                }

            }

            if (coloredMarker != 0)
            {
                color.b /= coloredMarker;
                color.g /= coloredMarker;
                color.r /= coloredMarker;
            }


            color.a = 1;

            marker.color = color;

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
            kmeans.Cluster(kmean_mat, nbObjects, assignments, centroids, false, true);

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