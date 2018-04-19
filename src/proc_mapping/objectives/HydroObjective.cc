//
// Created by coumarc9 on 7/21/17.
//

#include "HydroObjective.h"

namespace proc_mapping
{
    HydroObjective::HydroObjective() {

    }

    HydroObjective::~HydroObjective() {

    }

    void HydroObjective::addPing(const proc_hydrophone::PingPoseConstPtr &ping) {

        //pings.push_back(ping);

        // If we should process (for now, 1m from start ping)
        if (needProcess(ping))
        {

            // Get the position matrix
            auto positions = GetPositions();

            // Get the headings list
            auto headings = GetHeadings();

            // Get the centroid
            auto centroid = GetCentroids(positions, 1);

            // Get que right heading
            auto heading = GetHeading(headings);

            // PROCESSS, SOLVE
            auto newFunction = GetFunction(centroid, heading);

            //std::cout << "Before" << std::endl;

            // TODO Delete
            //functions.print();

            functions = join_rows(functions, newFunction);

            //std::cout << "After" << std::endl;

            // TODO Delete
            //functions.print();

            auto point = getPoint();

            // Clear pings vector
            pings.clear();

            // Add last received ping
            //pings.push_back(ping);


        }


        pings.push_back(ping);
        //auto newFunction = GetFunction(ping);

        //functions = join_rows(functions, newFunction);

        // TODO Delete
        //functions.print();

    }

    bool HydroObjective::needProcess(const proc_hydrophone::PingPoseConstPtr &ping) {

        if (pings.size() < 1)
            return false;

        auto firstPing = pings.front();

        auto firstPosition = firstPing->pose.position;
        auto lastPosition = ping->pose.position;

        auto firstPositionMatrix = arma::mat(2,1);
        firstPositionMatrix(0,0) = firstPosition.x;
        firstPositionMatrix(1,0) = firstPosition.y;

        auto lastPositionMatrix = arma::mat(2,1);
        lastPositionMatrix(0,0) = lastPosition.x;
        lastPositionMatrix(1,0) = lastPosition.y;

        double distance = norm(lastPositionMatrix - firstPositionMatrix);


        if (distance >= 1)
            return true;

        return false;
    }

    arma::mat HydroObjective::GetPositions() {

        if (pings.empty())
            return arma::mat();

        arma::mat positions(2,pings.size());

        for (unsigned int i = 0; i < pings.size(); ++i) {

            auto ping = pings[i]->pose.position;

            positions(0, i) = ping.x;
            positions(1, i) = ping.y;

        }

        return positions;

    }

    std::vector<double> HydroObjective::GetHeadings() {
        std::vector<double> headings;

        for (unsigned int i = 0; i < pings.size(); ++i) {

            headings.push_back(pings[i]->pose.orientation.z);

        }

        return headings;
    }

    arma::mat HydroObjective::GetCentroids(arma::mat matrix, unsigned int nb) {


        mlpack::kmeans::KMeans<> k;

        arma::mat result;

        k.Cluster(matrix, nb, result);

        //result.print();

        return result;

    }

    double HydroObjective::GetHeading(std::vector<double> headings) {

        if (headings.empty())
            return 0;


        double sumX = 0;
        double sumY = 0;

        // TODO Review this, not sure this is good
        for (auto heading : headings) {

            sumX += sin(heading);
            sumY += cos(heading);

        }

        auto count = headings.size();

        return atan2(sumY / count, sumX / count);

    }

    arma::mat HydroObjective::GetFunction(arma::mat position, double heading) {

        double x = position(0,0);
        double y = position(1,0);

        double m = cos(heading) / sin(heading);

        double b = y - m * x;

        arma::mat matrix(2,1);

        matrix(0,0) = m;
        matrix(1,0) = b;

        return matrix;

    }

    arma::mat HydroObjective::GetFunction(const proc_hydrophone::PingPoseConstPtr &ping) {

        double heading = ping->pose.orientation.z;
        double x = ping->pose.position.x;
        double y = ping->pose.position.y;

        double m = cos(heading) / sin(heading);

        double b = y - m * x;

        arma::mat matrix(2,1);

        matrix(0,0) = m;
        matrix(1,0) = b;

        return matrix;
    }





    geometry_msgs::PointConstPtr HydroObjective::getPoint() {

        // TODO Constants??
        uint8_t length = 10;

        if (functions.n_cols > 1)
        {
            // If we dont have the length, take all the matrix
            if (functions.n_cols < length)
            {
                length = functions.n_cols;
            }

            auto lastFunctions = functions.tail_cols(length);

            struct Point{
                double_t x;
                double_t y;
            };

            std::vector<Point> points;

            for (int i = 0; i < length; ++i) {
                for (int j = i + 1; j < length; ++j) {

                    arma::mat matI = lastFunctions.col(i);
                    arma::mat matJ = lastFunctions.col(j);

                    arma::mat mMatrix(2, 2);
                    mMatrix(0, 0) = matI(0, 0);
                    mMatrix(1, 0) = matJ(0, 0);
                    mMatrix(0, 1) = -1;
                    mMatrix(1, 1) = -1;

                    arma::mat bMatrix(2, 1);
                    bMatrix(0, 0) = -matI(1, 0);
                    bMatrix(1, 0) = -matJ(1, 0);

                    arma::mat solution = solve(mMatrix, bMatrix);

                    //std::cout << "M matrix" << std::endl;
                    //mMatrix.print();

                    //std::cout << "B matrix" << std::endl;
                    //bMatrix.print();

                    //std::cout << "END matrix" << std::endl;

                    Point point;
                    point.x = solution(0, 0);
                    point.y = solution(1, 0);

                    points.push_back(point);

                }
            }

            auto size = points.size();
            arma::mat pointsMatrix(2, size);

            for (uint16_t i = 0; i < size; ++i) {
                pointsMatrix(0, i) = points[i].x;
                pointsMatrix(1, i) = points[i].y;
            }

            auto centroid = GetCentroids(pointsMatrix, 1);

            //centroid.print();

            geometry_msgs::PointPtr point(new geometry_msgs::Point());

            point->x = centroid(0,0);
            point->y = centroid(1,0);

            return point;

        }

        return geometry_msgs::PointConstPtr();
    }

    void HydroObjective::setOdom(const nav_msgs::OdometryConstPtr &odom) {

        // Todo Logic
        this->odom = odom;

    }




}


