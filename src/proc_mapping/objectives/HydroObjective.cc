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

        pings.push_back(ping);

        auto newFunction = GetFunction(ping);

        functions = join_rows(functions, newFunction);

        // TODO Delete
        functions.print();

    }

    arma::mat HydroObjective::GetFunction(const proc_hydrophone::PingPoseConstPtr &ping) {

        double heading = ping->pose.orientation.z;
        double x = ping->pose.position.x;
        double y = ping->pose.position.y;

        double m = sin(heading) / cos(heading);

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

                    arma::mat mMatrix(2,2);
                    mMatrix(0,0) = matI(0,0);
                    mMatrix(1,0) = matI(0,0);
                    mMatrix(0,1) = -1;
                    mMatrix(1,1) = -1;

                    arma::mat bMatrix(2,1);
                    bMatrix(0,0) = -matI(1,0);
                    bMatrix(1,0) = -matJ(1,0);

                    arma::mat solution = solve(mMatrix, bMatrix);

                    Point point;
                    point.x = solution(0,0);
                    point.y = solution(1,0);

                    points.push_back(point);

                }
            }


            auto size = points.size();
            arma::mat pointsMatrix(size,2);

            for (uint16_t i = 0; i < size; ++i) {
                pointsMatrix(i, 0) = points[i].x;
                pointsMatrix(i, 1) = points[i].y;
            }




        }

        return geometry_msgs::PointConstPtr();
    }
}


