//
// Created by coumarc9 on 7/21/17.
//

#ifndef PROC_MAPPING_HYDROOBJECTIVE_H
#define PROC_MAPPING_HYDROOBJECTIVE_H

#include <proc_hydrophone/PingPose.h>
#include <nav_msgs/Odometry.h>
#define ARMA_DONT_PRINT_ERRORS
//#include <armadillo>
#include <mlpack/methods/kmeans/kmeans.hpp>

namespace proc_mapping{
    class HydroObjective {
    public:
        HydroObjective();
        ~HydroObjective();

        void addPing(const proc_hydrophone::PingPoseConstPtr &ping);
        geometry_msgs::PointConstPtr getPoint();

        void setOdom(const nav_msgs::OdometryConstPtr &odom);

    private:

        std::vector<proc_hydrophone::PingPoseConstPtr> pings;
        arma::mat functions;// Matrix with m and b of y = mx+b linear function

        arma::mat GetFunction(const proc_hydrophone::PingPoseConstPtr &ping);

        /// TODO delete
        nav_msgs::OdometryConstPtr odom;

        bool needProcess(const proc_hydrophone::PingPoseConstPtr &ping);

        arma::mat GetPositions();
        std::vector<double> GetHeadings();

        arma::mat GetCentroids(arma::mat matrix, unsigned int nb);
        double GetHeading(std::vector<double> headings);


        arma::mat GetFunction(arma::mat position, double heading);

    };
}




#endif //PROC_MAPPING_HYDROOBJECTIVE_H
