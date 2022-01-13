#include <boost/smart_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#include "common.h"
#include "tic_toc.h"






int main(int argc, char **argv)
{

    ros::init(argc, argv, "aFeatureTracking");
    ros::NodeHandle nh("~");

    ros::spin();

    return 0;
}