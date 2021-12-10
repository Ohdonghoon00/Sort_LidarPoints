// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/smart_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include "common.h"
#include "tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;

const int systemDelay = 0; 
const size_t kMaxNumberOfPoints = 1e5;
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
int cloudSize = 0;
std::vector<int> scanStartInd(N_SCANS, 0);
std::vector<int> scanEndInd(N_SCANS, 0);

float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

std::vector<Eigen::Vector3d> cornerPointsSharp;
std::vector<Eigen::Vector3d> cornerPointsLessSharp;
std::vector<Eigen::Vector3d> surfPointsFlat;
std::vector<Eigen::Vector3d> surfPointsLessFlat;
std::vector<std::vector<int>> PointIndexByChannel; // ( edge → 0 , plane → 1 , pass point → -1, occluded/parallel → -2 )

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;

double MINIMUM_RANGE = 0.1; 

static bool isBigEndian()
{
    volatile int num = 1;
    return *((char*) &num) == ((char) 1);
}

static const bool IS_BIG_ENDIAN = isBigEndian();

void RemoveClosedPointCloud(std::vector<Eigen::Vector3d> &cloud_in,
                              std::vector<Eigen::Vector3d> &cloud_out)
{
    if (&cloud_in != &cloud_out)
        cloud_out.resize(cloud_in.size());
    
    
    size_t j = 0;
    for (size_t i = 0; i < cloud_in.size(); ++i)
    {
        if (cloud_in[i].x() * cloud_in[i].x() + cloud_in[i].y() * cloud_in[i].y() + cloud_in[i].z() * cloud_in[i].z() < MINIMUM_RANGE * MINIMUM_RANGE)
            continue;
        cloud_out[j] = cloud_in[i];
        j++;
    }

    if (j != cloud_in.size())
        cloud_out.resize(j);
    
}

void RemoveNaNFromPointCloud(std::vector<Eigen::Vector3d> &cloud_in,
                              std::vector<Eigen::Vector3d> &cloud_out)
{
    if (&cloud_in != &cloud_out)
        cloud_out.resize(cloud_in.size());    
    
    size_t j = 0;
    // Remove Nan, Inf
    for(size_t i = 0; i < cloud_in.size(); i++){
        
        bool is_nan_x = isnan(cloud_in[i].x());
        bool is_nan_y = isnan(cloud_in[i].y());
        bool is_nan_z = isnan(cloud_in[i].z());

        if(is_nan_x || is_nan_y || is_nan_z)
            continue;
        
        bool is_inf_x = isinf(cloud_in[i].x());
        bool is_inf_y = isinf(cloud_in[i].y());
        bool is_inf_z = isinf(cloud_in[i].z());
        
        if(is_inf_x || is_inf_y || is_inf_z)
            continue;
        
        cloud_out[j] = cloud_in[i];
        j++;
    }

    if (j != cloud_in.size())
        cloud_out.resize(j);

}

sensor_msgs::PointCloud2 ConverToROSmsg(std::vector<Eigen::Vector3d> PointCloud)
{
    struct point { float x, y, z; };
    const size_t PointCloudNum = PointCloud.size();

    std::vector<uint8_t> data_buffer(PointCloudNum * sizeof(point));
    size_t idx = 0;

    point *dataptr = (point*) data_buffer.data();

    for(auto i : PointCloud){
        dataptr[idx++] = {(float)i(0), (float)i(1), (float)i(2)};
    }

    static const char* const names[3] = { "x", "y", "z" };
    static const std::size_t offsets[3] = { offsetof(point, x), offsetof(point, y), offsetof(point, z) };
    std::vector<sensor_msgs::PointField> fields(3);
    for (int i=0; i < 3; i++) {
        fields[i].name = names[i];
        fields[i].offset = offsets[i];
        fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        fields[i].count = 1;
    }


    sensor_msgs::PointCloud2 msg;
    msg.height = 1;
    msg.width = PointCloudNum;
    msg.fields = fields;
    msg.is_bigendian = IS_BIG_ENDIAN;
    msg.point_step = sizeof(point);
    msg.row_step = sizeof(point) * msg.width;
    msg.data = std::move(data_buffer);
    msg.is_dense = true;

    return msg;
}

std::vector<Eigen::Vector3d> ConverToEigenVector(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    std::vector<Eigen::Vector3d> laserPoints;
    uint8_t* byte = const_cast<uint8_t*>(&laserCloudMsg->data[0]);
    float* floatByte = reinterpret_cast<float*>(byte);
    int pointcloudNum = laserCloudMsg->data.size() / (sizeof(float) * 3);
    laserPoints.resize(pointcloudNum);

    for(int i = 0; i < pointcloudNum; i++){
        
        laserPoints[i].x() = (double)floatByte[3 * i];
        laserPoints[i].y() = (double)floatByte[3 * i + 1];
        laserPoints[i].z() = (double)floatByte[3 * i + 2];
    }

    return laserPoints;
}

void DividePointsByChannel(const std::vector<Eigen::Vector3d> &laserPoints, std::vector<Eigen::Matrix3Xd> &laserCloudScans)
{
    // Calculate Max and Min Vertical Angle
    std::vector<float> Verticalangles;
    Eigen::Vector3d point;
    std::vector<int> PointNumByScanID(N_SCANS, 0);
    
    for(int i = 0; i < cloudSize; i++){
        
        point.x() = laserPoints[i].x();
        point.y() = laserPoints[i].y();
        point.z() = laserPoints[i].z();        
        float angle = atan(point.z() / sqrt(point.x() * point.x() + point.y() * point.y())) * 180 / M_PI;
        Verticalangles.push_back(angle);
    }

    float MaxAngle = *max_element(Verticalangles.begin(), Verticalangles.end());
    float MinAngle = *min_element(Verticalangles.begin(), Verticalangles.end());
    float Ratio = (MaxAngle - MinAngle) / N_SCANS;

    // Sort Points by channel
    for (int i = 0; i < cloudSize; i++){
        point.x() = laserPoints[i].x();
        point.y() = laserPoints[i].y();
        point.z() = laserPoints[i].z();
        
        float angle = atan(point.z() / sqrt(point.x() * point.x() + point.y() * point.y())) * 180 / M_PI;
        int scanID = 0;
        for(int j = 0; j < N_SCANS; j++){
            if(angle < MinAngle + Ratio * (j + 1)){
                scanID = j;
                break;
            }
        }

        laserCloudScans[scanID](0, PointNumByScanID[scanID]) = point.x();
        laserCloudScans[scanID](1, PointNumByScanID[scanID]) = point.y();
        laserCloudScans[scanID](2, PointNumByScanID[scanID]) = point.z();

        PointNumByScanID[scanID]++;

    }
        // resize             
        for(int i = 0; i < N_SCANS; i++)
            laserCloudScans[i].conservativeResize(3, PointNumByScanID[i]);
}

void CalculateCurvature(std::vector<double> &PointRange)
{
    for (int i = 5; i < cloudSize - 5; i++){         
        cloudCurvature[i] = PointRange[i - 5] + PointRange[i - 4] +
                            PointRange[i - 3] + PointRange[i - 2] +
                            PointRange[i - 1] + PointRange[i + 1] +
                            PointRange[i + 2] + PointRange[i + 3] +
                            PointRange[i + 4] + PointRange[i + 5] +
                            - 10 * PointRange[i];
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }
}

void MarkOccludedPoints(std::vector<Eigen::Vector3d> &laserCloud, std::vector<double> &PointRange)
{
    // Occluded and Parallel beam
    int occluded_cnt = 0;
    int parallel_cnt = 0;
    for (int i = 5; i < cloudSize - 6; i++){
        
        // occluded points
        double depth1 = PointRange[i];
        double depth2 = PointRange[i + 1];
        double Diff = PointDistance(laserCloud[i + 1], laserCloud[i]);
std::cout << depth1 << std::endl;
        if (Diff * Diff < 0.05){
            if (depth1 - depth2 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        } 
            
        // parallel beam
        std::cout << "  !!!!!!!!!!!!!!  " << std::endl;
        std::cout << laserCloud[i - 1] << std::endl;
        std::cout << laserCloud[i] << std::endl;
        double diff1 = PointDistance(laserCloud[i - 1], laserCloud[i]);
        double diff2 = PointDistance(laserCloud[i + 1], laserCloud[i]);
            std::cout << diff1 << "  " << diff2  << std::endl;

        if (diff1 > 0.02 * PointRange[i] && diff2 > 0.02 * PointRange[i]){
            parallel_cnt++;
            cloudNeighborPicked[i] = 1;
        }
      
    }
    
    std::cout << "Occluded points : " << occluded_cnt << std::endl;
    std::cout << "Parallel points : " << parallel_cnt << std::endl;

}

void DividePointsByEdgeAndPlane(const std::vector<Eigen::Vector3d> &laserCloud)
{

    cornerPointsSharp.clear();
    cornerPointsLessSharp.clear();
    surfPointsFlat.clear();
    surfPointsLessFlat.clear();    
    
    // Edge Points and Plane Points
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < 6; j++){
            
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);

            // Edge Points
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--){
                int ind = cloudSortInd[k]; 

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 1.0)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2){                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud[ind]);
                        cornerPointsLessSharp.push_back(laserCloud[ind]);
                        // PointIndexByChannel[ind]
                    }
                    else if (largestPickedNum <= 20){                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud[ind]);
                    }
                    else
                        break;
                    
                    

                    cloudNeighborPicked[ind] = 1; 

                    for (int l = 1; l <= 5; l++){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;
                        
                        cloudNeighborPicked[ind + l] = 1;
                    }
                        

                    for (int l = -1; l >= -5; l--){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l + 1]);
                        if(diff * diff > 0.05)
                            break;
                        
                        cloudNeighborPicked[ind + l] = 1;
                    }
                        

                }
            }
            
            // Plane Points
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++){
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                        break;
                    

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++){ 
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++){
                if (cloudLabel[k] <= 0){
                    pcl::PointXYZ point_;
                    point_.x = laserCloud[k].x();
                    point_.y = laserCloud[k].y();
                    point_.z = laserCloud[k].z();
                    surfPointsLessFlatScan->push_back(point_);
                }
            }
        }
        
        // std::cout << " flat points num before downsize filtering : " << surfPointsLessFlatScan->size() << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        // std::cout << " flat points num After downsize filtering : " << surfPointsLessFlatScanDS.size() << std::endl;


        // surfPointsLessFlat += surfPointsLessFlatScanDS;
        for(size_t m = 0; m < surfPointsLessFlatScanDS.size(); m++){
            Eigen::Vector3d _point;
            _point << surfPointsLessFlatScanDS[m].x, surfPointsLessFlatScanDS[m].y, surfPointsLessFlatScanDS[m].z; 
            surfPointsLessFlat.push_back(_point);
        }
    } 
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited){ 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
            systemInited = true;
        else
            return;
    }
        
    TicToc t_whole;
    TicToc t_prepare;
    
    // Storage pointcloud from ROS msg        
    std::vector<Eigen::Vector3d> laserPoints = ConverToEigenVector(laserCloudMsg);
    
    // Remove useless points
    RemoveNaNFromPointCloud(laserPoints, laserPoints);
    RemoveClosedPointCloud(laserPoints, laserPoints);
    
    cloudSize = laserPoints.size();
// std::cout << "Point Num  :  " << cloudSize << std::endl; 
    
    PointIndexByChannel.clear();
    // PointIndexByChannel.resize(cloudSize, 0);
    
    std::vector<Eigen::Matrix3Xd> laserCloudScans(N_SCANS, Eigen::Matrix3Xd(3, kMaxNumberOfPoints ));
        
    // Sort Points by Scan Line
    DividePointsByChannel(laserPoints, laserCloudScans);


    std::vector<Eigen::Vector3d> laserCloud;
    std::vector<double> PointRange;
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);
    int idx = 0;
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud.size() + 5;

        for(int j = 0; j < laserCloudScans[i].cols(); j++){

            laserCloud.resize(idx + 1);
            laserCloud[idx].x() = laserCloudScans[i](0, j);
            laserCloud[idx].y() = laserCloudScans[i](1, j);
            laserCloud[idx].z() = laserCloudScans[i](2, j);
            PointRange.push_back(PointDistance(laserCloud[idx]));
            std::cout << PointDistance(laserCloud[idx]) << std::endl;
            idx++;
        }

        scanEndInd[i] = laserCloud.size() - 6;
        // std::cout << " start index : " << scanStartInd[i] << " end index : " << scanEndInd[i] << std::endl;
    }
    // printf("prepare time %f \n", t_prepare.toc());

    CalculateCurvature(PointRange);
    // MarkOccludedPoints(laserPoints, PointRange);
    
    
    // Occluded and Parallel beam
    int occluded_cnt = 0;
    int parallel_cnt = 0;
    for (int i = 5; i < cloudSize - 6; i++){
        
        // occluded points
        double depth1 = PointRange[i];
        double depth2 = PointRange[i + 1];
        double Diff = PointDistance(laserCloud[i + 1], laserCloud[i]);

        if (Diff * Diff < 0.05){
            if (depth1 - depth2 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        } 
            
        // parallel beam
        //     std::cout << "  !!!!!!!!!!!!!!  " << std::endl;
        // std::cout << laserCloud[i - 1] << std::endl;
        // std::cout << laserCloud[i] << std::endl;
        double diff1 = PointDistance(laserCloud[i - 1], laserCloud[i]);
        double diff2 = PointDistance(laserCloud[i + 1], laserCloud[i]);
            // std::cout << diff1 << "  " << diff2  << std::endl;

        if (diff1 > 0.02 * PointRange[i] && diff2 > 0.02 * PointRange[i]){
            parallel_cnt++;
            cloudNeighborPicked[i] = 1;
        }
      
    }
    
    std::cout << "Occluded points : " << occluded_cnt << std::endl;
    std::cout << "Parallel points : " << parallel_cnt << std::endl;    

    
    
    // sort Lidar points to edge and plane
    DividePointsByEdgeAndPlane(laserCloud);

    //
    std::cout << "laserCloud num : " << laserCloud.size() << std::endl;
    std::cout << "cornerPointsSharp num : " << cornerPointsSharp.size() << std::endl;
    std::cout << "cornerPointsLessSharp num : " << cornerPointsLessSharp.size() << std::endl;
    std::cout << "surfPointsFlat num : " << surfPointsFlat.size() << std::endl;
    std::cout << "surfPointsLessFlat num : " << surfPointsLessFlat.size() << std::endl;


    // Publish Points
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    laserCloudOutMsg = ConverToROSmsg(laserCloud);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    cornerPointsSharpMsg = ConverToROSmsg(cornerPointsSharp);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);


    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    cornerPointsLessSharpMsg = ConverToROSmsg(cornerPointsLessSharp);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    surfPointsFlat2 = ConverToROSmsg(surfPointsFlat);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    surfPointsLessFlat2 = ConverToROSmsg(surfPointsLessFlat);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
    // std::cout << "published ... " << std::endl;

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
        
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    nh.param<int>("scan_line", N_SCANS, 16);
    nh.param<double>("minimum_range", MINIMUM_RANGE, 5.0);

    //printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64){
        //printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    ros::spin();

    return 0;
}
    
    




    

    

 






    
 
    
    

    




