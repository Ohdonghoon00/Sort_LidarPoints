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

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
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

std::string LIDAR_TYPE;

const double scanPeriod = 0.1;

const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
ros::Publisher pubTestPoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1; 

static bool isBigEndian()
{
    volatile int num = 1;
    return *((char*) &num) == ((char) 1);
}

static const bool IS_BIG_ENDIAN = isBigEndian();

// template <typename PointT>
// void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
//                               pcl::PointCloud<PointT> &cloud_out, float thres)
// {
//     if (&cloud_in != &cloud_out)
//     {
//         cloud_out.header = cloud_in.header;
//         cloud_out.points.resize(cloud_in.points.size());
//     }

//     size_t j = 0;

//     for (size_t i = 0; i < cloud_in.points.size(); ++i)
//     {
//         if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
//             continue;
//         cloud_out.points[j] = cloud_in.points[i];
//         j++;
//     }
//     if (j != cloud_in.points.size())
//     {
//         cloud_out.points.resize(j);
//     }

//     cloud_out.height = 1;
//     cloud_out.width = static_cast<uint32_t>(j);
//     cloud_out.is_dense = true;
// }

void RemoveClosedPointCloud(std::vector<Eigen::Vector3f> &cloud_in,
                              std::vector<Eigen::Vector3f> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
        cloud_out.resize(cloud_in.size());
    
    
    size_t j = 0;
    for (size_t i = 0; i < cloud_in.size(); ++i)
    {
        if (cloud_in[i](0) * cloud_in[i](0) + cloud_in[i](1) * cloud_in[i](1) + cloud_in[i](2) * cloud_in[i](2) < thres * thres)
            continue;
        cloud_out[j] = cloud_in[i];
        j++;
    }

    if (j != cloud_in.size())
        cloud_out.resize(j);
    
}

void RemoveNaNFromPointCloud(std::vector<Eigen::Vector3f> &cloud_in,
                              std::vector<Eigen::Vector3f> &cloud_out)
{
    if (&cloud_in != &cloud_out)
        cloud_out.resize(cloud_in.size());    
    
    size_t j = 0;
    // Remove Nan, Inf
    for(size_t i = 0; i < cloud_in.size(); i++){
        
        bool is_nan_x = isnan(cloud_in[i](0));
        bool is_nan_y = isnan(cloud_in[i](1));
        bool is_nan_z = isnan(cloud_in[i](2));

        if(is_nan_x || is_nan_y || is_nan_z)
            continue;
        
        bool is_inf_x = isinf(cloud_in[i](0));
        bool is_inf_y = isinf(cloud_in[i](1));
        bool is_inf_z = isinf(cloud_in[i](2));
        
        if(is_inf_x || is_inf_y || is_inf_z)
            continue;
        
        cloud_out[j] = cloud_in[i];
        j++;
    }

    if (j != cloud_in.size())
        cloud_out.resize(j);

}

    
    

    



sensor_msgs::PointCloud2 ConverToROSmsg(std::vector<Eigen::Vector3f> PointCloud)
{
    struct point { float x, y, z; };

    const size_t PointCloudNum = PointCloud.size();

    std::vector<uint8_t> data_buffer(PointCloudNum * sizeof(point));
    size_t idx = 0;

    point *dataptr = (point*) data_buffer.data();

    for(auto i : PointCloud){
        dataptr[idx++] = {i(0), i(1), i(2)};
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

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }
    
    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    std::vector<Eigen::Vector3f> laserPoints;
    // pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    // std::vector<int> indices;

    laserPoints.clear();
    for(size_t i = 0; i < laserCloudMsg->data.size(); i++){
        
        laserPoints.resize(laserCloudMsg->data.size());
        // laserPoints[i](0) = laserCloudMsg->data[i][0];
        // laserPoints[i](1) = laserCloudIn.points[i].y;
        // laserPoints[i](2) = laserCloudIn.points[i].z;
    }
    // std::cout << (float)laserCloudMsg->data[0] << std::endl;
    
    // pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    // removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

    RemoveNaNFromPointCloud(laserPoints, laserPoints);
    RemoveClosedPointCloud(laserPoints, laserPoints, MINIMUM_RANGE);

// std::cout << "testpoint size :  " << laserPoints.size() << "laserpoint size : " << laserCloudIn.points.size() << std::endl;

    int cloudSize = laserPoints.size();
    float startOri = -atan2(laserPoints[0](1), laserPoints[0](0));
    float endOri = -atan2(laserPoints[cloudSize - 1](1),
                          laserPoints[cloudSize - 1](0)) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    ////printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    // PointType point;
    Eigen::Vector3f point;

    // Visual remove points
    // pcl::PointCloud<PointType> removePoints;
    // PointType removePoint;
    
    // std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    std::vector<std::vector<Eigen::Vector3f>> laserCloudScans(N_SCANS);

    
    std::vector<float> Verticalangles;
    for(int i = 0; i < cloudSize; i++){
        
        point(0) = laserPoints[i](0);
        point(1) = laserPoints[i](1);
        point(2) = laserPoints[i](2);        
        float angle = atan(point(2) / sqrt(point(0) * point(0) + point(1) * point(1))) * 180 / M_PI;
        Verticalangles.push_back(angle);
    }
    
    float MaxAngle = *max_element(Verticalangles.begin(), Verticalangles.end());
    float MinAngle = *min_element(Verticalangles.begin(), Verticalangles.end());
    float Ratio = (MaxAngle - MinAngle) / N_SCANS;
    
    // removePoints.clear();
    for (int i = 0; i < cloudSize; i++)
    {
        point(0) = laserPoints[i](0);
        point(1) = laserPoints[i](1);
        point(2) = laserPoints[i](2);
        


        float angle = atan(point(2) / sqrt(point(0) * point(0) + point(1) * point(1))) * 180 / M_PI;
        int scanID = 0;
        for(int j = 0; j < N_SCANS; j++){
            if(angle < MinAngle + Ratio * (j + 1)){
                scanID = j;
                break;
                
            }
        }

        // if (LIDAR_TYPE == "VLP16" && N_SCANS == 16)
        // {
        //     scanID = int((angle + 15) / 2 + 0.5);
        //     // if(scanID == 15) std::cout << angle << std::endl;
        //     if (scanID > (N_SCANS - 1) || scanID < 0)
        //     {
        //         removePoints.push_back(point);
        //         count--;
        //         continue;
        //     }
        // }


        // else if (LIDAR_TYPE == "HDL32" && N_SCANS == 32)
        // {
        //     scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
        //     if (scanID > (N_SCANS - 1) || scanID < 0)
        //     {
        //         count--;
        //         continue;
        //     }
        // }
        // // HDL64 (e.g., KITTI)
        // else if (LIDAR_TYPE == "HDL64" && N_SCANS == 64)
        // {   
        //     if (angle >= -8.83)
        //         scanID = int((2 - angle) * 3.0 + 0.5);
        //     else
        //         scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

        //     // use [0 50]  > 50 remove outlies 
        //     if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
        //     {
        //         count--;
        //         continue;
        //     }
        // }
        // // Ouster OS1-64 (e.g., MulRan)
        // else if (LIDAR_TYPE == "OS1-64" && N_SCANS == 64)
        // {   
        //     scanID = int((angle + 22.5) / 2 + 0.5); // ouster os1-64 vfov is [-22.5, 22.5] see https://ouster.com/products/os1-lidar-sensor/
        //     if (scanID > (N_SCANS - 1) || scanID < 0)
        //     {
        //         count--;
        //         continue;
        //     }
        // }
        // else
        // {
        //     //printf("wrong scan number\n");
        //     ROS_BREAK();
        // }
        ////printf("angle %f scanID %d \n", angle, scanID);
        // std::cout << " gt : " << scanID << "  Estimate : " << scanID_ << std::endl;

        float ori = -atan2(point(1), point(0));
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        // point.intensity = scanID + scanPeriod * relTime;
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;
    //printf("points size %d \n", cloudSize);

    // pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    // boost::shared_ptr<std::vector<Eigen::Vector3f>> laserCloud(new std::vector<Eigen::Vector3f>());
    std::vector<Eigen::Vector3f> laserCloud;

    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud.size() + 5;
        for(size_t j = 0; j < laserCloudScans[i].size(); j++)
            laserCloud.push_back(laserCloudScans[i][j]);
        scanEndInd[i] = laserCloud.size() - 6;
        // std::cout << " start index : " << scanStartInd[i] << " end index : " << scanEndInd[i] << std::endl;
    }
        



    //printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud[i - 5](0) + laserCloud[i - 4](0) + laserCloud[i - 3](0) + laserCloud[i - 2](0) + laserCloud[i - 1](0) - 10 * laserCloud[i](0) + laserCloud[i + 1](0) + laserCloud[i + 2](0) + laserCloud[i + 3](0) + laserCloud[i + 4](0) + laserCloud[i + 5](0);
        float diffY = laserCloud[i - 5](1) + laserCloud[i - 4](1) + laserCloud[i - 3](1) + laserCloud[i - 2](1) + laserCloud[i - 1](1) - 10 * laserCloud[i](1) + laserCloud[i + 1](1) + laserCloud[i + 2](1) + laserCloud[i + 3](1) + laserCloud[i + 4](1) + laserCloud[i + 5](1);
        float diffZ = laserCloud[i - 5](2) + laserCloud[i - 4](2) + laserCloud[i - 3](2) + laserCloud[i - 2](2) + laserCloud[i - 1](2) - 10 * laserCloud[i](2) + laserCloud[i + 1](2) + laserCloud[i + 2](2) + laserCloud[i + 3](2) + laserCloud[i + 4](2) + laserCloud[i + 5](2);

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }


    TicToc t_pts;

    // pcl::PointCloud<PointType> cornerPointsSharp;
    // pcl::PointCloud<PointType> cornerPointsLessSharp;
    // pcl::PointCloud<PointType> surfPointsFlat;
    // pcl::PointCloud<PointType> surfPointsLessFlat;
    std::vector<Eigen::Vector3f> cornerPointsSharp;
    std::vector<Eigen::Vector3f> cornerPointsLessSharp;
    std::vector<Eigen::Vector3f> surfPointsFlat;
    std::vector<Eigen::Vector3f> surfPointsLessFlat;


    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k]; 

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 1.0)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud[ind]);
                        cornerPointsLessSharp.push_back(laserCloud[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1; 

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud[ind + l](0) - laserCloud[ind + l - 1](0);
                        float diffY = laserCloud[ind + l](1) - laserCloud[ind + l - 1](1);
                        float diffZ = laserCloud[ind + l](2) - laserCloud[ind + l - 1](2);
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud[ind + l](0) - laserCloud[ind + l + 1](0);
                        float diffY = laserCloud[ind + l](1) - laserCloud[ind + l + 1](1);
                        float diffZ = laserCloud[ind + l](2) - laserCloud[ind + l + 1](2);
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud[ind + l](0) - laserCloud[ind + l - 1](0);
                        float diffY = laserCloud[ind + l](1) - laserCloud[ind + l - 1](1);
                        float diffZ = laserCloud[ind + l](2) - laserCloud[ind + l - 1](2);
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud[ind + l](0) - laserCloud[ind + l + 1](0);
                        float diffY = laserCloud[ind + l](1) - laserCloud[ind + l + 1](1);
                        float diffZ = laserCloud[ind + l](2) - laserCloud[ind + l + 1](2);
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    pcl::PointXYZ point_;
                    point_.x = laserCloud[k](0);
                    point_.y = laserCloud[k](1);
                    point_.z = laserCloud[k](2);
                    surfPointsLessFlatScan->push_back(point_);
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        // surfPointsLessFlat += surfPointsLessFlatScanDS;
        for(size_t m = 0; m < surfPointsLessFlatScanDS.size(); m++){
            Eigen::Vector3f _point;
            _point << surfPointsLessFlatScanDS[m].x, surfPointsLessFlatScanDS[m].y, surfPointsLessFlatScanDS[m].z; 
            surfPointsLessFlat.push_back(_point);
        }
    }
    //printf("sort q time %f \n", t_q_sort);
    //printf("seperate points time %f \n", t_pts.toc());

    // sensor_msgs::PointCloud2 removelaserCloudOutMsg;
    // // pcl::toROSMsg(removePoints, removelaserCloudOutMsg);
    // removelaserCloudOutMsg = ConverToROSmsg(removePoints);
    // removelaserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    // removelaserCloudOutMsg.header.frame_id = "/camera_init";
    // pubRemovePoints.publish(removelaserCloudOutMsg);
    

    sensor_msgs::PointCloud2 laserCloudOutMsg;
    // pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg = ConverToROSmsg(laserCloud);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    // pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg = ConverToROSmsg(cornerPointsSharp);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);


    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    // pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg = ConverToROSmsg(cornerPointsLessSharp);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    // pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2 = ConverToROSmsg(surfPointsFlat);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    // pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2 = ConverToROSmsg(surfPointsLessFlat);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
    // std::cout << "published ... " << std::endl;


    // // Test Points
    // auto testmsg = ConverToROSmsg(laserPoints);
    // testmsg.header.stamp = laserCloudMsg->header.stamp;
    // testmsg.header.frame_id = "/camera_init";
    // pubTestPoints.publish(testmsg);

    // pub each scam
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            // pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg = ConverToROSmsg(laserCloudScans[i]);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "/camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    //printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
        
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    nh.param<int>("scan_line", N_SCANS, 16);
    nh.param<std::string>("lidar_type", LIDAR_TYPE, "KITTI");
    nh.param<double>("minimum_range", MINIMUM_RANGE, 5.0);

    //printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        //printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);


    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    pubTestPoints = nh.advertise<sensor_msgs::PointCloud2>("/TestPoints", 100);

    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    return 0;
}
