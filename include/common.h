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

#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

// #include <pcl/point_types.h>

sensor_msgs::PointCloud2 ConverToROSmsg(const std::vector<Eigen::Vector3d> &PointCloud);

void PublishPointCloud(const ros::Publisher &publisher, const std::vector<Eigen::Vector3d> &pointclouds, const ros::Time &timestamp, const std::string &frameid)
{
    sensor_msgs::PointCloud2 pubmsg;
    pubmsg = ConverToROSmsg(pointclouds);
    pubmsg.header.stamp = timestamp;
    pubmsg.header.frame_id = frameid;
    publisher.publish(pubmsg);    
}

sensor_msgs::PointCloud2 ConverToROSmsg(const std::vector<Eigen::Vector3d> &PointCloud)
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
    msg.is_bigendian = true;
    msg.point_step = sizeof(point);
    msg.row_step = sizeof(point) * msg.width;
    msg.data = std::move(data_buffer);
    msg.is_dense = true;

    return msg;
}

std::vector<Eigen::Vector3d> ConvertFromROSmsg(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
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

float VerticalAngle(Eigen::Vector3d p){
  return atan(p.z() / sqrt(p.x() * p.x() + p.y() * p.y())) * 180 / M_PI;
}

double PointDistance(Eigen::Vector3d p){
  return sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
}

double PointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2){
  return sqrt((p1.x()-p2.x())*(p1.x()-p2.x()) + (p1.y()-p2.y())*(p1.y()-p2.y()) + (p1.z()-p2.z())*(p1.z()-p2.z()));
}
