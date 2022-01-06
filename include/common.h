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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

// #include <pcl/point_types.h>

struct Line;
sensor_msgs::PointCloud2 ConverToROSmsg(const std::vector<Eigen::Vector3d> &PointCloud);
visualization_msgs::MarkerArray ConverToROSmsg(const std::vector<Line> &line, const ros::Time &timestamp, const std::string &frameid);
visualization_msgs::MarkerArray ConverToROSmsg(const ros::Time &timestamp, const std::string &frameid);


struct Line
{
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    // Eigen::Vector3d vec = p2 - p1;
};

struct Plane
{
    Eigen::Vector3d normal;
    Eigen::Vector3d centroid;
};

void PublishPointCloud(const ros::Publisher &publisher, const std::vector<Eigen::Vector3d> &pointclouds, const ros::Time &timestamp, const std::string &frameid)
{
    sensor_msgs::PointCloud2 pubmsg;
    pubmsg = ConverToROSmsg(pointclouds);
    pubmsg.header.stamp = timestamp;
    pubmsg.header.frame_id = frameid;
    publisher.publish(pubmsg);    
}

void PublishLine(const ros::Publisher &publisher, const std::vector<Line> &line, const ros::Time &timestamp, const std::string &frameid)
{
    visualization_msgs::MarkerArray pubmsg;
    pubmsg = ConverToROSmsg(line, timestamp, frameid);

    visualization_msgs::MarkerArray DeleteMarkermsg;
    DeleteMarkermsg = ConverToROSmsg(timestamp, frameid);
    
    publisher.publish(DeleteMarkermsg);
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

visualization_msgs::MarkerArray ConverToROSmsg(const std::vector<Line> &line, const ros::Time &timestamp, const std::string &frameid)
{
    visualization_msgs::MarkerArray line_arr;



    for(size_t i = 0; i < line.size(); i++){
        visualization_msgs::Marker msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frameid;
        msg.id = (int)i;
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 0.03;
        msg.scale.y = 0.08;
        msg.scale.z = 0.05;
        msg.color.r = 1.0;
        msg.color.a = 1.0;          
        msg.type = visualization_msgs::Marker::ARROW;
        
        geometry_msgs::Point p;
        p.x = line[i].p1.x();
        p.y = line[i].p1.y();
        p.z = line[i].p1.z();
        msg.points.push_back(p);

        p.x = line[i].p2.x();
        p.y = line[i].p2.y();
        p.z = line[i].p2.z();
        msg.points.push_back(p);

        line_arr.markers.push_back(msg);
    }


  return line_arr;
}

visualization_msgs::MarkerArray ConverToROSmsg(const ros::Time &timestamp, const std::string &frameid)
{
    visualization_msgs::MarkerArray msg_arr;
    visualization_msgs::Marker msg;
    
    msg.header.stamp = timestamp;
    msg.header.frame_id = frameid;
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg_arr.markers.push_back(msg);
    
    return msg_arr;
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

double Point2LineDistance(Line l, Eigen::Vector3d p){
    return ((p - l.p1).cross(p - l.p2)).norm() / (l.p1 - l.p2).norm();
}

double CosRaw2(double a, double b, float ang){
    return sqrt(a * a + b * b - 2 * a * b * cos(ang * M_PI / 180));
}

double LineThres(Eigen::Vector3d p, float VerticalAngleRatio){
    
    double dist = PointDistance(p);
    float VerticalAngle_ = VerticalAngle(p);
    double PointToLine = dist * cos(std::abs(VerticalAngle_) * M_PI / 180);
    float theta = std::abs(VerticalAngleRatio + VerticalAngle_);
    double NextPointDis = PointToLine / cos(theta * M_PI / 180);
    
    return CosRaw2(dist, NextPointDis, VerticalAngleRatio);
}

Eigen::Matrix3d NormalToRotation(Eigen::Vector3d n)
{
    Eigen::Matrix3d r;
    Eigen::Vector3d UnitVec(0, 1, 1);
    
    Eigen::Vector3d r1 = UnitVec.cross(n);
    Eigen::Vector3d r2 = n.cross(r1);
    Eigen::Vector3d r3 = n;

    r.row(0) = r1;
    r.row(1) = r2;
    r.row(2) = r3;

    return r;

}

// Eigen::Matrix3d NormalToRotation(Eigen::Vector3d n)
// {
//     Eigen::Matrix3d r;
//     Eigen::Vector3d UnitVec(0, 1, 0);
    
//     Eigen::Vector3d w = UnitVec.cross(n);
    
//     Eigen::Vector3d a = UnitVec.cross(w);
//     Eigen::Vector3d b = n.cross(w);
    
//     Eigen::Matrix3d A;
//     A <<    UnitVec.transpose(),
//             w.transpose(),
//             a.transpose();
    
//     Eigen::Matrix3d B;
//     B <<    n.transpose(),
//             w.transpose(),
//             b.transpose();

//     r = B.dot(A.inverse());

//     return r;

// }
