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
#include "sort_lidarpoints/feature_info.h"

sort_lidarpoints::feature_info FeatureInfo;

ros::Subscriber subFeature;

rviz_visual_tools::RvizVisualToolsPtr VisualLine;
rviz_visual_tools::RvizVisualToolsPtr VisualPlane;
rviz_visual_tools::RvizVisualToolsPtr VisualPlane2;
rviz_visual_tools::RvizVisualToolsPtr VisualArrow;

// Total Feature info
std::vector<Line> Lines;
std::vector<Plane> Planes;

// Last Feature info
std::vector<Line> LastLines;
std::vector<Plane> LastPlanes;

int FrameNum = 0;
int MaxLineId = 0;
int MaxPlaneId = 0;

//////////// Parameter ////////////

// plane
double PlaneCentroidDisThres = 6.0;

// line
double LineToLineDisThres = 0.0;

void Initialization(std::vector<Line> &line, std::vector<Plane> &plane)
{
    
    for(auto i : line){
        Lines.push_back(i);
        Lines[MaxLineId].id = MaxLineId;
        line[MaxLineId].id = MaxLineId;
        MaxLineId++;
    }

    for(auto i : plane){
        Planes.push_back(i);
        Planes[MaxPlaneId].id = MaxPlaneId;
        plane[MaxPlaneId].id = MaxPlaneId;
        MaxPlaneId++;
    }
}

void CopyNewFeature(const std::vector<Line> curr_line, const std::vector<Plane> curr_plane)
{
    LastLines.clear();
    LastPlanes.clear();

    LastLines.assign(curr_line.begin(), curr_line.end());
    LastPlanes.assign(curr_plane.begin(), curr_plane.end());

    FrameNum++;   
}
void ConvertfromRosMsg( const sort_lidarpoints::feature_infoConstPtr &FeatureMsg,
                    std::vector<Line> &line, std::vector<Plane> &plane)
{
    // Memory
    line.resize(FeatureMsg->p1_x.size());
    plane.resize(FeatureMsg->scale.size());
    
    MsgToFeature(FeatureMsg, line, plane);
}

void MsgToFeatureInfo(const sort_lidarpoints::feature_info FeatureInfo, Vector6f *IMUdata)
{
    *IMUdata <<  FeatureInfo.gyro_rx,
                FeatureInfo.gyro_ry,
                FeatureInfo.gyro_rz,
                FeatureInfo.gyro_x,
                FeatureInfo.gyro_y,
                FeatureInfo.gyro_z;
    
}

void MovePlane(const Eigen::Matrix4f LidarRotation, std::vector<Plane>* last_plane)
{
    int PlaneNum = last_plane->size();
    Eigen::MatrixXd MatrixPoints(4, PlaneNum * 2);
    for(int i = 0; i < PlaneNum; i++){
        MatrixPoints(0, 2*i) = (*last_plane)[i].centroid.x();
        MatrixPoints(1, 2*i) = (*last_plane)[i].centroid.y();
        MatrixPoints(2, 2*i) = (*last_plane)[i].centroid.z();
        MatrixPoints(3, 2*i) = 1.0;

        MatrixPoints(0, 2*i + 1) = (*last_plane)[i].normal.x();
        MatrixPoints(1, 2*i + 1) = (*last_plane)[i].normal.y();
        MatrixPoints(2, 2*i + 1) = (*last_plane)[i].normal.z();
        MatrixPoints(3, 2*i + 1) = 1.0;
    }

    float angle = ToAngle(LidarRotation);
    Eigen::Vector3f Axis = ToAxis(LidarRotation);   
    Axis = Axis * angle;

    double data[] = {(double)Axis(0, 0), (double)Axis(1, 0), (double)Axis(2, 0)};
    cv::Mat R(3, 1, CV_64FC1, data);
    cv::Rodrigues(R, R);

    Eigen::Matrix4d RT;
    RT <<   R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), LidarRotation(0, 3),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), LidarRotation(1, 3),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), LidarRotation(2, 3),
            0,                  0,                  0,              1;

    Eigen::Matrix4Xd MatrixPoints_ = RT.inverse() * MatrixPoints;

    for(int i = 0; i < PlaneNum; i++){
        
        (*last_plane)[i].centroid.x() = MatrixPoints_(0, 2 * i);
        (*last_plane)[i].centroid.y() = MatrixPoints_(1, 2 * i);
        (*last_plane)[i].centroid.z() = MatrixPoints_(2, 2 * i);

        (*last_plane)[i].normal.x() = MatrixPoints_(0, 2 * i + 1);
        (*last_plane)[i].normal.y() = MatrixPoints_(1, 2 * i + 1);
        (*last_plane)[i].normal.z() = MatrixPoints_(2, 2 * i + 1);
    }    
}

void MovePlaneUsingIMUdata(const sort_lidarpoints::feature_info FeatureInfo, std::vector<Plane> *last_plane)
{
    Vector6f IMUdata;
    MsgToFeatureInfo(FeatureInfo, &IMUdata);
    Eigen::Matrix4f LidarRotation = To44RT(IMUdata);
    MovePlane(LidarRotation, last_plane);
    
}


int FindSamePlane(Plane &curr_plane, const std::vector<Plane> last_plane)
{
    double Mindis = 100.0;
    int plane_id = 0;
    for(auto i : last_plane){
        
        // Angle between two normal
        float angle = AngleBetweenPlane(i, curr_plane);
        // std::cout << angle << std::endl;
        if(angle < 2.0 || angle > 178.0){

            // Distance to centroid between two plane
            double Dis = PointDistance(curr_plane.centroid, i.centroid);
        
            if(Mindis > Dis){
                Mindis = Dis;
                plane_id = i.id;
            }
        }
    }
        
    if(Mindis < PlaneCentroidDisThres){

        //find
        return plane_id;
    } 
    else    
        return -1;    
}
    
void PlaneCorrespondance(const std::vector<Plane> last_plane, std::vector<Plane> &curr_plane)
{
    for(size_t i = 0; i < curr_plane.size(); i++){
        
        int plane_id = FindSamePlane(curr_plane[i], last_plane);
        if(plane_id == -1){
            // New Plane
            curr_plane[i].id = MaxPlaneId++;
            Planes.push_back(curr_plane[i]);
        }
        else{
            // Traking ordinary Plane
            curr_plane[i].id = plane_id;
        }
    }
}

int FindSameLine(Line &curr_line, const std::vector<Line> last_line)
{
    double Mindis = 100.0;
    int line_id = 0;
    for(auto i : last_line){

        double Dis = LineToLineDistance(curr_line, i); 
        if(Mindis > Dis){
            Mindis = Dis;
            line_id = i.id;
        }
    }

    if(Mindis < LineToLineDisThres){

        // find
        return line_id;
    }
    else
        return -1;
    
}

void LineCorrespondance(const std::vector<Line> last_line, std::vector<Line> &curr_line)
{
    for(size_t i = 0; i < curr_line.size(); i++){

        int line_id = FindSameLine(curr_line[i], last_line);
        if(line_id == -1){
            // New Line
            curr_line[i].id = MaxLineId++;
            Lines.push_back(curr_line[i]);
        }
        else{
            // Tracking oridinary Line
            curr_line[i].id = line_id;
        }
    }
}

void featureHandler(const sort_lidarpoints::feature_infoConstPtr &FeatureMsg)
{
    std::cout << "Frame Num : " << FrameNum << std::endl;
    
    std::vector<Line> NewLines;
    std::vector<Plane> NewPlanes;

    // Convert from ROS Msg
    FeatureInfo = *FeatureMsg;
    ConvertfromRosMsg(FeatureMsg, NewLines, NewPlanes);
    std::cout << " feature info : " << FeatureInfo.gyro_x << std::endl;

    // Initial Id
    if(FrameNum == 0)
        Initialization(NewLines, NewPlanes);
    else{
        // Tracking
        // IMU Rotation data
        // VisualizePlane(VisualPlane, LastPlanes);
        MovePlaneUsingIMUdata(FeatureInfo, &LastPlanes);
        
        
        PlaneCorrespondance(LastPlanes, NewPlanes);
        LineCorrespondance(LastLines, NewLines);
    }

        for(auto i : NewPlanes){
            std::cout << i.id << " ";
        }
        std::cout << std::endl;
        
        // Visual Feature
        VisualizeLine(VisualLine, NewLines);
        VisualizePlane(VisualPlane, NewPlanes);
        // VisualizePlane(VisualPlane2, LastPlanes);
        VisualizePlaneNormal(VisualArrow, NewPlanes);

        // Prepare Next Frame
        CopyNewFeature(NewLines, NewPlanes);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "aFeatureTracking");
    ros::NodeHandle nh("~");

    subFeature = nh.subscribe<sort_lidarpoints::feature_info>("/featureInfo", 100, featureHandler);

    VisualLine.reset(new rviz_visual_tools::RvizVisualTools( LidarFrame, "/line"));
    VisualLine->loadMarkerPub();

    VisualArrow.reset(new rviz_visual_tools::RvizVisualTools( LidarFrame, "/planeNormal"));
    VisualArrow->loadMarkerPub();
    
    VisualPlane.reset(new rviz_visual_tools::RvizVisualTools( LidarFrame, "/plane"));
    VisualPlane->loadMarkerPub();

    VisualPlane2.reset(new rviz_visual_tools::RvizVisualTools( LidarFrame, "/plane2"));
    VisualPlane2->loadMarkerPub();

    ros::spin();

    return 0;
}
    
    


     





