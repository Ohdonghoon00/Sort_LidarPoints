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


ros::Subscriber subFeature;

rviz_visual_tools::RvizVisualToolsPtr VisualLine;
rviz_visual_tools::RvizVisualToolsPtr VisualPlane;
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
std::string LidarFrame = "/camera_init";

//////////// Parameter ////////////

// plane
double PlaneCentroidDisThres = 3.0;

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
    line.resize(FeatureMsg->p1.size() / 3);
    plane.resize(FeatureMsg->scale.size());
    
    MsgToFeature(FeatureMsg, line, plane);
}

int FindSamePlane(Plane &curr_plane, const std::vector<Plane> last_plane)
{
    double Mindis = 100.0;
    int plane_id = 0;
    for(auto i : last_plane){
        
        double Dis = PointDistance(curr_plane.centroid, i.centroid);
        if(Mindis > Dis){
            Mindis = Dis;
            plane_id = i.id;
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
    ConvertfromRosMsg(FeatureMsg, NewLines, NewPlanes);
    
    // Initial Id
    if(FrameNum == 0)
        Initialization(NewLines, NewPlanes);

    // Tracking
    PlaneCorrespondance(LastPlanes, NewPlanes);
    LineCorrespondance(LastLines, NewLines);

    for(auto i : NewPlanes){
        std::cout << i.id << " ";
    }
    std::cout << std::endl;
    
    // Visual Feature
    VisualizeLine(VisualLine, NewLines);
    VisualizePlane(VisualPlane, NewPlanes);
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


    ros::spin();

    return 0;
}
    
    


     





