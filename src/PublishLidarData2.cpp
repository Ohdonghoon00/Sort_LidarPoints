#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <sstream>
 
#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <glog/logging.h>



ros::Publisher pubLaserCloud;


using namespace pcl;
using namespace std;

namespace po = boost::program_options;

const std::vector<float> imu2rig_pose = {-0.011773881878296,-2.212344247385963,2.229193892963689,-0.016975989407230,0.016444757006134,0.128779023189435};
const std::vector<float> lidar2rig_pose = {1.5620435019860173, -0.005377623186353324, 0.003014408980859652, -8.458334129298635E-4, -0.19542397891778734, -0.0012719333618026098};
static bool isBigEndian()
{
    volatile int num = 1;
    return *((char*) &num) == ((char) 1);
}

static const bool IS_BIG_ENDIAN = isBigEndian();
struct LidarData 
{

    std::vector<char> binary_data;
    int64_t timestamp_ns;
    int num_points, num_blocks;
    uint8_t num_channels;

    
    LidarData()
        : num_points(0), num_blocks(0), num_channels(0) {}
    virtual ~LidarData() {}
    
    
    float* points_ptr() const { return (float*) binary_data.data(); }
    uint8_t* intensities_ptr() const { return (uint8_t*) &binary_data[3 * num_points * sizeof(float)]; } // reflectivity
    uint8_t* azimuth_idxs_ptr() const { return intensities_ptr() + num_points; }
    float* azimuth_degs_ptr() const { return (float*) (azimuth_idxs_ptr() + num_points); }
    
     

};

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

Eigen::Matrix4f To44RT(std::vector<float> rot)
{

    cv::Mat R( 1, 3, CV_32FC1);
    R.at<float>(0, 0) = rot[0];
    R.at<float>(0, 1) = rot[1];
    R.at<float>(0, 2) = rot[2];

    cv::Rodrigues(R, R);

    Eigen::Matrix4f RT;
    RT << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), rot[3],
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), rot[4],
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), rot[5],
                0,                 0,                   0,                  1;

    return RT;
}

Eigen::Matrix4f gyroToRotation(Eigen::Vector3f gyro)
{
    float t = 0.005; // 200Hz
    float angle_x(gyro[0] * t), angle_y(gyro[1] * t), angle_z(gyro[2] * t);  
    
    float data_x[] = {  1.0, 0.0, 0.0,
                        0.0, cos(angle_x), sin(angle_x),
                        0.0, -sin(angle_x), cos(angle_x)};

    float data_y[] = {  cos(angle_y), 0.0, -sin(angle_y),
                        0.0, 1.0, 0.0,
                        sin(angle_y), 0.0, cos(angle_y)};

    float data_z[] = {  cos(angle_z), sin(angle_z), 0.0,
                        -sin(angle_z), cos(angle_z), 0.0,
                        0.0, 0.0, 1.0};

    cv::Mat R_x( 3, 3, CV_32FC1, data_x);
    cv::Mat R_y( 3, 3, CV_32FC1, data_y);
    cv::Mat R_z( 3, 3, CV_32FC1, data_z);
    cv::Mat R = R_x * R_y * R_z;




    Eigen::Matrix4f RT;
    RT << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), 0,
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), 0,
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), 0,
                0,                  0,                  0,              1;

    return RT;
}

float ToAngle(Eigen::Matrix4f LidarRotation)
{
    float data[] = {    LidarRotation(0, 0), LidarRotation(0, 1), LidarRotation(0, 2),
                        LidarRotation(1, 0), LidarRotation(1, 1), LidarRotation(1, 2),
                        LidarRotation(2, 0), LidarRotation(2, 1), LidarRotation(2, 2)};

    
    cv::Mat rot(3, 3, CV_32FC1, data);
    cv::Rodrigues(rot, rot);
    float angle = sqrt( rot.at<float>(0, 0) * rot.at<float>(0, 0) + 
                        rot.at<float>(1, 0) * rot.at<float>(1, 0) +
                        rot.at<float>(2, 0) * rot.at<float>(2, 0) );

    return angle;
}

Eigen::Vector3f ToAxis(Eigen::Matrix4f LidarRotation)
{
    float angle = ToAngle(LidarRotation);
    float data[] = {    LidarRotation(0, 0), LidarRotation(0, 1), LidarRotation(0, 2),
                        LidarRotation(1, 0), LidarRotation(1, 1), LidarRotation(1, 2),
                        LidarRotation(2, 0), LidarRotation(2, 1), LidarRotation(2, 2)};

    
    cv::Mat rot(3, 3, CV_32FC1, data);
    cv::Rodrigues(rot, rot);

    Eigen::Vector3f Axis;
    Axis << rot.at<float>(0, 0), rot.at<float>(1, 0), rot.at<float>(2, 0);
    Axis = Axis / angle;

    return Axis;

}

void MoveDistortionPoints(std::vector<Eigen::Vector3f> &points, Eigen::Matrix4f LidarRotation, int ScanStepNum, int num_seqs)
{
    int PointNum = points.size();
    Eigen::MatrixXf MatrixPoints(4, PointNum);
    for(int i = 0; i < PointNum; i++){
        MatrixPoints(0, i) = points[i](0);
        MatrixPoints(1, i) = points[i](1);
        MatrixPoints(2, i) = points[i](2);
        MatrixPoints(3, i) = 1.0;
    }
    
    float angle = ToAngle(LidarRotation);
    Eigen::Vector3f Axis = ToAxis(LidarRotation);
    
    float AngleRatio = (((float)(ScanStepNum + 1) / (float)num_seqs) * angle);
    Axis = Axis * AngleRatio;
    
    float data[] = {Axis(0, 0), Axis(1, 0), Axis(2, 0)};
    cv::Mat R(3, 1, CV_32FC1, data);
    cv::Rodrigues(R, R);

    Eigen::Matrix4f RT;
    RT <<   R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), 0,
            R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), 0,
            R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), 0,
            0,                  0,                  0,              1;


    Eigen::Matrix4Xf MatrixPoints_ = RT.inverse() * MatrixPoints;
    points.clear();
    for(int i = 0; i < PointNum; i++){
        Eigen::Vector3f Point;
        Point(0) = MatrixPoints_(0, i);
        Point(1) = MatrixPoints_(1, i);
        Point(2) = MatrixPoints_(2, i);
        points.push_back(Point);
    }

}

int main(int argc, char **argv) 
{
    
    ros::init(argc, argv, "TestPublishData");
    ros::NodeHandle nh("~");
    LidarData lidar_data;
    
    // launch parameter
    rosbag::Bag bag;
    bool to_bag, ToUndistortionPoints;
    std::string data_dir, output_bag_file, yaml_path;
    int publish_delay;
    
    nh.getParam("data_dir", data_dir);
    nh.getParam("to_bag", to_bag);
    nh.getParam("ToUndistortionPoints", ToUndistortionPoints);
    nh.getParam("publish_delay", publish_delay);
    
    // Read Yaml setting file


    // Extrinsic parameter rig - imu / rig - lidar
    const Eigen::Matrix4f RigToIMU = To44RT(imu2rig_pose);
    const Eigen::Matrix4f RigToLidar = To44RT(lidar2rig_pose);
    
    
    
    Eigen::Matrix4f LidarRotation;
    
    
    
    
    // bagfile
    if (to_bag){
        nh.getParam("output_bag_file", output_bag_file);
        bag.open(data_dir + output_bag_file, rosbag::bagmode::Write);
    }
    
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    // publish delay
    ros::Rate r(10.0 / publish_delay);

    
    // Lidar timestamp.csv path
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::ifstream LidarcsvFile(LidarcsvPath, std::ifstream::in);

    if(!LidarcsvFile.is_open()){
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    // IMU csv data path
    std::string IMUcsvPath = data_dir + "imu_data.csv";
    std::ifstream IMUcsvFile(IMUcsvPath, std::ifstream::in);

    if(!IMUcsvFile.is_open()){
        std::cout << " IMU csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }



    
    std::string Lidarcsvline, IMUcsvline;
    int Lidarline_num = 0;
    int IMUline_num = 0;
    
    std::vector<double> IMUtimestamps;
    vector<Eigen::Vector3f> IMUGyros;
    
    // Read IMU Data csv
    while(std::getline(IMUcsvFile, IMUcsvline) && ros::ok())
    {
        if(IMUline_num == 0){
            IMUline_num++;
            continue;
        }
            
        std::string IMUvalue;
        std::vector<std::string> IMUvalues;
            
        // IMUvalues[0] -> Timestamp (ns)
        // IMUvalues[1] -> Gyro_x
        // IMUvalues[2] -> Gyro_y
        // IMUvalues[3] -> Gyro_z
        // IMUvalues[4] -> Acc_x
        // IMUvalues[5] -> Acc_y
        // IMUvalues[6] -> Acc_z

        std::stringstream ss(IMUcsvline);
        while(std::getline(ss, IMUvalue, ','))
            IMUvalues.push_back(IMUvalue);
        // std::cout << " IMUline num : " << IMUline_num << "th    ";
            
        IMUtimestamps.push_back(std::stod(IMUvalues[0]));
        // std::cout.precision(15);
        // std::cout << " IMU timestamp : " << IMUtimestamp << std::endl;

        Eigen::Vector3f Gyro;
        Gyro.resize(3);
        Gyro[0] = std::stof(IMUvalues[1]);
        Gyro[1] = std::stof(IMUvalues[2]);
        Gyro[2] = std::stof(IMUvalues[3]);
        IMUGyros.push_back(Gyro);
         
             
            
        std::cout << std::endl;
        IMUline_num++;
    } 

    IMUcsvFile.close();
       

    int IMUcount = 0;
    // Read Lidar timestamp.csv
    while(std::getline(LidarcsvFile, Lidarcsvline) && ros::ok())
    {
        if(Lidarline_num == 0){
            Lidarline_num++;
            continue;
        }
        
        std::string value;
        std::vector<std::string> values;
        
        // values[0] -> First Seq Timestamp (ns)
        // values[1] -> Last Seq Timestamp (ns)
        // values[2] -> Fidx
        // values[3] -> Num pts
        // values[4] -> Date
        
        std::stringstream ss(Lidarcsvline);
        while(std::getline(ss, value, ','))
            values.push_back(value);
        int fidx = std::stoi(values[2]);
        double LidarScantimestamp = std::stod(values[1]);

        // Binary Data Path
        std::stringstream Lidar_binary_path;
        Lidar_binary_path <<    data_dir + "lidar/" << 
                                std::setfill('0') << 
                                std::setw(5) << fidx << ".xyz";
        
        std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
        
        if (!ifs.is_open()){
            std::cout << "xyz file failed to open: " << std::endl;
            return EXIT_FAILURE;
        }        

        std::vector<Eigen::Vector3f> PublishPoints;
        // const size_t kMaxNumberOfPoints = 1e6; 
        // PublishPoints.clear();
        // PublishPoints.reserve(kMaxNumberOfPoints);
        
        
        std::cout << " File number : " << fidx << "     " << std::endl;
        
        // Read Binary data file
        int num_seqs = 0;
        ifs.read((char*)&num_seqs, sizeof(int));
        std::cout << " num_seqs : " << num_seqs << std::endl;

         
        // Integral IMU rotation to one lidar scan
        Eigen::Matrix4f IMURotation_integral = Eigen::Matrix4f::Identity();
        while(LidarScantimestamp > IMUtimestamps[IMUcount]){
            
            Eigen::Matrix4f IMURotation = gyroToRotation(IMUGyros[IMUcount]);
            IMURotation_integral = IMURotation * IMURotation_integral;
            IMUcount++;
        }

        Eigen::Matrix4f RT_ = RigToIMU * IMURotation_integral * RigToIMU.inverse();
        Eigen::Matrix4f RT = RigToLidar.inverse() * RT_ * RigToLidar;
        LidarRotation = RT;

        for (int j = 0; j < num_seqs; j++){

            Eigen::Vector3f point;
            std::vector<Eigen::Vector3f> Points;
            
            
            int& num_pts = lidar_data.num_points;
            ifs.read((char*)&num_pts, sizeof(int));
            
            lidar_data.binary_data.resize((4 * num_pts) * sizeof(float) + 2 * num_pts);
            

            ifs.read((char*) lidar_data.points_ptr(), num_pts * sizeof(float) * 3);
            ifs.read((char*) lidar_data.intensities_ptr(), num_pts );
            
            ifs.read((char*) lidar_data.azimuth_idxs_ptr(), num_pts);    
            ifs.read((char*) &lidar_data.num_blocks, sizeof(int));
            CHECK_LE(lidar_data.num_blocks, num_pts);  
            ifs.read((char*) lidar_data.azimuth_degs_ptr(),
                    lidar_data.num_blocks * sizeof(float));
            ifs.read((char*) &lidar_data.num_channels, sizeof(uint8_t));
            ifs.read((char*) &lidar_data.timestamp_ns, sizeof(int64_t));


            // save 3D points and intensity 
            for(int k = 0; k < num_pts * 3; k+=3){
                point(0) = *(lidar_data.points_ptr() + k);
                point(1) = *(lidar_data.points_ptr() + k + 1);
                point(2) = *(lidar_data.points_ptr() + k + 2);
                // point.intensity = (((float)*( lidar_data.intensities_ptr() + (k/3) ) ) / 255); // 0 ~ 1 , raw data : 0 ~ 254
                Points.push_back(point);
            }

            // UndistortionPoints
            if(ToUndistortionPoints) MoveDistortionPoints(Points, LidarRotation, j, num_seqs);

            for(size_t i = 0; i < Points.size(); i ++){
                Eigen::Vector3f NoDistortionPoint;
                NoDistortionPoint(0) = Points[i](0);
                NoDistortionPoint(1) = Points[i](1);
                NoDistortionPoint(2) = Points[i](2);
                    
                PublishPoints.push_back(NoDistortionPoint);
            }        

        }
            
        
        
        // timestamp
        ros::Time timestamp_ros;
        timestamp_ros.fromNSec(lidar_data.timestamp_ns);
        std::cout << timestamp_ros << std::endl;

        // publish
        sensor_msgs::PointCloud2 output;
        // pcl::toROSMsg(PublishPoints, output);
        output = ConverToROSmsg(PublishPoints);
        output.header.stamp = timestamp_ros;
        output.header.frame_id = "/camera_init";
        pubLaserCloud.publish(output);

        // bagfile
        if( to_bag ) bag.write("/velodyne_points", timestamp_ros, output);


        Lidarline_num++;
        r.sleep();
        ifs.close();   
        std::cout << "ab" << std::endl;
    }

    LidarcsvFile.close();
            
    std::cout << std::endl;
    if (to_bag) bag.close();
    
    std::cout << "Publish Lidar Data Done ..." << std::endl;
    
    return 0;
}
            
    

       
            

            


    







             

    



