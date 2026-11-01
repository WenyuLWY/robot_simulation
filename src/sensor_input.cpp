#include <ros/ros.h>

// ROS messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

// message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Boost
#include <boost/bind/bind.hpp>

// STL
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace RobotSimulation
{  
    using PointT = pcl::PointXYZI;

    class SensorInputNode
    {
        public:
        SensorInputNode()
            :nh(),
            private_nh("~")
        {
            readParameters();

            crop_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
            crop_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
            crop_filter.setNegative(true);
            std::cout<<"[RobotSimulation/SensorInputNode]Crop box: min("<<min_x<<","<<min_y<<","<<min_z<<") max("<<max_x<<","<<max_y<<","<<max_z<<")"<<std::endl;

            tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
            staticBr = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

            if(mode =="2d")
            {
                target_frame_id = robot_base_frame_id;
                pub_odometry_topic = "/odom";
            }
            else if(mode =="3d")
            {
                // target_frame_id = lidar_frame_id;
                target_frame_id = "sensor";
                pub_odometry_topic= "/state_estimation";
                auto start1 = ros::Time::now();
                try
                {
                    tfBaseToLidar = tfBuffer.lookupTransform(
                        robot_base_frame_id,
                        lidar_frame_id,
                        ros::Time(0),
                        ros::Duration(20.0));
                    auto end1 = ros::Time::now();
                    double elapsed1 = (end1 - start1).toSec();
                    tfSensorToVehicle.header.stamp = ros::Time::now();
                    tfSensorToVehicle.header.frame_id = "sensor";
                    tfSensorToVehicle.child_frame_id = "vehicle";
                    double vehicleHeight = tfBaseToLidar.transform.translation.z;
                    tfSensorToVehicle.transform.translation.x = -tfBaseToLidar.transform.translation.x;
                    tfSensorToVehicle.transform.translation.y = -tfBaseToLidar.transform.translation.y;
                    tfSensorToVehicle.transform.translation.z = 0;
                    tfSensorToVehicle.transform.rotation.x = 0.0;
                    tfSensorToVehicle.transform.rotation.y = 0.0;
                    tfSensorToVehicle.transform.rotation.z = 0.0;
                    tfSensorToVehicle.transform.rotation.w = 1.0;

                    geometry_msgs::TransformStamped tfVehicleToBase;
                    tfVehicleToBase.header.stamp = ros::Time::now();
                    tfVehicleToBase.header.frame_id = "vehicle";
                    tfVehicleToBase.child_frame_id = robot_base_frame_id;
                    tfVehicleToBase.transform.translation.z = -vehicleHeight;
                    tfVehicleToBase.transform.rotation.w = 1.0; 

                    staticBr->sendTransform({tfSensorToVehicle, tfVehicleToBase});
                    ROS_INFO("[RobotSimulation/SensorInputNode]TF acquire success! Elapsed time: %.3f s", elapsed1);
                    SensorCorrectMat= tf2::transformToEigen(tfBaseToLidar);
                    SensorCorrectMat.translation().setZero();
                    // SensorCorrectMatInv = SensorCorrectMat.inverse();
                    
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("[RobotSimulation/SensorInputNode]Failed to republish TF: %s", ex.what());
                    auto end1 = ros::Time::now();
                    double elapsed1 = (end1 - start1).toSec();
                    ROS_WARN("[RobotSimulation/SensorInputNode] %s to %s TF not found within %.3f s, skipping.", robot_base_frame_id.c_str(), lidar_frame_id.c_str(), elapsed1);
                }
            }

            if(target_frame_id!=source_frame_id)
                transform_odometry=true;

            if(transform_odometry)
            {
                auto start2 = ros::Time::now();
                try
                {
                    auto tfCorrect=tfBuffer.lookupTransform(
                            target_frame_id,
                            source_frame_id,
                            ros::Time(0),
                            ros::Duration(5.0));
                    auto end1 = ros::Time::now();
                    double elapsed1 = (end1 - start2).toSec();
                    transformCorrectMat = tf2::transformToEigen(tfCorrect);
                    transformCorrectMatInv = transformCorrectMat.inverse();
                    // tf2::fromMsg(tfCorrect.transform, transformCorrectMat);
                    ROS_INFO("[RobotSimulation/SensorInputNode]Correction TF acquire success! Elapsed time: %.3f s", elapsed1);
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("[RobotSimulation/SensorInputNode]Failed to republish TF: %s", ex.what());
                    auto end2 = ros::Time::now();
                    double elapsed2 = (end2 - start2).toSec();
                    ROS_ERROR("[RobotSimulation/SensorInputNode] %s to %s TF not found within %.3f s, Stop.", target_frame_id.c_str(), source_frame_id.c_str(), elapsed2);
                    return;
                }
            }

           
            pubLaserCloudRegistered = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 10);
            pubLaserCloudSensorscan = nh.advertise<sensor_msgs::PointCloud2>("/sensor_scan", 10);
            pubOdometry = nh.advertise<nav_msgs::Odometry>(pub_odometry_topic, 10);
            pubOdometryScan = nh.advertise<nav_msgs::Odometry>("/state_estimation_at_scan", 10);
            pubCropBox = nh.advertise<visualization_msgs::Marker>("/vehicle_box", 1, true);

            


            // pubCropBox.publish(marker);

            subOdometry.subscribe(nh, sub_odometry_topic, 50);
            subLaserCloud.subscribe(nh, cloud_topic, 50);
            sync_.reset(new Sync(syncPolicy(50), subOdometry, subLaserCloud));
            sync_->registerCallback(boost::bind(&SensorInputNode::laserCloudAndOdometryHandler, this, _1, _2));
        }

        ~SensorInputNode()
        {
            std::cerr << "\033[31m"<<"[RobotSimulation/SensorInputNode] stopped"<<"\033[0m" << std::endl;
        }

        private:

        //handlers
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        //params
        std::string  lidar_frame_id;
        std::string  robot_base_frame_id;
        std::string  odom_frame_id;
        std::string  frame_id;
        std::string  cloud_topic;
        std::string  sub_odometry_topic;
        std::string  pub_odometry_topic;
        std::string  mode;
        std::string  source_frame_id;
        std::string  target_frame_id;
        std::string crop_box_str;
        double min_x, max_x, min_y, max_y, min_z, max_z;
        bool pub_registered_cloud = false;
        bool transform_odometry = false;

        //message_filters sync
        message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
        message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> syncPolicy;
        typedef message_filters::Synchronizer<syncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

        //publishers
        ros::Publisher pubOdometry;
        ros::Publisher pubOdometryScan;
        ros::Publisher pubLaserCloudRegistered; 
        ros::Publisher pubLaserCloudSensorscan;
        ros::Publisher pubCropBox;
        tf2_ros::TransformBroadcaster tfBroadcaster;

        tf2_ros::Buffer tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticBr;
        geometry_msgs::TransformStamped  tfBaseToLidar;
        geometry_msgs::TransformStamped  tfSensorToVehicle;
        Eigen::Affine3d transformCorrectMat;
        Eigen::Affine3d transformCorrectMatInv;
        Eigen::Affine3d SensorCorrectMat;

        pcl::CropBox<PointT> crop_filter;

        visualization_msgs::Marker marker;

        void readParameters()
        {
            private_nh.param<std::string>("vehicle_crop_box",crop_box_str, "-0.01 0.01 -0.01 0.01 -0.01 0.01");
            std::stringstream ss(crop_box_str);
            if (!(ss >> min_x >> max_x >> min_y >> max_y >> min_z >> max_z))
            {
                ROS_ERROR("Invalid vehicle_crop_box: %s", crop_box_str.c_str());
            }

            private_nh.param<std::string>("lidar_frame_id", lidar_frame_id, "velodyne");
            private_nh.param<std::string>("robot_base_frame_id", robot_base_frame_id, "base_footprint");
            private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
            private_nh.param<std::string>("sub_odometry_topic", sub_odometry_topic, "/odom");
            // private_nh.param<std::string>("frame_id", frame_id, "velodyne");
            private_nh.param<std::string>("mode", mode, "2d");
            private_nh.param<std::string>("source_frame_id", source_frame_id, "base_link");
            
            private_nh.param<std::string>("raw_cloud_topic", cloud_topic, "/velodyne_points");
        }

        void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometryData,
                                  const sensor_msgs::PointCloud2::ConstPtr& laserCloudIn)
        {

            Eigen::Affine3d transformMat;
            tf2::fromMsg(odometryData->pose.pose, transformMat);
            if(transform_odometry)
                transformMat = transformCorrectMat * transformMat * transformCorrectMatInv;

            nav_msgs::Odometry odom; 
            odom.header = odometryData->header;
            odom.header.frame_id = odom_frame_id;
            odom.pose.pose = tf2::toMsg(transformMat);

            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = odometryData->header.stamp;
            transformStamped.header.frame_id = odom_frame_id;
            transformStamped.transform = tf2::eigenToTransform(transformMat).transform;

            pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
            pcl::fromROSMsg(*laserCloudIn, *cloud_in);
            pcl::transformPointCloud(*cloud_in,*cloud_in,SensorCorrectMat.cast<float>());
            pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>());
            crop_filter.setInputCloud(cloud_in);
            crop_filter.filter(*cloud_crop);
            pcl::transformPointCloud(*cloud_crop,*cloud_crop,transformMat.cast<float>());

            sensor_msgs::PointCloud2 laserCloudRegistered; 
            pcl::toROSMsg(*cloud_crop, laserCloudRegistered);
            laserCloudRegistered.header.stamp = laserCloudIn->header.stamp;
            laserCloudRegistered.header.frame_id = odom_frame_id;

            if(mode =="2d")
            {
                odom.child_frame_id = robot_base_frame_id;
                transformStamped.child_frame_id = robot_base_frame_id;
            }
            else if(mode =="3d")
            {
                odom.child_frame_id = "sensor";
                nav_msgs::Odometry odomScan;
                odomScan.header.stamp = laserCloudIn->header.stamp;
                odomScan.header.frame_id = odom_frame_id;
                odomScan.child_frame_id = "sensor_at_scan";
                odomScan.pose=odom.pose;

                sensor_msgs::PointCloud2 laserCloudScan;
                laserCloudScan=*laserCloudIn;
                laserCloudScan.header.stamp = laserCloudIn->header.stamp;
                laserCloudScan.header.frame_id = "sensor_at_scan";

                //publish
                pubOdometryScan.publish(odomScan);
                pubLaserCloudSensorscan.publish(laserCloudScan);
                
                geometry_msgs::TransformStamped transformStampedScan=transformStamped;
                transformStampedScan.header.stamp = laserCloudIn->header.stamp;
                transformStampedScan.child_frame_id = "sensor_at_scan";
                tfBroadcaster.sendTransform(transformStampedScan);
                transformStamped.child_frame_id = "sensor";
            }

            pubOdometry.publish(odom);
            pubLaserCloudRegistered.publish(laserCloudRegistered);
            tfBroadcaster.sendTransform(transformStamped);


            marker.header.frame_id = "sensor";
            marker.header.stamp = ros::Time(0); //ros::Time::now()
            marker.ns = "vehicle_box";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            // box center
            marker.pose.position.x = (min_x + max_x) / 2.0;
            marker.pose.position.y = (min_y + max_y) / 2.0;
            marker.pose.position.z = (min_z + max_z) / 2.0;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // box size
            marker.scale.x = max_x - min_x;
            marker.scale.y = max_y - min_y;
            marker.scale.z = max_z - min_z;

            // green
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.25;

            marker.lifetime = ros::Duration(0);
            pubCropBox.publish(marker);

            
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_input");

    RobotSimulation::SensorInputNode sensor_input_node;

    ros::spin();

    return 0;
}