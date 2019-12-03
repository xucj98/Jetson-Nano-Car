#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

static cv::Mat rimg_l, Q_l;
static pcl::visualization::CloudViewer viewer_l("Left Camera");
static cv::Mat rimg_r, Q_r;
// static pcl::visualization::CloudViewer viewer_r("Right Camera");


void rimg_l_callback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    rimg_l = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

void rimg_r_callback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    rimg_r = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

void dprt_l_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat dprt;
    dprt = cv_bridge::toCvCopy(msg, "16SC1")->image;
    dprt.convertTo(dprt, CV_32F, 1.0/16);    
    
    cv::Mat dprt_u8 = cv::Mat(dprt.rows, dprt.cols, CV_8UC1);
    normalize(dprt, dprt_u8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("dprt", dprt_u8);
    cv::waitKey(1);

    cv::Mat xyz;
    if (Q_l.size() != cv::Size(4, 4))
        return;
    cv::reprojectImageTo3D(dprt, xyz, Q_l, true);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p;
    int cnt = 0;
    for (int i=0; i<xyz.rows; i++)
        for (int j=0; j<xyz.cols; j++)
        {
            cv::Vec3f pos = xyz.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = rimg_l.at<cv::Vec3b>(i, j);
            if (!isfinite(pos[2]) || pos[2] >= 10000) 
            {
                cnt ++;
                continue;
            }
            p.x = (float)pos[0]/1000;
            p.y = -(float)pos[1]/1000;
            p.z = -(float)pos[2]/1000;
            // std::cout << pos << ' ' << bgr << std::endl;
            p.b = bgr[0];
            p.g = bgr[1];
            p.r = bgr[2];
            cloud->points.push_back(p);
        }
    std::cout << cnt << std::endl;
    viewer_l.showCloud(cloud);

    cv::imshow("rimg_l", rimg_l);
    cv::waitKey(1);
}

void dprt_r_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat dprt;
    dprt = cv_bridge::toCvCopy(msg, "16SC1")->image;
    dprt.convertTo(dprt, CV_32F, 1.0/16);    
    cv::Mat xyz;
    if (Q_r.size() != cv::Size(4, 4))
        return;
    cv::reprojectImageTo3D(dprt, xyz, Q_r, true);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p;
    int cnt = 0;
    for (int i=0; i<xyz.rows; i++)
        for (int j=0; j<xyz.cols; j++)
        {
            cv::Vec3f pos = xyz.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = rimg_l.at<cv::Vec3b>(i, j);
            if (!isfinite(pos[2]) || pos[2] >= 10000) 
            {
                cnt ++;
                continue;
            }
            p.x = (float)pos[0]/1000;
            p.y = -(float)pos[1]/1000;
            p.z = -(float)pos[2]/1000;
            // std::cout << pos << ' ' << bgr << std::endl;
            p.b = bgr[0];
            p.g = bgr[1];
            p.r = bgr[2];
            cloud->points.push_back(p);
        }
    std::cout << cnt << std::endl;
    // viewer_r.showCloud(cloud);
}

void Q_l_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Q_l = cv_bridge::toCvCopy(msg, "64FC1")->image;
}

void Q_r_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Q_r = cv_bridge::toCvCopy(msg, "64FC1")->image;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "host_cpp_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    // viewer_l.runOnVisualizationThreadOnce();

    ros::Subscriber rimg_l_sub = nh.subscribe("camera/left/rectify_img/compressed", 1, rimg_l_callback);
    ros::Subscriber dprt_l_sub = nh.subscribe("camera/left/disparity", 1, dprt_l_callback);
    ros::Subscriber Q_l_sub = nh.subscribe("camera/left/Q", 1, Q_l_callback);
  
    /*  
    ros::Subscriber rimg_r_sub = nh.subscribe("camera/right/rectify_img/compressed", 1, rimg_l_callback);
    ros::Subscriber dprt_r_sub = nh.subscribe("camera/right/disparity", 1, dprt_l_callback);
    ros::Subscriber Q_r_sub = nh.subscribe("camera/right/Q", 1, Q_l_callback);
    */

    ros::spin();
    
}
