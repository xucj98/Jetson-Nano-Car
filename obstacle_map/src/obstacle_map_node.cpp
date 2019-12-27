#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static image_transport::Publisher dprt_map_l_pub;
static image_transport::Publisher Q_l_pub;
static image_transport::Publisher rimg_l_pub;
static image_transport::Publisher xyz_l_pub;
static ros::Publisher depth_l_pub;
static image_transport::Publisher dprt_map_r_pub;
static image_transport::Publisher Q_r_pub;
static image_transport::Publisher rimg_r_pub;
static image_transport::Publisher xyz_r_pub;
static ros::Publisher depth_r_pub;

static YAML::Node cam_cfg_l;
static YAML::Node cam_cfg_r;

namespace YAML
{
template<>
struct convert<cv::Mat>
{
    static Node encode(const cv::Mat &rhs)
    {
        assert(rhs.type() == CV_64FC1);
        Node node;
        for (int i=0; i<rhs.rows; i++)
        {
            Node row;
            for (int j=0; j<rhs.cols; j++)
                row.push_back(rhs.at<double>(i, j));
            node.push_back(row);
        }
    }
    static bool decode(const Node& node, cv::Mat& rhs)
    {
        int rows = node.size();
        int cols = node[0].size();
        rhs.create(rows, cols, CV_64FC1);
        for (int i=0; i<rows; i++)
            for (int j=0; j<cols; j++)
                rhs.at<double>(i, j) = node[i][j].as<double>();
        return true;
    }    
};
}

void left_cam_callback(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time begin = ros::Time::now();
    
    // transform image message to image
    cv::Mat img;
    img = cv_bridge::toCvShare(msg, "bgr8")->image;

    int height = img.rows;
    int width = img.cols / 2;
    cv::Mat img_l = img(cv::Rect(0, 0, width, height));
    cv::Mat img_r = img(cv::Rect(width, 0, width, height));

    // get parameters from config object
    cv::Mat dist_l = cam_cfg_l["DISTORTION_L"].as<cv::Mat>();
    cv::Mat dist_r = cam_cfg_l["DISTORTION_R"].as<cv::Mat>();
    cv::Mat intr_l = cam_cfg_l["INTRINSICS_L"].as<cv::Mat>();
    cv::Mat intr_r = cam_cfg_l["INTRINSICS_R"].as<cv::Mat>();
    double tf_x = (double)width / cam_cfg_l["WIDTH"].as<int>();
    double tf_y = (double)height / cam_cfg_l["HEIGHT"].as<int>();
    intr_l.at<double>(0, 0) *= tf_x;
    intr_l.at<double>(0, 2) *= tf_x;
    intr_l.at<double>(1, 1) *= tf_y;
    intr_l.at<double>(1, 2) *= tf_y; 
    intr_r.at<double>(0, 0) *= tf_x;
    intr_r.at<double>(0, 2) *= tf_x;
    intr_r.at<double>(1, 1) *= tf_y;
    intr_r.at<double>(1, 2) *= tf_y; 
    cv::Mat rot = cam_cfg_l["ROTATION"].as<cv::Mat>();
    cv::Mat trans = cam_cfg_l["TRANSLATION"].as<cv::Mat>();

    // stereo rectify
    cv::Rect valid_roi_l, valid_roi_r;
    cv::Mat rot_l, rot_r, proj_l, proj_r, Q;
    cv::Mat map_lx, map_ly, map_rx, map_ry;
    cv::Mat rimg_l, rimg_r;    
    cv::stereoRectify(intr_l, dist_l, intr_r, dist_r, img_l.size(), rot, trans, rot_l, rot_r, proj_l, proj_r, Q, cv::CALIB_ZERO_DISPARITY, 1, img_l.size(), &valid_roi_l, &valid_roi_r);
    cv::initUndistortRectifyMap(intr_l, dist_l, rot_l, proj_l, img_l.size(), CV_32FC1, map_lx, map_ly);
    cv::initUndistortRectifyMap(intr_r, dist_r, rot_r, proj_r, img_r.size(), CV_32FC1, map_rx, map_ry);
    cv::remap(img_l, rimg_l, map_lx, map_ly, cv::INTER_LINEAR);
    cv::remap(img_r, rimg_r, map_rx, map_ry, cv::INTER_LINEAR);
    sensor_msgs::ImagePtr Q_msg = cv_bridge::CvImage(msg->header, "64FC1", Q).toImageMsg();
    Q_l_pub.publish(Q_msg);
    sensor_msgs::ImagePtr rimg_msg = cv_bridge::CvImage(msg->header, "bgr8", rimg_l).toImageMsg();
    rimg_l_pub.publish(rimg_msg);
    // cv::imshow("rimg_l", rimg_l);
    // cv::imshow("rimg_r", rimg_r);
    // cv::waitKey(1);

    // compute disparity map
    cv::Mat dprt;

    int mindisparity = 0;
    int ndisparities = 64;
    int SADWindowSize = 9;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
    sgbm->setP1(8 * img_l.channels() * SADWindowSize * SADWindowSize);
    sgbm->setP2(32 * img_l.channels() * SADWindowSize * SADWindowSize);
    sgbm->setPreFilterCap(15);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(2);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(1);
    sgbm->compute(img_l, img_r, dprt);
    // publish dprt
    sensor_msgs::ImagePtr dprt_map_msg = cv_bridge::CvImage(msg->header, "16SC1", dprt).toImageMsg();
    dprt_map_l_pub.publish(dprt_map_msg); 

    /*
    int min_=10000, max_=-10000;
    for (int i=0; i<dprt.rows; i++)
        for (int j=0; j<dprt.cols; j++)
        {
            if (dprt.at<short>(i, j) < min_)
                min_ = dprt.at<short>(i, j);
            if (dprt.at<short>(i, j) > max_)
                max_ = dprt.at<short>(i, j);
        }
    std::cout << min_ << ' ' << max_ << std::endl; 
    */    
    dprt.convertTo(dprt, CV_32F, 1.0/16);
    
    // publish dprt_u8
    // cv::Mat dprt_u8 = cv::Mat(dprt.rows, dprt.cols, CV_8UC1);
    // normalize(dprt, dprt_u8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    // sensor_msgs::ImagePtr dprt_map_msg = cv_bridge::CvImage(msg->header, "mono8", dprt_u8).toImageMsg();
    // dprt_map_l_pub.publish(dprt_map_msg); 
    // cv::imshow("dprt", dprt_u8);
    // cv::waitKey(1);
     
    // reproject image to 3D
    cv::Mat xyz;
    cv::reprojectImageTo3D(dprt, xyz, Q, true);
    sensor_msgs::ImagePtr xyz_msg = cv_bridge::CvImage(msg->header, "32FC3", xyz).toImageMsg();
    xyz_l_pub.publish(xyz_msg);

    static float depth[100000];
    int depth_cnt = 0;
    for (int i=0; i<xyz.rows; i++)
	for (int j=0; j<xyz.cols; j++)
	{    
	    cv::Vec3f pos = xyz.at<cv::Vec3f>(i, j);
            if (!isfinite(pos[2]) || pos[2] >= 100000)
	        continue;
	    depth[depth_cnt] = pos[2];
	    depth_cnt ++;
	} 
    std::sort(depth, depth + depth_cnt);
    std::cout << "left:                " << depth_cnt << ' ' << depth[10000] << std::endl;

    std_msgs::Int16 depth_msg;
    depth_msg.data = depth[5000];
    depth_l_pub.publish(depth_msg);
    // checkresult    
    // cv::circle(rimg_l, cv::Point(160, 120), 2, CV_RGB(255, 255, 0), 2);
    // cv::circle(rimg_l, cv::Point(80, 120), 2, CV_RGB(255, 255, 0), 2);
    // cv::imshow("check", rimg_l);
    // std::cout << xyz.at<cv::Vec3f>(120, 160) << ' ' << xyz.at<cv::Vec3f>(120, 80) << std::endl; 
    
    // std::cout << (ros::Time::now() - begin).toSec() << std::endl;    
}

void right_cam_callback(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time begin = ros::Time::now();
    
    // transform image message to image
    cv::Mat img;
    img = cv_bridge::toCvShare(msg, "bgr8")->image;

    int height = img.rows;
    int width = img.cols / 2;
    cv::Mat img_l = img(cv::Rect(0, 0, width, height));
    cv::Mat img_r = img(cv::Rect(width, 0, width, height));

    // get parameters from config object
    cv::Mat dist_l = cam_cfg_r["DISTORTION_L"].as<cv::Mat>();
    cv::Mat dist_r = cam_cfg_r["DISTORTION_R"].as<cv::Mat>();
    cv::Mat intr_l = cam_cfg_r["INTRINSICS_L"].as<cv::Mat>();
    cv::Mat intr_r = cam_cfg_r["INTRINSICS_R"].as<cv::Mat>();
    double tf_x = (double)width / cam_cfg_r["WIDTH"].as<int>();
    double tf_y = (double)height / cam_cfg_r["HEIGHT"].as<int>();
    intr_l.at<double>(0, 0) *= tf_x;
    intr_l.at<double>(0, 2) *= tf_x;
    intr_l.at<double>(1, 1) *= tf_y;
    intr_l.at<double>(1, 2) *= tf_y; 
    intr_r.at<double>(0, 0) *= tf_x;
    intr_r.at<double>(0, 2) *= tf_x;
    intr_r.at<double>(1, 1) *= tf_y;
    intr_r.at<double>(1, 2) *= tf_y; 
    cv::Mat rot = cam_cfg_r["ROTATION"].as<cv::Mat>();
    cv::Mat trans = cam_cfg_r["TRANSLATION"].as<cv::Mat>();

    // stereo rectify
    cv::Rect valid_roi_l, valid_roi_r;
    cv::Mat rot_l, rot_r, proj_l, proj_r, Q;
    cv::Mat map_lx, map_ly, map_rx, map_ry;
    cv::Mat rimg_l, rimg_r;    
    cv::stereoRectify(intr_l, dist_l, intr_r, dist_r, img_l.size(), rot, trans, rot_l, rot_r, proj_l, proj_r, Q, cv::CALIB_ZERO_DISPARITY, 1, img_l.size(), &valid_roi_l, &valid_roi_r);
    cv::initUndistortRectifyMap(intr_l, dist_l, rot_l, proj_l, img_l.size(), CV_32FC1, map_lx, map_ly);
    cv::initUndistortRectifyMap(intr_r, dist_r, rot_r, proj_r, img_r.size(), CV_32FC1, map_rx, map_ry);
    cv::remap(img_l, rimg_l, map_lx, map_ly, cv::INTER_LINEAR);
    cv::remap(img_r, rimg_r, map_rx, map_ry, cv::INTER_LINEAR);
    sensor_msgs::ImagePtr Q_msg = cv_bridge::CvImage(msg->header, "64FC1", Q).toImageMsg();
    Q_r_pub.publish(Q_msg);
    sensor_msgs::ImagePtr rimg_msg = cv_bridge::CvImage(msg->header, "bgr8", rimg_l).toImageMsg();
    rimg_r_pub.publish(rimg_msg);
    // cv::imshow("rimg_l", rimg_l);
    // cv::imshow("rimg_r", rimg_r);
    // cv::waitKey(1);

    // compute disparity map
    cv::Mat dprt;

    int mindisparity = 0;
    int ndisparities = 64;
    int SADWindowSize = 9;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
    sgbm->setP1(8 * img_l.channels() * SADWindowSize * SADWindowSize);
    sgbm->setP2(32 * img_l.channels() * SADWindowSize * SADWindowSize);
    sgbm->setPreFilterCap(15);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(2);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(1);
    sgbm->compute(img_l, img_r, dprt);
    // publish dprt
    sensor_msgs::ImagePtr dprt_map_msg = cv_bridge::CvImage(msg->header, "16SC1", dprt).toImageMsg();
    dprt_map_r_pub.publish(dprt_map_msg); 

    /*
    int min_=10000, max_=-10000;
    for (int i=0; i<dprt.rows; i++)
        for (int j=0; j<dprt.cols; j++)
        {
            if (dprt.at<short>(i, j) < min_)
                min_ = dprt.at<short>(i, j);
            if (dprt.at<short>(i, j) > max_)
                max_ = dprt.at<short>(i, j);
        }
    std::cout << min_ << ' ' << max_ << std::endl; 
    */    
    dprt.convertTo(dprt, CV_32F, 1.0/16);
    
    // publish dprt_u8
    // cv::Mat dprt_u8 = cv::Mat(dprt.rows, dprt.cols, CV_8UC1);
    // normalize(dprt, dprt_u8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    // sensor_msgs::ImagePtr dprt_map_msg = cv_bridge::CvImage(msg->header, "mono8", dprt_u8).toImageMsg();
    // dprt_map_l_pub.publish(dprt_map_msg); 
    // cv::imshow("dprt", dprt_u8);
    // cv::waitKey(1);
     
    // reproject image to 3D
    cv::Mat xyz;
    cv::reprojectImageTo3D(dprt, xyz, Q, true);
    sensor_msgs::ImagePtr xyz_msg = cv_bridge::CvImage(msg->header, "32FC3", xyz).toImageMsg();
    xyz_r_pub.publish(xyz_msg);

    static float depth[100000];
    int depth_cnt = 0;
    for (int i=0; i<xyz.rows; i++)
	for (int j=0; j<xyz.cols; j++)
	{    

	    cv::Vec3f pos = xyz.at<cv::Vec3f>(i, j);
            if (!isfinite(pos[2]) || pos[2] >= 100000)
	        continue;
	    depth[depth_cnt] = pos[2];
	    depth_cnt ++;
	} 
    std::sort(depth, depth + depth_cnt);
    std::cout << "right: " << depth_cnt << ' ' << depth[10000] << std::endl;

    std_msgs::Int16 depth_msg;
    depth_msg.data = depth[5000];
    depth_r_pub.publish(depth_msg);
    // checkresult    
    // cv::circle(rimg_l, cv::Point(160, 120), 2, CV_RGB(255, 255, 0), 2);
    // cv::circle(rimg_l, cv::Point(80, 120), 2, CV_RGB(255, 255, 0), 2);
    // cv::imshow("check", rimg_l);
    // std::cout << xyz.at<cv::Vec3f>(120, 160) << ' ' << xyz.at<cv::Vec3f>(120, 80) << std::endl; 
    
    // std::cout << (ros::Time::now() - begin).toSec() << std::endl;
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_map_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    cam_cfg_l = YAML::LoadFile(ros::package::getPath("camera") + "/cfg/stereo_1.yaml");
    cam_cfg_r = YAML::LoadFile(ros::package::getPath("camera") + "/cfg/stereo_2.yaml");

    ros::Subscriber cam_l_sub = nh.subscribe("camera/left", 1, left_cam_callback);
    ros::Subscriber cam_r_sub = nh.subscribe("camera/right", 1, right_cam_callback);
    
    dprt_map_l_pub = it.advertise("camera/left/disparity", 2);
    rimg_l_pub = it.advertise("camera/left/rectify_img", 2);
    xyz_l_pub = it.advertise("camera/left/xyz", 2);
    Q_l_pub = it.advertise("camera/left/Q", 2);
    depth_l_pub = nh.advertise<std_msgs::Int16>("camera/left/depth", 2);

    dprt_map_r_pub = it.advertise("camera/right/disparity", 2);
    rimg_r_pub = it.advertise("camera/right/rectify_img", 2);
    xyz_r_pub = it.advertise("camera/right/xyz", 2);
    Q_r_pub = it.advertise("camera/right/Q", 2);
    depth_r_pub = nh.advertise<std_msgs::Int16>("camera/right/depth", 2);

    // ros::Publisher dprt_map_l_cmprs_pub = nh.advertise<sensor_msgs::CompressedImage>("camera/left/disparity/compressed", 2);

    ros::spin();
}
