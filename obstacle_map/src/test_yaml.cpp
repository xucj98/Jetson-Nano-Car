#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static YAML::Node cam_cfg_l;

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "obstacle_map_node");
    ros::NodeHandle nh;
    /*
    image_transport::ImageTransport it(nh);
    
    cam_cfg_l = YAML::LoadFile(ros::package::getPath("camera") + "/cfg/stereo_1.yaml");
    cv::Mat distortion_l = cam_cfg_l["DISTORTION_L"].as<cv::Mat>();
    std::cout << distortion_l << std::endl;
    std::cout << distortion_l.size() << std::endl;
    std::cout << distortion_l.type() << std::endl;
    */
    YAML::Node node = YAML::Load("WIDTH: 640\nHEIGHT: 480");
    std::cout << node["HEIGHT"].as<int>();
}

namespace YAML
{
template<>
struct convert<cv::Mat>
{
    static Node encode(const cv::Mat &rhs)
    {
        assert(rhs.type() == CV_32FC1);
        Node node;
        for (int i=0; i<rhs.rows; i++)
        {
            Node row;
            for (int j=0; j<rhs.cols; j++)
                row.push_back(rhs.at<float>(i, j));
            node.push_back(row);
        }
    }
    static bool decode(const Node& node, cv::Mat& rhs)
    {
        int rows = node.size();
        int cols = node[0].size();
        rhs.create(rows, cols, CV_32FC1);
        for (int i=0; i<rows; i++)
            for (int j=0; j<cols; j++)
                rhs.at<float>(i, j) = node[i][j].as<float>();
        return true;
    }    
};
}
