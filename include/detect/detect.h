#ifndef RADAR_DETECT_H
#define RADAR_DETECT_H

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.hpp>
#include<opencv2/opencv.hpp>

#include"detect/process.h"

//API文档：rclcpp: rclcpp: ROS Client Library for C++ (ros2.org)

class ImageSubscribe: public rclcpp::Node
{
public:
    ImageSubscribe(std::string name);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscribe;

    void command_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif