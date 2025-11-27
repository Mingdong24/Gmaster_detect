#ifndef RADAR_PROCESS_H
#define RADAR_PROCESS_H

#include<opencv2/opencv.hpp>

class Light
{
public:
    cv::RotatedRect rect;
    float angle;
    float length; 
    float width;
    cv::Point2f center;
    std::vector<cv::Point> contour;
};

struct Armor 
{
public:
    Light up;
    Light down;
    std::vector<std::vector<cv::Point>> lightcontours;
    cv::Point2f center;   
    float width;            
    float height;           
    cv::RotatedRect box;    
};

class Process
{
public:
    cv::Mat preprocess(const cv::Mat& img);

    bool isLight(const std::vector<cv::Point>& contour);

    std::vector<Light> findLight(const cv::Mat& img);

    bool isArmour(const Light& a,const Light& b);

    std::vector<Armor> findArmor(const std::vector<Light>& lightArray);

    void drawline(cv::Mat img,const std::vector<Light> &lightArray,const std::vector<Armor>& armorArray);
};



#endif