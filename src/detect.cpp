#include"detect/detect.h"

ImageSubscribe::ImageSubscribe(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "ImageSubscribe节点已启动");

    cv::namedWindow("test01",cv::WINDOW_FREERATIO);
    cv::namedWindow("test02",cv::WINDOW_FREERATIO);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    image_subscribe = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        std::bind(&ImageSubscribe::command_callback, this, std::placeholders::_1));
}

void ImageSubscribe::command_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    cv::Mat dst = img.clone();  

    std::vector<Light> lightArray;
    std::vector<Armor>  armorArray;

    Process p;
    dst = p.preprocess(img);
    lightArray = p.findLight(dst);
    armorArray = p.findArmor(lightArray);
    p.drawline(img,lightArray,armorArray);

    cv::imshow("test01",img);
    cv::imshow("test02",dst);
    cv::waitKey(1);
}