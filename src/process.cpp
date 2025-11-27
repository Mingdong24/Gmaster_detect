#include"process.h"



cv::Mat Process::preprocess(const cv::Mat& img)
{
    if(img.empty())
    {
        return cv::Mat(cv::Size(20, 28), CV_8UC1);
    };

    cv::Mat gray;
    cv::Mat gray_bin;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    threshold(gray, gray_bin, 120, 255, cv::THRESH_BINARY);

    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    morphologyEx(gray_bin, gray_bin, cv::MORPH_OPEN, kernel);
    morphologyEx(gray_bin, gray_bin, cv::MORPH_CLOSE, kernel);

    return gray_bin;
}

bool Process::isLight(const std::vector<cv::Point>& contour)
{
    if (contour.size() < 3) return false;

    cv::RotatedRect rect = minAreaRect(contour);

    float w = cv::min(rect.size.width, rect.size.height);
    float h = cv::max(rect.size.width, rect.size.height);
    float ratio = h / w;

    if (ratio < 1.0 || ratio > 10.0) return false;

    float area = rect.size.area();
    if (area < 60 || area > 5000) return false;

    // float angle = (rect.size.width < rect.size.height) ? rect.angle : rect.angle + 90;
    // angle = fabs(angle);
    // if (angle < 10 || angle > 80) return false;

    return true;
}

std::vector<Light> Process::findLight(const cv::Mat& img)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> lightcontours;
    std::vector<Light> lightArray;
    cv::findContours(img,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    for(int i = 0;i < contours.size(); i++)
    {
        if(isLight(contours[i]))
        {
            lightcontours.push_back(contours[i]);
        }
    }
    for(int i = 0;i < lightcontours.size(); i++ )
    {
        Light light;

        light.contour = lightcontours[i];

        cv::RotatedRect rect = cv::minAreaRect(lightcontours[i]);
        light.rect = rect;

        light.center = rect.center;

        float w = rect.size.width;
        float h = rect.size.height;
        float length = std::max(w, h);
        float width  = std::min(w, h);
        light.length = length;
        light.width = width;

        float angle = rect.angle;
        if (rect.size.width < rect.size.height) {
            angle = rect.angle + 90;
        }
        angle = fabs(angle);
        light.angle = angle;

        lightArray.push_back(light);
    }

    return lightArray;
}
    
bool Process::isArmour(const Light& a,const Light& b)
{
    if (fabs(a.angle - b.angle) > 20.0f)
    {
        return false;
    }

    if (fabs(a.center.x - b.center.x) > 100.0f)
    {
        return false;
    }

    float len_diff = fabs(a.length - b.length) / std::max(a.length, b.length);
    if (len_diff > 1.0f)
    {
        return false;
    }
    float cdist = cv::norm(a.center - b.center);
    float avg_len = (a.length + b.length) * 0.5f;

    float ratio = cdist / avg_len;
    if (ratio < 1.0f || ratio > 9.0f)
    {
        return false;
    }

    return true;
}

std::vector<Armor> Process::findArmor(const std::vector<Light>& lightArray)
{
    std::vector<Armor> armors;

    for (size_t i = 0; i < lightArray.size(); i++) {
        for (size_t j = i + 1; j < lightArray.size(); ++j) {

            const Light& a = lightArray[i];
            const Light& b = lightArray[j];

            if (!isArmour(a, b))
                continue;

            Armor am;

            if (a.center.y < b.center.y) {
                am.up = a;
                am.down = b;
            } else {
                am.up = b;
                am.down = a;
            }

            am.center = (am.up.center + am.down.center) * 0.5f;

            am.width = cv::norm(am.up.center - am.down.center);
            am.height = (am.up.length + am.down.length) * 0.5f;

            am.box = cv::RotatedRect(
                am.center,
                cv::Size2f(am.width, am.height),
                (am.up.angle + am.down.angle) * 0.5f
            );

            am.lightcontours.push_back(a.contour);
            am.lightcontours.push_back(b.contour);

            armors.push_back(am);
        }
    }
    return armors;
}

void Process::drawline(cv::Mat img,const std::vector<Light> &lightArray,const std::vector<Armor>& armorArray)
{

    // for (const auto& lb : lightArray)
    // {
    //     cv::polylines(img, lb.contour, true, cv::Scalar(0,255,0), 2);

    //     cv::Point2f pts[4];
    //     lb.rect.points(pts);
    //     for(int i = 0; i < 4; i++)
    //     {
    //         cv::line(img, pts[i], pts[(i+1)%4], cv::Scalar(0,255,0), 2);
    //     }

    // }

    for (const auto& armor : armorArray)
    {

        cv::circle(img, armor.center, 5, cv::Scalar(0,0,255), -1);

        for(int i = 0;i < 2;i ++)
        {
            cv::polylines(img, armor.lightcontours[i], true, cv::Scalar(0,255,0), 2);
        }
        cv::Point2f pts1[4];
        armor.up.rect.points(pts1);
        for(int i = 0; i < 4; i++)
        {
            cv::line(img, pts1[i], pts1[(i+1)%4], cv::Scalar(0,255,0), 2);
        }

        cv::Point2f pts2[4];
        armor.down.rect.points(pts2);
        for(int i = 0; i < 4; i++)
        {
            cv::line(img, pts2[i], pts2[(i+1)%4], cv::Scalar(0,255,0), 2);
        }

    }

}