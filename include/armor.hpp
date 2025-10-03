#ifndef ARMOR_HPP
#define ARMOR_HPP


//channel B0 G1 R2
#define BLUE_LIGHTBARS 0   //蓝色灯条
#define RED_LIGHTBARS 2     //红色灯条



#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>

class lightbars
{
public:
    lightbars() = default;
    lightbars(const cv::RotatedRect& minRect)
    {
        this->minRect = minRect;
        this->length = std::max(minRect.size.width, minRect.size.height); //灯条长度
        this->width = std::min(minRect.size.width, minRect.size.height);  //灯条宽度
        this->area = minRect.size.area(); //灯条面积
        this->angle = minRect.angle; //灯条倾斜角
    }





    cv::RotatedRect minRect; //最小外接矩形
    float length;  //灯条长度
    float width;  //灯条宽度
    float area; //灯条面积
    float angle; //灯条倾斜角
};

cv::Mat process_img(const cv::Mat& img);
void detect_lightbar(const cv::Mat& binary_img, const cv::Mat& img);
//void draw_lightbars(std::vector<lightbars> lightbars_res, cv::Mat &img);

std::vector<cv::RotatedRect> minRects;




#endif