//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cstdio>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;
        if(u_img < 0.0f) u_img = 0.0f;
        else if(u_img > width) u_img = width;
        if(v_img < 0.0f) v_img = 0.0f;
        else if(v_img > height) v_img = height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v){
        float u_img = u * width;
        float v_img = v * height;
        float lu = floor(u_img), lv = floor(v_img);
        float hu = ceil(u_img), hv = ceil(v_img);
        Eigen::Vector3f cll = getColor(lu / width, lv / height);
        Eigen::Vector3f chl = getColor(lu / width, hv / height);
        Eigen::Vector3f clh = getColor(hu / width, lv / height);
        Eigen::Vector3f chh = getColor(hu / width, hv / height);
        cll = (cll * (v_img - lv) / ((float)hv - lv)) + (clh * (hv - v_img) / ((float)hv - lv));
        chl = (chl * (v_img - lv) / ((float)hv - lv)) + (chh * (hv - v_img) / ((float)hv - lv));
        cll = (cll * (u_img - lu) / ((float)hu - lu)) + (chl * (hu - u_img) / ((float)hu - lu));
        return cll;
    }

};
#endif //RASTERIZER_TEXTURE_H
