//
// Created by LEI XU on 4/27/19.
//

#pragma once

#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
  cv::Mat image_data;

public:
  Texture(const std::string &name) {
    image_data = cv::imread(name);
    cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
    width  = image_data.cols;
    height = image_data.rows;
  }

  int width, height;

  Eigen::Vector3f getColor(float u, float v) {
    int u_img  = static_cast<int>(u * width);
    int v_img  = static_cast<int>((1 - v) * height);
    auto color = image_data.at<cv::Vec3b>(v_img, u_img);
    return Eigen::Vector3f(color[0], color[1], color[2]) / 255.f;
  }

  Eigen::Vector3f getColor(Eigen::Vector2f uv) { return getColor(uv.x(), uv.y()); }
};
