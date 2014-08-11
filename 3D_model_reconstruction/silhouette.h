#pragma once
#include <opencv2/core/core.hpp>

cv::Mat create_silhouette(cv::Mat);
bool determine_bgColor(cv::Mat image);
void setup_default_bgColor();

void remove_noise(cv::Mat image);