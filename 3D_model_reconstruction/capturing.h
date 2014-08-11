#pragma once
#include <opencv2/core/core.hpp>

void define_acam_recommendation();
void find_closest_acam(cv::Mat current_cam, cv::Mat current_frame);
void capture_image(cv::Mat current_cam, cv::Mat current_frame, int closest_rec_acam_index);
int suggest_direction(cv::Mat current_view);

void should_capture_or_not();
std::vector<cv::Mat>* cal_voxel2D();

std::vector<cv::Mat>* get_all_camera_view();
std::vector<cv::Mat>* get_all_captured_image();
std::vector<cv::Mat>* get_recommended_acam();
bool get_is_captured_at(int index);