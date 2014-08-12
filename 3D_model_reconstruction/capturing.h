#pragma once
#include <opencv2/opencv.hpp>

void define_acam_recommendation();
void find_closest_acam(cv::Mat current_cam, cv::Mat current_frame);
void capture_image(cv::Mat current_cam, cv::Mat current_frame, int closest_rec_acam_index);
void transfer();
int suggest_direction(cv::Mat current_view);

void should_capture_or_not(cv::Mat user_camera_position);
void cal_voxel2D(std::vector<int> range);
std::vector<int> find_candidate_in_range(cv::Mat user_cam_position);
std::vector<int> find_acam_in_range(cv::Mat user_cam_position);
std::vector<int> find_closest_view_ranged(int voxel_id, cv::Mat v_cam, std::vector<float> c_values, std::vector<int> range);

std::vector<cv::Mat>* get_all_camera_view();
std::vector<cv::Mat>* get_all_captured_image();
std::vector<cv::Mat>* get_recommended_acam();

bool get_should_capture_at(int index);
bool get_is_captured_at(int index);