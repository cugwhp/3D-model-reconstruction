#pragma once
#include <opencv2/core/core.hpp>

// voxel and translation matrix based operation
cv::Mat cvt_3dPoints_2dPoints_array(cv::Mat points, double marker_trans[3][4], double camera_matrix[3][4]);
cv::Mat cvt_3dPoints_2dPoints_cvmat(cv::Mat point_3D, cv::Mat marker_trans, cv::Mat camera_mat);
void cvt_trans_mat_to_array(cv::Mat, double[4][4]);
void cvt_trans_mat_to_glarray(cv::Mat mat, float output[16]);
cv::Mat cvt_trans_mat_to_cvmat(double ar_trans_mat[3][4]);
void copy_trans_array(double src[4][4], double dest[][4]);
cv::Mat extract_rotation_mat(cv::Mat input);

// coverting index for voxel pointing
int get_voxel_index(int x, int y, int z);
void get_voxel_coord(int index, int output[3]);

// camera move recommendation
int get_acam_start_index(int pitch_layer, int total_acam);
int get_acam_end_index(int pitch_layer, int total_acam, int start_index);
float get_pitch_layer_start(int pitch_layer);
float get_pitch_layer_end(int pitch_layer);

// distance and translation/rotation matrix generation
cv::Mat cal_trans_mat(float rx, float ry, float rz, float tx, float ty, float tz);
float cal_pitch(cv::Mat translation);
float cal_3d_distance(cv::Mat first_point, cv::Mat second_point);
float cal_3d_sqr_distance(cv::Mat first_point, cv::Mat second_point);
float cal_color3c_sqr_distance(cv::Vec3b first_color, cv::Vec3b second_color);
float cal_3d_point2line_distance(cv::Mat l_first_point, cv::Mat l_second_point, cv::Mat point);

// some mathematic utilities
float var(float mean, std::vector<float> data);
float percentile(float percentile, std::vector<float> data);
float deg_to_rad(float degree);
float rad_to_deg(float rad);

// help in debugging and gui
void show_image(const char* name, cv::Mat image);
void printMatInReadableWay(cv::Mat, int, int);