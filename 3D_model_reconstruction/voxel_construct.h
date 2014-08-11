#pragma once
#include "constants.h"
#include <opencv2/core/core.hpp>

void init_voxels_hp();
void define_virtual_cam();
void visibility_viewer(int visible);
void reshape_viewer(int w, int h);
void idle_viewer(void);
void display_viewer(void);

void shape_voxels(cv::Mat image, cv::Mat voxels2D, cv::Mat silhouette);

void construct_depth_map(cv::Mat cam_view, int depth_voxel_id[][prefWidth], float depth_map[][prefWidth]);
void color_voxel(cv::Mat current_frame, cv::Mat voxel_2D);
void vd_color_voxel(std::vector<cv::Mat> cam_views, std::vector<cv::Mat> a_cam_images, int id, int x, int y);
void vd_texture_mapping();
int find_closest_view(int voxel_id, cv::Mat v_cam, int exclude_id=-1);
std::vector<int> find_closest_view_optimized(int voxel_id, cv::Mat v_cam, int exclude_id, std::vector<float> c_values);
void cal_depth_id_acam();
bool is_in_depth_map(int voxel_id);
bool is_visible_in_acam_view(int voxel_id, int acam_id);
void update_voxel_validity();
bool check_sorrounded_voxel(int x, int y, int z);

//- DEBUG -
float get_depth_at(int x, int y);
int get_depth_id(int x, int y);