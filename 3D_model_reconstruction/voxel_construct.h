#pragma once
#include "constants.h"
#include <opencv2/opencv.hpp>

//=================================================
//== GLUT FUNCTIONS                              ==
//=================================================
void define_virtual_cam();
void visibility_viewer(int visible);
void reshape_viewer(int w, int h);
void idle_viewer(void);
void display_viewer(void);

//=================================================
//== VOXEL CARVING                               ==
//=================================================
void init_voxels_hp();
void shape_voxels(cv::Mat image, cv::Mat voxels2D, cv::Mat silhouette);
void update_voxel_validity();
bool check_sorrounded_voxel(int x, int y, int z);

//=================================================
//== VIEW DEPENDENT TEXTURE MAPPING              ==
//=================================================
void construct_depth_map(cv::Mat cam_view, int depth_voxel_id[][prefWidth], float depth_map[][prefWidth]);
void init_acam_voxel2D();
cv::Mat construct_vcam(int y_rotation);
void vd_texture_mapping();
int find_closest_view(int voxel_id, cv::Mat v_cam, int exclude_id = -1); //temp!!
std::vector<int> find_x_closest_view(int voxel_id, cv::Mat v_cam, int exclude_id, std::vector<float> c_values);

//=================================================
//== PREPARING CAPTURED IMAGES                   ==
//=================================================
void finish_capturing();
void find_voxel_bound(cv::Mat input, float output[4]);

//=================================================
//== GETTERS (THIS MAYBE TEMPORARY)              ==
//=================================================
//- DEBUG -
float get_depth_at(int x, int y);
int get_depth_id(int x, int y);