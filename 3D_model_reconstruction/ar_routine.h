#pragma once
#include <opencv2/core/core.hpp>
#include <AR/gsub_lite.h>

void camera_set_context(ARGL_CONTEXT_SETTINGS_REF arContext);
void setup_voxels();
bool setup_camera();
bool setup_marker();

void cleanup(void);
void visibility_camera(int);
void idle_camera(void);
void display_camera(void);
void drawHUD();
void draw();
void reshape(int, int);

void voxel_carving_routine();
void image_capturing_routine();

cv::Mat cvt_camCS_to_markerCS(cv::Mat marker_position);
cv::Mat get_voxels();
cv::Mat get_shift_origin();
cv::Mat get_camera_matrix();
cv::Mat get_current_view();
bool get_is_mapping();
int get_closest_view_index();

//keyboard function - for debug only purpose, will be deleted after finish
void debug_only(unsigned char key, int x, int y);