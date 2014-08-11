#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

#include "capturing.h"
#include "ar_routine.h"
#include "voxel_construct.h"
#include "utilities.h"
#include "constants.h"

// use for direction suggestion
#define DONE 0;
#define UP 1
#define RIGHT 2
#define DOWN 3
#define LEFT 4

using namespace cv;

vector<Mat> a_cam_views;								// contains all of the translation mat for all captured view in MARKER CS
vector<Mat> a_cam_images;								// contains all of the images captured so far NOTE: the index is coresponding to a_cam_views
vector<Mat> recommended_acam;							// list of defined virtual camera transformation matrix in MARKER CS

vector<bool> is_captured;								// [flag] true when that view is already captured
vector<bool> should_capture;							// [flag] true when that view is worth capture
bool is_carving_pitch_done = false;						// [flag] will be ture iff all of the recommended view in carving layer are already captured
bool pitch_layer_done[pitch_points];					// [flag] true if all of recommended view in that layer are already captured
bool is_capturing_done = false;							// [flag] all of the layer are complete
int recommended_layer = carving_pitch;					// recommended pitch layer
int captured_count = 0;									// how many the images captured so far

// find the closest recommended acam view based on current user's camera position
// also capture the image if the distance between user's camera and the closest recommended camera is less than capture_trigger_distance
void find_closest_acam(Mat current_cam, Mat current_frame) {
	// find the closest recommended camera position based on user's current camera position
	float closest_rec_acam_sqrdistance = 999999;
	int closest_rec_acam_index = -1;
	Mat zero(3, 1, CV_32F, 0);
	for (int i = 0; i < recommended_acam.size(); i++) {
		// using the distance between user's camera and a line drawn from 0,0,0 to recommended views position
		float dis = cal_3d_point2line_distance(zero, recommended_acam[i].col(3), current_cam.col(3));
		// only suggest if that view is not captured yet
		if (!is_captured[i] && dis < closest_rec_acam_sqrdistance) {
			closest_rec_acam_sqrdistance = dis;
			closest_rec_acam_index = i;
		}
	}

	if (closest_rec_acam_index == -1) return;

	// if the distance between user's camera and closest recommended views is close enough, captured the picture
	float distance = cal_3d_point2line_distance(zero, recommended_acam[closest_rec_acam_index].col(3), current_cam.col(3));
	//std::cout << distance << std::endl;
	if (distance < capture_trigger_distance) {
		std::cout << "captured : " << closest_rec_acam_index << " " << captured_count << " " << recommended_acam[closest_rec_acam_index].at<float>(2, 3) << std::endl;
		std::cout << recommended_layer << std::endl;
		capture_image(current_cam, current_frame, closest_rec_acam_index);
	}
}

// this function called by find_closest_acam
// called when the user's camera and the closest recommended camera is close enough
void capture_image(Mat current_cam, Mat current_frame, int closest_rec_acam_index) {
	a_cam_views.push_back(current_cam);
	a_cam_images.push_back(current_frame);
	is_captured[closest_rec_acam_index] = true;
	captured_count++;
}

// decide whether to capture this view or not
// generate a virtual image from the candidate 
// for all pixels, calculate the variance from the 3 images that are closest
// if the variance is low enough, don't have to capture from that point.
void should_capture_or_not() {
	for (int i = 0; i < recommended_acam.size(); i++) {
		int depth_voxel_id[prefHeight][prefWidth];		// the id is start at 0, -1 for not having any voxel
		float depth_map[prefHeight][prefWidth];		// the depth value of each voxel in specific frame
		Mat current_candidate = recommended_acam.at(i);
		vector<Mat>* camera_views = get_all_camera_view();
		vector<Mat>* captured_images = get_all_captured_image();
		vector<Mat>* voxel_2D_vec = cal_voxel2D();
		vector<float> variances;
		
		construct_depth_map(current_candidate, depth_voxel_id, depth_map);
		
		// c values used for computing 3 closest camera views
		vector<float> c_values;
		for (int j = 0; j < camera_views->size(); j++) {
			float c = cal_3d_sqr_distance(camera_views->at(j), current_candidate);
			c_values.push_back(c);
		}

		// loop though the depth_id map of the candidate view
		for (int y = 0; y < prefHeight; y++) {
			for (int x = 0; x < prefWidth; x++) {
				if (depth_voxel_id[y][x] == -1) continue;
				// find 3 closest actual camera view and image
				int voxel_id = depth_voxel_id[y][x];
				vector<int> c_index = find_closest_view_optimized(voxel_id, current_candidate, -1, c_values);

				// get the color from 3 closest actual views
				vector<vector<float>> colors;
				int r = 0, g = 0, b = 0;
				for (int i = 0; i < 3; i++) {
					Mat closest_image = captured_images->at(c_index[i]);
					Mat voxel_2D = voxel_2D_vec->at(c_index[i]).col(voxel_id);
					int image_x_loc = round(voxel_2D.at<float>(0, 0));
					int image_y_loc = round(voxel_2D.at<float>(1, 0));
					if (image_x_loc < 0 || image_y_loc < 0 || image_x_loc > prefWidth - 1 || image_y_loc > prefHeight - 1) continue;

					Vec3b new_color = closest_image.at<Vec3b>(image_y_loc, image_x_loc);
					r += new_color.val[2];
					g += new_color.val[1];
					b += new_color.val[0];
					colors.at(0).push_back(new_color.val[2]);
					colors.at(1).push_back(new_color.val[1]);
					colors.at(2).push_back(new_color.val[0]);
				}

				// calculate variance
				variances.push_back(var(r / 3, colors[0]));
				variances.push_back(var(g / 3, colors[1]));
				variances.push_back(var(b / 3, colors[2]));
			}
		}

		// compare (var_error_checking_percentile)th with color_error_threshold to make a decision
		std::sort(variances.begin(), variances.end());
		if (percentile(var_error_checking_percentile, variances) < color_error_threshold) should_capture[i] = false;
	}
}

vector<Mat>* cal_voxel2D() {
	Mat voxel_3D = get_voxels();
	vector<Mat>* camera_views = get_all_camera_view();
	vector<Mat> voxel_2D_vec;

	for (int i = 0; i < camera_views->size(); ++i){
		voxel_2D_vec.push_back(cvt_3dPoints_2dPoints_cvmat(voxel_3D, (*camera_views)[i].inv(), get_camera_matrix()));
	}

	return &voxel_2D_vec;
}

// suggest the direction for the user to move his camera
// voxel_carving_pitch layer first then lowest layer to highest layer
int suggest_direction(Mat current_view) {
	if (is_capturing_done) return DONE;
	float cam_pitch = cal_pitch(current_view);
	//std::cout << cam_pitch << " " << get_pitch_layer_end(recommended_layer) << " " << get_pitch_layer_start(recommended_layer) << std::endl;
	float recommended_layer_pitch_end = get_pitch_layer_end(recommended_layer);
	float recommended_layer_pitch_start = get_pitch_layer_start(recommended_layer);

	if (cam_pitch > recommended_layer_pitch_start) return UP;
	if (cam_pitch < recommended_layer_pitch_end) return DOWN;

	if (!is_carving_pitch_done) {
		int start_acam_index = get_acam_start_index(carving_pitch, recommended_acam.size());
		int end_acam_index = get_acam_end_index(carving_pitch, recommended_acam.size(), start_acam_index);

		for (int i = start_acam_index; i <= end_acam_index; i++) {
			if (!is_captured[i]) return RIGHT;
		}

		is_carving_pitch_done = true;
		pitch_layer_done[carving_pitch] = true;
		recommended_layer = 1;
		return RIGHT;
	}

	int start_acam_index = get_acam_start_index(recommended_layer, recommended_acam.size());
	int end_acam_index = get_acam_end_index(recommended_layer, recommended_acam.size(), start_acam_index);
	for (int i = start_acam_index; i <= end_acam_index; i++) {
		if (!is_captured[i]) return RIGHT;
	}

	recommended_layer++;
	if (recommended_layer == carving_pitch) recommended_layer++;
	if (captured_count == recommended_acam.size()) is_capturing_done = true;
	return RIGHT;
}

// define all recommended camera view for user to capture.
void define_acam_recommendation() {
	std::cout << get_pitch_layer_end(1) << " " << get_pitch_layer_start(1) << std::endl;
	std::cout << get_pitch_layer_end(2) << " " << get_pitch_layer_start(2) << std::endl;
	std::cout << get_pitch_layer_end(3) << " " << get_pitch_layer_start(3) << std::endl;
	std::cout << get_pitch_layer_end(4) << " " << get_pitch_layer_start(4) << std::endl;
	std::cout << get_pitch_layer_end(5) << " " << get_pitch_layer_start(5) << std::endl;

	int count = 0;
	int current_yaw_point = yaw_points;
	for (int i = 0; i < pitch_points - 1; i++) {
		for (int j = 0; j < current_yaw_point; j++) {
			float pitch = (90 - starting_pitch) - (((90 - starting_pitch) / (pitch_points - 1)) * i);
			float yaw = (360.0 / current_yaw_point) * j;

			float x = acam_rec_sphere_radius * cos(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
			float y = acam_rec_sphere_radius * sin(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
			float z = acam_rec_sphere_radius * cos(deg_to_rad(pitch));
			std::cout << "v cam #" << count << " " << yaw << " " << pitch << " " << x << " " << y << " " << z << std::endl;
			count++;

			if (x <= 0.001 && x >= -0.001) x = 0;
			if (y <= 0.001 && y >= -0.001) y = 0;
			if (z <= 0.001 && z >= -0.001) z = 0;
			Mat acam_recommended = cal_trans_mat(0, pitch, yaw, x, y, z);
			// fix the rotaion of the camera
			Mat to_origin = (Mat_<float>(4, 4) << 1, 0, 0, -x, 0, 1, 0, -y, 0, 0, 1, -z, 0, 0, 0, 1);
			Mat from_origin = (Mat_<float>(4, 4) << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1);
			acam_recommended = to_origin * acam_recommended;
			acam_recommended = acam_recommended.inv();
			acam_recommended = cal_trans_mat(0, 0, -90, 0, 0, 0) * acam_recommended;
			acam_recommended = cal_trans_mat(180, 0, 0, 0, 0, 0) * acam_recommended;
			acam_recommended = acam_recommended.inv();
			acam_recommended = from_origin * acam_recommended;

			recommended_acam.push_back(acam_recommended);
			is_captured.push_back(false);
		}
		current_yaw_point -= reduce_yaw_step;
	}

	float pitch = 0;
	float yaw = 0;

	float x = acam_rec_sphere_radius * cos(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
	float y = acam_rec_sphere_radius * sin(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
	float z = acam_rec_sphere_radius * cos(deg_to_rad(pitch));
	std::cout << "v cam #" << count << " " << yaw << " " << pitch << " " << x << " " << y << " " << z << std::endl;
	count++;

	if (x <= 0.001 && x >= -0.001) x = 0;
	if (y <= 0.001 && y >= -0.001) y = 0;
	if (z <= 0.001 && z >= -0.001) z = 0;
	Mat acam_recommended = cal_trans_mat(0, pitch, yaw, x, y, z);
	recommended_acam.push_back(acam_recommended);
	is_captured.push_back(false);
}

//return all of the captured views in MARKER CS
vector<Mat>* get_all_camera_view() {
	return &a_cam_views;
}

vector<Mat>* get_all_captured_image() {
	return &a_cam_images;
}

vector<Mat>* get_recommended_acam() {
	return &recommended_acam;
}

bool get_is_captured_at(int index) {
	return is_captured[index];
}
