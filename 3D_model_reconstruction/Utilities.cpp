#define _USE_MATH_DEFINES

#include <iostream>
#include <math.h>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "utilities.h"
#include "constants.h"

using namespace cv;

int candidate_index[pitch_points][yaw_points];

// ************************************************************************
// voxel and translation matrix based operation
// ************************************************************************

Mat cvt_3dPoints_2dPoints_array(Mat points, double marker_trans[3][4], double camera_matrix[3][4]) {
	//setup transformation matrix
	cv::Mat camMatrix(4, 4, CV_64F, camera_matrix);
	cv::Mat markerTrans(4, 4, CV_64F, marker_trans);
	camMatrix.at<double>(3, 0) = 0; camMatrix.at<double>(3, 1) = 0; camMatrix.at<double>(3, 2) = 0; camMatrix.at<double>(3, 3) = 1;
	markerTrans.at<double>(3, 0) = 0; markerTrans.at<double>(3, 1) = 0; markerTrans.at<double>(3, 2) = 0; markerTrans.at<double>(3, 3) = 1;
	camMatrix.convertTo(camMatrix, CV_32F);
	markerTrans.convertTo(markerTrans, CV_32F);

	float originToXTrans[4][4] = { { 1, 0, 0, 100 }, { 0, 1, 0, -50 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	cv::Mat originToXMat(4, 4, CV_32F, originToXTrans);
	markerTrans = markerTrans * originToXMat;
	
	//calculate the 2D region of which occupied by cube
	cv::Mat region = points.clone();
	region = markerTrans * region;
	region = camMatrix * region;
	//devind x and y by z
	for (int i = 0; i < region.cols; i++) {
		region.at<float>(0, i) /= region.at<float>(2, i);
		region.at<float>(1, i) /= region.at<float>(2, i);
	}

	return region;
}

Mat cvt_3dPoints_2dPoints_cvmat(Mat point_3D, Mat marker_trans, Mat camera_mat) {
	cv::Mat result = camera_mat * marker_trans * point_3D;

	for (int i = 0; i < result.cols; i++) {
		result.at<float>(0, i) /= result.at<float>(2, i);
		result.at<float>(1, i) /= result.at<float>(2, i);
	}

	return result;
}

void cvt_trans_mat_to_array(cv::Mat mat, double dest[4][4]) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			dest[i][j] = mat.at<float>(i, j);
		}
	}
}

void cvt_trans_mat_to_glarray(Mat mat, float output[16]) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			output[(i * 4) + j] = mat.at<float>(j, i);
		}
	}
}

Mat cvt_trans_mat_to_cvmat(double ar_trans_mat[3][4]) {
	double trans_array[4][4] = { {} };
	copy_trans_array(ar_trans_mat, trans_array);
	Mat trans_mat(4, 4, CV_64F, trans_array);
	trans_mat.convertTo(trans_mat, CV_32F);

	return trans_mat;
}

void copy_trans_array(double src[4][4], double dest[][4]) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			dest[i][j] = src[i][j];
		}
	}
	dest[3][0] = 0; dest[3][1] = 0; dest[3][2] = 0; dest[3][3] = 1;
}

Mat extract_rotation_mat(Mat input) {
	Mat result = input.clone();
	result.convertTo(result, CV_32F);
	result.at<float>(0, 3) = 0;
	result.at<float>(1, 3) = 0;
	result.at<float>(2, 3) = 0;

	return result;
}


// ************************************************************************
// coverting index
// ************************************************************************

int get_voxel_index(int x, int y, int z) {
	x -= first_voxel_x;
	y -= first_voxel_y;
	z -= first_voxel_z;
	return (x * voxel_y * voxel_z) + (y * voxel_z) + z;
}

void get_voxel_coord(int index, int output[3]) {
	int z = (index % voxel_z) + first_voxel_z;
	int y = (((index - z) / voxel_z) % voxel_y) + first_voxel_y;
	int x = ((((index - z) / voxel_z) - y) / voxel_y) + first_voxel_x;
	output[0] = x;
	output[1] = y;
	output[2] = z;
}


// ************************************************************************
// camera move recommendation
// ************************************************************************

int get_acam_start_index(int pitch_layer, int total_acam) {
	if (pitch_layer == 1) return 0;
	if (pitch_layer == pitch_points) return total_acam - 1;
	return (yaw_points * (pitch_layer - 1)) - ((pitch_layer - 2) * reduce_yaw_step);
}

int get_acam_end_index(int pitch_layer, int total_acam, int start_index) {
	if (pitch_layer == 1) return yaw_points - 1;
	if (pitch_layer == pitch_points) return total_acam - 1;
	return (start_index + (yaw_points - ((pitch_layer - 1) * reduce_yaw_step))) - 1;
}

float get_pitch_layer_start(int pitch_layer) {
	if (pitch_layer == pitch_points) return 0 + pitch_threshold;
	if (pitch_layer == 1) return (90 - starting_pitch) + pitch_threshold;
	float allow_region = 90 - starting_pitch;
	return (allow_region - ((allow_region / (pitch_points - 1)) * (pitch_layer - 1))) + pitch_threshold;
}

float get_pitch_layer_end(int pitch_layer) {
	return (get_pitch_layer_start(pitch_layer) - pitch_threshold * 2) + 1;	// upper bound
}


// ************************************************************************
// distance and translation/rotation matrix generation
// ************************************************************************

Mat cal_trans_mat(float rx, float ry, float rz, float tx, float ty, float tz) {
	Mat trans_mat(4, 4, CV_32F);

	rx = deg_to_rad(rx);
	ry = deg_to_rad(ry);
	rz = deg_to_rad(rz);

	trans_mat.at<float>(0, 0) = cos(rz)*cos(ry);
	trans_mat.at<float>(0, 1) = cos(rz)*sin(ry)*sin(rx) - sin(rz)*cos(rx);
	trans_mat.at<float>(0, 2) = cos(rz)*sin(ry)*cos(rx) + sin(rz)*sin(rx);
	trans_mat.at<float>(0, 3) = tx;
	trans_mat.at<float>(1, 0) = sin(rz)*cos(ry);
	trans_mat.at<float>(1, 1) = sin(rz)*sin(ry)*sin(rx) + cos(rz)*cos(rx);
	trans_mat.at<float>(1, 2) = sin(rz)*sin(ry)*cos(rx) - cos(rz)*sin(rx);
	trans_mat.at<float>(1, 3) = ty;
	trans_mat.at<float>(2, 0) = -sin(ry);
	trans_mat.at<float>(2, 1) = cos(ry)*sin(rx);
	trans_mat.at<float>(2, 2) = cos(ry)*cos(rx);
	trans_mat.at<float>(2, 3) = tz;
	trans_mat.at<float>(3, 0) = 0;
	trans_mat.at<float>(3, 1) = 0;
	trans_mat.at<float>(3, 2) = 0;
	trans_mat.at<float>(3, 3) = 1;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (trans_mat.at<float>(i, j) <= 0.001 && trans_mat.at<float>(i, j) >= -0.001) trans_mat.at<float>(i, j) = 0;
		}
	}

	return trans_mat;
}

//alpha = yaw
//beta = roll
//gamma = pitch
float cal_pitch(Mat translation) {
	float beta = asin(-translation.at<float>(2, 0));
	float gamma = asin(translation.at<float>(2, 1) / cos(beta));
	return abs(rad_to_deg(gamma));
}

float cal_yaw(Mat translation) {
	float beta = asin(-translation.at<float>(2, 0));
	float alpha = asin(translation.at<float>(1, 0) / cos(beta));
	return rad_to_deg(alpha);
}

float cal_3d_distance(Mat first_point, Mat second_point) {
	float x1 = first_point.at<float>(0, 0);
	float x2 = second_point.at<float>(0, 0);
	float y1 = first_point.at<float>(1, 0);
	float y2 = second_point.at<float>(1, 0);
	float z1 = first_point.at<float>(2, 0);
	float z2 = second_point.at<float>(2, 0);

	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

float cal_3d_sqr_distance(Mat first_point, Mat second_point) {
	float x1 = first_point.at<float>(0, 0);
	float x2 = second_point.at<float>(0, 0);
	float y1 = first_point.at<float>(1, 0);
	float y2 = second_point.at<float>(1, 0);
	float z1 = first_point.at<float>(2, 0);
	float z2 = second_point.at<float>(2, 0);

	return ((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1));
}

float cal_color3c_sqr_distance(Vec3b first_color, Vec3b second_color) {
	float r1 = first_color.val[2];
	float r2 = second_color.val[2];
	float g1 = first_color.val[1];
	float g2 = second_color.val[1];
	float b1 = first_color.val[0];
	float b2 = second_color.val[0];

	return ((r2 - r1) * (r2 - r1)) + ((g2 - g1) * (g2 - g1)) + ((b2 - b1) * (b2 - b1));
}

float cal_3d_point2line_distance(Mat l_first_point, Mat l_second_point, Mat point) {
	Mat v = l_second_point - l_first_point;
	Mat w = point - l_first_point;

	double c1 = w.dot(v);
	double c2 = v.dot(v);
	double b = c1 / c2;

	Mat norm = point - (l_first_point + b * v);
	return norm.dot(norm);
}

Mat cal_3d_point2line_intersect(Mat l_first_point, Mat l_second_point, Mat point) {
	Mat v = l_second_point - l_first_point;
	Mat w = point - l_first_point;

	double c1 = w.dot(v);
	double c2 = v.dot(v);
	double b = c1 / c2;

	return l_first_point + b * v;
}


// ************************************************************************
// some mathematic utilities
// ************************************************************************

float var(float mean, vector<float> data) {
	float sum = 0;
	for (int i = 0; i < data.size(); i++) {
		float diff = data[i] - mean;
		sum += diff * diff;
	}

	return sum / data.size();
}

float percentile(float percentile, vector<float> data) {
	return ceil((percentile / 100) * data.size());
}

float deg_to_rad(float degree) {
	return degree * (M_PI / 180);
}

float rad_to_deg(float rad) {
	return rad * (180 / M_PI);
}




// ************************************************************************
// I like Java
// ************************************************************************

bool contains(vector<int> v, int element) {
	if (std::find(v.begin(), v.end(), element) != v.end()) return true;
	return false;
}

// ************************************************************************
// help in debugging and gui
// ************************************************************************

void show_image(const char* name, Mat image) {
	imshow(name, image);
}

void printMatInReadableWay(cv::Mat img, int row, int col) {
	if (row == -1 && col == -1) std::cout << format(img, "python") << std::endl;
	else if (row == -1) std::cout << format(img.col(col), "python") << std::endl;
	else if (col == -1) std::cout << format(img.row(row), "python") << std::endl;
	else std::cout << format(img.row(row).col(col), "python") << std::endl;
}