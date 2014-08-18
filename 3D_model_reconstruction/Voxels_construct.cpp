#ifdef _WIN32
#include <windows.h>
#endif

#include <gl/GL.h>			// Header File For The OpenGL32 Library
#include <gl/GLU.h>			// Header File For The GLu32 Library
#include <GL/glut.h>

//#include <AR/gsub_lite.h>

#include <math.h>
#include <iostream>
#include <ctime>

#include <opencv2/opencv.hpp>

#include "voxel_construct.h"
#include "constants.h"
#include "ar_routine.h"
#include "capturing.h"
#include "utilities.h"

using namespace cv;

// GLUT displaying
int degree = 0;													// rotation degree of the model
Mat trans_opengl = (Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 0, 1, -50, 0, -1, 0, 0, 0, 0, 0, 1);	// fix the wrong rotation
Mat voxels_clone;												// use for drawing only
Mat voxels_color(1, total_voxel, CV_8UC3, Scalar(0, 255, 0));	// color of each voxel

// voxel curving
int voxel_hp[total_voxel] = { -2048 };					// stores 'hp' of each voxel
char voxel_hp_regen[total_voxel] = { 0 };				// stores the 'regen' value of each voxel
bool voxel_valid[total_voxel] = { true };				// the voxel are valid only if it's surficial and has hp left

// texture mapping
int depth_voxel_id[prefHeight][prefWidth] = { 0 };		// the id is start at 0, -1 for not having any voxel
float depth_map[prefHeight][prefWidth] = { 0 };			// the depth value of each voxel in specific frame
vector<Mat> voxel_2D_vec;								// stores the voxel 2D location of all actual camera views
bool is_voxel_colored[total_voxel] = { };				// true if the voxel is already colored, false otherwise
bool is_ready_to_texture_map = false;					// before texture mapping start, make sure that all of the captured images are grabcutted

//=================================================
//== GLUT FUNCTIONS                              ==
//=================================================

void visibility_viewer(int visible) {
	if (visible == GLUT_VISIBLE) glutIdleFunc(idle_viewer);
	else glutIdleFunc(NULL);
}

void reshape_viewer(int w, int h) {
	// Call through to anyone else who needs to know about window sizing here.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void idle_viewer(void) {

}

void display_viewer(void) {
	vd_texture_mapping();
	if (voxels_clone.empty()) {
		voxels_clone = get_voxels().clone();
		//the model is in wrong rotation, this does fix it
		voxels_clone = trans_opengl * voxels_clone;
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho(-150,150,-112,112,-1000,1000);  // real viewer
	//glOrtho(-400, 400, -300, 300, -1000, 1000);  // debug : camera location display
	gluPerspective(60, (0.0 + prefWidth) / prefHeight, 1, 1500);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(100, 100, 200, 0, 0, 0, 0, 1, 0);

	glPushMatrix();
	glRotatef(degree, 0, 1, 0);
	
	degree++;
	if (degree >= 360) degree = 0;
	
	//- DEBUG -
	glBegin(GL_LINES);
	glColor3ub(255, 0, 0);
	glVertex3f(-500, 0, 0);
	glVertex3f(500, 0, 0);
	glColor3ub(0, 255, 0);
	glVertex3f(0, -500, 0);
	glVertex3f(0, 500, 0);
	glColor3ub(0, 0, 255);
	glVertex3f(0, 0, -500);
	glVertex3f(0, 0, 500);
	glEnd();
	
	/*vector<Mat>* views = get_all_camera_view();
	glColor3f(0.3f, 0.3f, 1.0f);
	for (int i = 0; i < views->size(); i++) {
		Mat current_view = trans_opengl * views->at(i);
		glPushMatrix();
		Mat current_view_t = current_view.t();
		glMultMatrixf(current_view_t.ptr<float>());
		glutSolidCube(5);
		glPopMatrix();
	}

	vector<Mat>* rec_cam = get_recommended_acam();
	glColor3f(0.8f, 0.8f, 1.0f);
	for (int i = 0; i < rec_cam->size(); i++) {
		if (get_should_capture_at(i)) continue;
		Mat current_view = trans_opengl * rec_cam->at(i);
		glPushMatrix();
		Mat current_view_t = current_view.t();
		glMultMatrixf(current_view_t.ptr<float>());
		glutSolidCube(5);
		glPopMatrix();
	}*/
	// DEBUG end
	
	//don't foget to change this back to voxel_size
	glPointSize(2.0f);
	glBegin(GL_POINTS);
	for (int i = 0; i < voxels_clone.cols; i++) {
		if (voxel_hp[i] > 0) {
			int r = voxels_color.at<Vec3b>(0, i).val[2];
			int g = voxels_color.at<Vec3b>(0, i).val[1];
			int b = voxels_color.at<Vec3b>(0, i).val[0];

			glColor3ub(r, g, b);
			glVertex3f(voxels_clone.at<float>(0, i), voxels_clone.at<float>(1, i), voxels_clone.at<float>(2, i));
		}
	}
	glEnd();

	glPopMatrix();
	glutSwapBuffers();
}

//=================================================
//== VOXEL CARVING                               ==
//=================================================

void init_voxels_hp() {
	std::fill_n(voxel_hp, total_voxel, voxel_max_hp);
	std::fill_n(voxel_valid, total_voxel, true);
}

void shape_voxels(Mat image, Mat voxels2D, Mat silhouette) {
	for (int i = 0; i < voxels2D.cols; i++) {
		int x = voxels2D.at<float>(0, i);
		int y = voxels2D.at<float>(1, i);

		if (x < 0 || y < 0) continue;
		if (x >= silhouette.cols || y >= silhouette.rows) continue;
		
		//if the voxel located the region marked as background, decrease its hp
		if (silhouette.at<Vec3b>(y, x)[0] == fillColor.val[0] && silhouette.at<Vec3b>(y, x)[1] == fillColor.val[1] && silhouette.at<Vec3b>(y, x)[2] == fillColor.val[2]) {
			voxel_hp[i]--;
		}
		//otherwise, if it's in foreground region for 5 consecutive cycles (of shpae_voxels) it gains 1 hp
		else {
			voxel_hp_regen[i]++;
			if (voxel_hp_regen[i] == voxel_hp_regen_rate) {
				voxel_hp_regen[i] = 0;
				if (voxel_hp[i] < voxel_max_hp) voxel_hp[i]++;
			}
		}
	}
}

//this function update the validity array of each voxel by doing the following test
//the voxel is SURFICIAL and
//the voxel have HP LEFT
//Note that this function DOES NOT check whether the voxel is visible in specific camera view or not
void update_voxel_validity() {
	//loop though all of the voxel in 3D
	for (int x = first_voxel_x; x <= last_voxel_x; x++) {
		for (int y = first_voxel_y; y <= last_voxel_y; y++) {
			for (int z = first_voxel_z; z <= last_voxel_z; z++) {
				if (check_sorrounded_voxel(x, y, z)) voxel_valid[get_voxel_index(x, y, z)] = true;
				else voxel_valid[get_voxel_index(x, y, z)] = false;
			}
		}
	}
}

bool check_sorrounded_voxel(int x, int y, int z) {
	//if the voxel doesn't have any hp left, it's invalid
	if (voxel_hp[get_voxel_index(x, y, z)] <= 0) return false;

	//if the voxel is in the surface of the whole voxel cube, it's surficial
	if (x + 1 > last_voxel_x || x - 1 < first_voxel_x)  return true;
	if (y + 1 > last_voxel_y || y - 1 < first_voxel_y)  return true;
	if (z + 1 > last_voxel_z || z - 1 < first_voxel_z)  return true;

	//check that the voxel is not sorrounded by other voxel
	int up = get_voxel_index(x, y + 1, z);
	int down = get_voxel_index(x, y - 1, z);
	int left = get_voxel_index(x - 1, y, z);
	int right = get_voxel_index(x + 1, y, z);
	int front = get_voxel_index(x, y, z + 1);
	int back = get_voxel_index(x, y, z - 1);

	if (voxel_hp[up] <= 0) return true;
	if (voxel_hp[down] <= 0) return true;
	if (voxel_hp[left] <= 0) return true;
	if (voxel_hp[right] <= 0) return true;
	if (voxel_hp[front] <= 0) return true;
	if (voxel_hp[back] <= 0) return true;

	//if it failed all test, the voxel is not surficial (invalid)
	return false;
}

//=================================================
//== VIEW DEPENDENT TEXTURE MAPPING              ==
//=================================================

//construct a depth map and depth id based on the camera views
void construct_depth_map(Mat cam_view, int depth_voxel_id[][prefWidth], float depth_map[][prefWidth]) {
	Mat voxel_3D = get_voxels();
	Mat voxel_2D = cam_view * voxel_3D;
	Mat z_value_of_voxels = voxel_2D.row(2).clone();
	voxel_2D = get_camera_matrix() * voxel_2D;
	cv::divide(voxel_2D.row(0), voxel_2D.row(2), voxel_2D.row(0));
	cv::divide(voxel_2D.row(1), voxel_2D.row(2), voxel_2D.row(1));

	//- DEBUG -
	Mat depth(480, 640, CV_8UC1, Scalar(0));

	//reset the depth_map array and depth_voxel_id array
	for (int i = 0; i < prefHeight; i++) {
		for (int j = 0; j < prefWidth; j++) {
			depth_map[i][j] = 999999;
			depth_voxel_id[i][j] = -1;
		}
	}

	for (int i = 0; i < total_voxel; i++) {
		//if voxel is not valid, don't do anything
		if (!voxel_valid[i]) continue;

		//get the voxel's x and y location in the image and round to integer
		int x = voxel_2D.at<float>(0, i);
		int y = voxel_2D.at<float>(1, i);
		if (x < 0 || y < 0) continue;
		if (x > prefWidth - 1 || y > prefHeight - 1) continue;

		//draw a square centered at the voxel
		for (int j = -voxel_size / 2; j < voxel_size / 2; j++) {
			for (int k = -voxel_size / 2; k < voxel_size / 2; k++) {
				int new_x = x + j;
				int new_y = y + k;
				if (new_x < 0 || new_y < 0 || new_x > prefWidth - 1 || new_y > prefHeight - 1) continue;

				//if more than one voxels are in the same x and y, keep the one that closer to the camera
				float old_z = depth_map[new_y][new_x];
				float current_z = z_value_of_voxels.at<float>(0, i);
				if (current_z < old_z) {
					depth_map[new_y][new_x] = current_z;
					//also store the id of the voxel
					depth_voxel_id[new_y][new_x] = i;
				}

				//- DEBUG -
				if (current_z < old_z) {
					int gray = current_z;
					gray = 255 - ((gray / 500.0) * 255);
					Scalar c(gray);
					depth.row(new_y).col(new_x) = c;
				}
			}
		}
	}

	show_image("depth map", depth);
	waitKey(10);
}

void init_acam_voxel2D() {
	std::cout << "calculating voxel 2D location for each actual camera" << std::endl;
	Mat voxel_3D = get_voxels();
	vector<Mat>* camera_views = get_all_camera_view();

	for (int i = 0; i < camera_views->size(); ++i){
		voxel_2D_vec.push_back(cvt_3dPoints_2dPoints_cvmat(voxel_3D, (*camera_views)[i].inv(), get_camera_matrix()));
	}
}

Mat construct_vcam(int y_rotation) {
	float yaw = y_rotation;
	float pitch = v_cam_pitch;
	float x = v_cam_sphere_radius * cos(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
	float y = v_cam_sphere_radius * sin(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
	float z = v_cam_sphere_radius * cos(deg_to_rad(pitch));
	if (x <= 0.001 && x >= -0.001) x = 0;
	if (y <= 0.001 && y >= -0.001) y = 0;
	if (z <= 0.001 && z >= -0.001) z = 0;
	Mat v_cam = cal_trans_mat(0, pitch, yaw, x, y, z);
	Mat to_origin = (Mat_<float>(4, 4) << 1, 0, 0, -x, 0, 1, 0, -y, 0, 0, 1, -z, 0, 0, 0, 1);
	Mat from_origin = (Mat_<float>(4, 4) << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1);
	v_cam = to_origin * v_cam;
	v_cam = v_cam.inv();
	v_cam = cal_trans_mat(0, 0, -90, 0, 0, 0) * v_cam;
	v_cam = cal_trans_mat(180, 0, 0, 0, 0, 0) * v_cam;
	v_cam = v_cam.inv();
	v_cam = from_origin * v_cam;

	return v_cam;
}

void vd_texture_mapping() {
	if (!get_is_mapping() && !is_ready_to_texture_map) return;

	vector<Mat>* captured_images = get_all_captured_image();
	vector<Mat>* camera_views = get_all_camera_view();
	Mat output_image(480, 640, CV_8UC3);
	Mat v_cam = construct_vcam(degree);

	construct_depth_map(v_cam.inv(), depth_voxel_id, depth_map);

	//pre-compute the c value to use in cosine laws for determining the closest actual camera
	vector<float> distance_acam_vcam;
	for (int i = 0; i < camera_views->size(); i++) {
		float distance = cal_3d_sqr_distance(camera_views->at(i) , v_cam);
		distance_acam_vcam.push_back(distance);
	}

	//for each voxel appeared in depth map id
	for (int y = 0; y < prefHeight; y++) {
		for (int x = 0; x < prefWidth; x++) {
			if (depth_voxel_id[y][x] == -1) continue;
			//find the closest actual camera view and image
			int voxel_id = depth_voxel_id[y][x];
			vector<int> c_index = find_x_closest_view(voxel_id, v_cam, -1, distance_acam_vcam);
			
			int r = 0, g = 0, b = 0, count = 0;
			for (int i = 0; i < image_count; i++) {
				Mat closest_image = captured_images->at(c_index[i]);
				Mat voxel_2D = voxel_2D_vec[c_index[i]].col(voxel_id);
				int image_x_loc = voxel_2D.at<float>(0, 0);
				int image_y_loc = voxel_2D.at<float>(1, 0);
				if (image_x_loc < 0 || image_y_loc < 0 || image_x_loc > prefWidth - 1 || image_y_loc > prefHeight - 1) continue;

				Vec3b new_color = closest_image.at<Vec3b>(image_y_loc, image_x_loc);
				if (new_color.val[0] == 0 && new_color.val[1] == 0 && new_color.val[2] == 0) continue;
				r += new_color.val[2];
				g += new_color.val[1];
				b += new_color.val[0];
				count++;
			}

			if (count == 0) continue;
			Vec3b new_color(b / count, g / count, r / count);

			//get the color at <voxel_2D> on the image to color the voxel
			voxels_color.at<Vec3b>(0, voxel_id) = new_color;
			is_voxel_colored[voxel_id] = true; //set colored flag to true
			output_image.at<Vec3b>(y, x) = new_color;
		}
	}

	show_image("3D Model", output_image);
	waitKey(10);
}

//find the closest actual view from the v_cam
int find_closest_view(int voxel_id, Mat v_cam, int exclude_id) {
	//get all ACTUAL camera views and accquire 3D voxel location both of these are in MARKER CS
	vector<Mat>* cam_views = get_all_camera_view();
	Mat voxel_location = get_voxels().col(voxel_id);
	//voxel_location = get_shift_origin() * voxel_location;
	int closest_acam_index = 0;
	float cosine_deg = 999999;

	for (int i = 0; i < cam_views->size(); i++) {
		Mat current_actual_view = cam_views->at(i);
		if (i == exclude_id) continue;

		float a = cal_3d_distance(v_cam, voxel_location);
		float b = cal_3d_distance(current_actual_view, voxel_location);
		float c = cal_3d_distance(current_actual_view, v_cam);

		//use cosine law
		float angle = ((a*a) + (b*b) - (c*c)) / (2 * a*b);
		angle = rad_to_deg(acos(angle));

		if (angle < cosine_deg) {
			cosine_deg = angle;
			closest_acam_index = i;
		}
	}

	return closest_acam_index;
}

//find the [x] closest actual view from the v_cam, where x is an integer
vector<int> find_x_closest_view(int voxel_id, Mat v_cam, int exclude_id, vector<float> c_values) {
	vector<float> closest_deg(image_count);
	vector<int> closest_index(image_count);
	for (int i = 0; i < image_count; i++) {
		closest_deg[i] = 999999;
	}
	//get all ACTUAL camera views and accquire 3D voxel location both of these are in MARKER CS
	vector<Mat>* cam_views = get_all_camera_view();
	Mat voxel_location = get_voxels().col(voxel_id);

	float a = cal_3d_sqr_distance(v_cam, voxel_location);
	for (int i = 0; i < cam_views->size(); i++) {
		Mat current_actual_view = cam_views->at(i);
		if (i == exclude_id) continue;

		float b = cal_3d_sqr_distance(current_actual_view, voxel_location);
		float c = c_values.at(i);

		//use cosine law
		float angle = ((a) + (b) - (c)) / (2 * sqrt((a*b)));
		angle = rad_to_deg(acos(angle));

		for (int j = 0; j < image_count; j++) {
			if (angle < closest_deg[j]) {
				if (closest_deg[j] != 999999) {
					for (int k = image_count - 1; k >= j; k--) {
						if (k + 1 >= image_count) continue;
						closest_deg[k + 1] = closest_deg[k];
						closest_index[k + 1] = closest_index[k];

					}
				}
				closest_deg[j] = angle;
				closest_index[j] = i;
				break;
			}
		}
	}

	return closest_index;
}

//=================================================
//== PREPARING CAPTURED IMAGES                   ==
//=================================================

//grabcut the image to eliminates the background
void finish_capturing() {
	if (voxel_2D_vec.empty()) init_acam_voxel2D();

	vector<Mat>* a_cam_views = get_all_camera_view();
	vector<Mat>* a_cam_images = get_all_captured_image();
	for (int i = 0; i < a_cam_views->size(); i++) {
		float bound[4] = { 999999, 0, 999999, 0 }; //min_x max_x min_y max_y
		find_voxel_bound(voxel_2D_vec[i], bound);
		int w = (bound[1] - bound[0]) + (object_border_expand * 2);
		int h = (bound[3] - bound[2]) + (object_border_expand * 2);

		//grabcut the image
		Mat current_image = (*a_cam_images)[i];
		Mat mask, bg_model, fg_model;
		Rect poi(bound[0] - object_border_expand, bound[2] - object_border_expand, w, h);
		grabCut(current_image, mask, poi, bg_model, fg_model, grabcut_iter, GC_INIT_WITH_RECT);

		//produce the result image
		mask = mask & 1;
		Mat result(current_image.size(), current_image.type());
		current_image.copyTo(result, mask);

		//- DEBUG -
		rectangle(current_image, poi, Scalar(0, 0, 255), 3);
		(*a_cam_images)[i] = result;
		show_image("cut", result);
		show_image("ori", current_image);
		waitKey(10);
	}

	is_ready_to_texture_map = true;
}

//find the rectangel bound of the modeling object by using 2D voxel's location
void find_voxel_bound(Mat input, float output[4]) {
	for (int i = 0; i < input.cols; i++) {
		if (!voxel_valid[i]) continue;
		if (input.at<float>(0, i) < output[0]) output[0] = input.at<float>(0, i);
		else if (input.at<float>(0, i) > output[1]) output[1] = input.at<float>(0, i);

		if (input.at<float>(1, i) < output[2]) output[2] = input.at<float>(1, i);
		else if (input.at<float>(1, i) > output[3]) output[3] = input.at<float>(1, i);
	}
}


//=================================================
//== GETTERS (THIS MAYBE TEMPORARY)              ==
//=================================================

float get_depth_at(int x, int y) {
	return depth_map[y][x];
}

int get_depth_id(int x, int y) {
	return depth_voxel_id[y][x];
}