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

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "voxel_construct.h"
#include "constants.h"
#include "ar_routine.h"
#include "capturing.h"
#include "utilities.h"

using namespace cv;

// voxel curving
int voxel_hp[total_voxel] = { -2048 };
char voxel_hp_regen[total_voxel] = { 0 };

// for display result model in viewer
int degree = 0;													// rotation degree of the model
Mat trans_opengl = (Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 0, 1, -50, 0, -1, 0, 0, 0, 0, 0, 1);	// fix the wrong rotation
Mat voxels_clone;												// use for drawing only
Mat voxels_color(1, total_voxel, CV_8UC3, Scalar(0, 255, 0));	// color of each voxel

// for texture mapping
bool voxel_valid[total_voxel] = { true };				// the voxel are valid only if it's surficial and has hp left
int depth_voxel_id[prefHeight][prefWidth] = { 0 };		// the id is start at 0, -1 for not having any voxel
float depth_map[prefHeight][prefWidth] = { 0 };			// the depth value of each voxel in specific frame
int temp_clicked_voxel_id = -1;

vector<Mat> acam_depth_idmap;
vector<Mat> voxel_2D_vec;
bool is_voxel_colored[total_voxel] = { };				// true if the voxel is already colored, false otherwise

void init_voxels_hp() {
	std::fill_n(voxel_hp, total_voxel, voxel_max_hp);
	std::fill_n(voxel_valid, total_voxel, true);
}

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
	gluPerspective(60, (0.0 + prefWidth) / prefHeight, 1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(200, 200, 700, 0, 0, 0, 0, 1, 0);

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
	
	vector<Mat>* views = get_all_camera_view();
	glColor3f(0.3f, 0.3f, 1.0f);
	for (int i = 0; i < views->size(); i++) {
		Mat current_view = trans_opengl * views->at(i);
		glPushMatrix();
		Mat current_view_t = current_view.t();
		glMultMatrixf(current_view_t.ptr<float>());
		glutSolidCube(5);
		glPopMatrix();
	}
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

	if (temp_clicked_voxel_id != -1) {
		glPointSize(10.0f);
		glBegin(GL_POINTS);
		glColor3ub(255, 0, 0);
		glVertex3f(voxels_clone.at<float>(0, temp_clicked_voxel_id), voxels_clone.at<float>(1, temp_clicked_voxel_id), voxels_clone.at<float>(2, temp_clicked_voxel_id));
		glEnd();
	}

	glPopMatrix();
	glutSwapBuffers();
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

//construct a depth map and depth id based on the camera views
void construct_depth_map(Mat cam_view, int depth_voxel_id[][prefWidth], float depth_map[][prefWidth]) {
	clock_t time = clock();
	clock_t time_d = clock();
	Mat voxel_3D = get_voxels();
	clock_t time_d2 = clock();
	std::cout << "        depth map: get_voxels() " << time_d2 - time_d << std::endl;
	time_d = clock();
	Mat voxel_2D = cam_view * voxel_3D;
	time_d2 = clock();
	std::cout << "        depth map: matrix mul " << time_d2 - time_d << std::endl;
	time_d = clock();
	Mat z_value_of_voxels = voxel_2D.row(2).clone();
	time_d2 = clock();
	std::cout << "        depth map: z value - matrix cloning " << time_d2 - time_d << std::endl;
	time_d = clock();
	//voxel_2D = cvt_3dPoints_2dPoints_cvmat(voxel_3D, cam_view, get_camera_matrix());
	voxel_2D = get_camera_matrix() * voxel_2D;
	cv::divide(voxel_2D.row(0), voxel_2D.row(2), voxel_2D.row(0));
	cv::divide(voxel_2D.row(1), voxel_2D.row(2), voxel_2D.row(1));
	time_d2 = clock();
	std::cout << "        depth map: cvt_3dPoints_2dPoints_cvmat() " << time_d2 - time_d << std::endl;
	clock_t time_2 = clock();
	std::cout << "    depth map: preparing voxel used " << time_2 - time << std::endl;

	//- DEBUG -
	Mat depth(480, 640, CV_8UC1, Scalar(0));

	time = clock();
	//reset the depth_map array and depth_voxel_id array
	for (int i = 0; i < prefHeight; i++) {
		for (int j = 0; j < prefWidth; j++) {
			depth_map[i][j] = 999999;
			depth_voxel_id[i][j] = -1;
		}
	}
	time_2 = clock();
	std::cout << "    depth map: re-initialize array " << time_2 - time << std::endl;
	
	time = clock();
	long sum = 0;
	for (int i = 0; i < total_voxel; i++) {
		//if voxel is not valid, don't do anything
		if (!voxel_valid[i]) continue;

		//get the voxel's x and y location in the image and round to integer
		int x = round(voxel_2D.at<float>(0, i));
		int y = round(voxel_2D.at<float>(1, i));
		if (x < 0 || y < 0) continue;
		if (x > prefWidth - 1 || y > prefHeight - 1) continue;

		clock_t time_l = clock();
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
		clock_t time_l2 = clock();
		sum += time_l2 - time_l;
	}
	time_2 = clock();
	std::cout << "    depth map: whole calculation " << time_2 - time << std::endl;
	std::cout << "        depth map: filling square in depth map " << sum << std::endl;

	//show_image("depth map", depth);
}

void vd_color_voxel(vector<Mat> cam_views, vector<Mat> images, int id, int x, int y) {
	update_voxel_validity();

	//reset all windows to unclicked state
	for (int i = 1; i <= images.size(); i++) {
		std::stringstream ss;
		ss << i;
		show_image(ss.str().c_str(), images.at(i - 1));
	}

	//invert the translation matrix back to the camera space
	Mat clicked_multimarker_mat = cam_views.at(id).inv();
	Mat cameraMat = get_camera_matrix();
	Mat voxel_3D = get_voxels();

	//construct the depeth map of the clicked window
	/*Mat voxel_2D = clicked_multimarker_mat * voxel_3D;
	Mat z_value_of_voxels = voxel_2D.clone();
	voxel_2D = cvt_3dPoints_2dPoints_cvmat(voxel_3D, clicked_multimarker_mat, cameraMat);*/
	construct_depth_map(clicked_multimarker_mat, depth_voxel_id, depth_map);

	//get the voxel that is clicked
	int clicked_voxel_id = depth_voxel_id[y][x];
	if (clicked_voxel_id == -1) return;
	temp_clicked_voxel_id = clicked_voxel_id;
	Mat clicked_voxel_3D = voxel_3D.col(clicked_voxel_id);
	
	int c_index = find_closest_view(clicked_voxel_id, clicked_multimarker_mat, id) + 1;
	std::cout << c_index << std::endl;
	
	Mat v_view(480, 640, CV_8UC3, Scalar(0,0,0));
	//for each voxel appeared in depth map id
	for (int y = 0; y < prefHeight; y++) {
		//std::cout << y << std::endl;
		for (int x = 0; x < prefWidth; x++) {
			if (depth_voxel_id[y][x] == -1) continue;

			//find the closest actual camera view and image
			int voxel_id = depth_voxel_id[y][x];
			int c_index = find_closest_view(voxel_id, clicked_multimarker_mat, id);
			//if (!is_visible_in_acam_view(voxel_id, closest_acam_index)) continue;
			Mat closest_image = get_all_captured_image()->at(c_index);
			Mat cloeset_acam_view = get_all_camera_view()->at(c_index).inv();


			//convert the voxel 3D location to 2D location
			Mat voxel_2D = get_voxels().col(voxel_id);
			//voxel_2D = shift_origin * voxel_2D;
			voxel_2D = cvt_3dPoints_2dPoints_cvmat(voxel_2D, cloeset_acam_view, get_camera_matrix());

			//get the color at <voxel_2D> on the image to color the voxel
			int image_x_loc = round(voxel_2D.at<float>(0, 0));
			int image_y_loc = round(voxel_2D.at<float>(1, 0));
			Vec3b new_color = closest_image.at<Vec3b>(image_y_loc, image_x_loc);
			voxels_color.at<Vec3b>(0, voxel_id) = new_color;
			is_voxel_colored[voxel_id] = true; //set colored flag to true
			v_view.at<Vec3b>(y, x) = new_color;
		}
	}
	show_image("v_view", v_view);

	for (int i = 1; i <= images.size(); i++) {
		Mat current_view = cam_views.at(i - 1).inv();
		
		Mat clicked_voxel_2D = cvt_3dPoints_2dPoints_cvmat(clicked_voxel_3D, current_view, cameraMat);
		/*voxel_2D = current_view * voxel_3D;
		z_value_of_voxels = voxel_2D.clone();
		voxel_2D = cvt_3dPoints_2dPoints_cvmat(voxel_3D, current_view, cameraMat);*/
		construct_depth_map(current_view, depth_voxel_id, depth_map);

		//get the location of this voxel in other windows
		int xx = round(clicked_voxel_2D.at<float>(0, 0));
		int yy = round(clicked_voxel_2D.at<float>(1, 0));
		if (xx < 0 || yy < 0 || xx >= prefWidth || yy >= prefHeight) continue;

		//highlight it with green circle (seen) or red circle (unseen)
		Mat image = images.at(i - 1).clone();
		if (i == c_index) circle(image, Point(xx, yy), 5, Scalar(0, 255, 255), -1);
		else if (is_in_depth_map(clicked_voxel_id)) circle(image, Point(xx, yy), 5, Scalar(0, 255, 0), -1);
		else circle(image, Point(xx, yy), 5, Scalar(0, 0, 255), -1);
		
		std::stringstream ss;
		ss << i;
		show_image(ss.str().c_str(), image);
	}
}

void init_acam_voxel2D() {
	Mat voxel_3D = get_voxels();
	vector<Mat>* camera_views = get_all_camera_view();

	clock_t time = clock();
	for (int i = 0; i < camera_views->size(); ++i){
		voxel_2D_vec.push_back(cvt_3dPoints_2dPoints_cvmat(voxel_3D, (*camera_views)[i].inv(), get_camera_matrix()));
	}
	clock_t time_2 = clock();
	std::cout << "preaparing voxel_2D_vec " << time_2 - time << std::endl;
}

void vd_texture_mapping() {
	if (!get_is_mapping()) return;
	clock_t time = clock();
	std::cout << degree << std::endl;
	vector<Mat>* captured_images = get_all_captured_image();
	vector<Mat>* camera_views = get_all_camera_view();
	Mat shift_origin = get_shift_origin();
	Mat voxel_3D = get_voxels();
	clock_t time_2 = clock();
	std::cout << "getting parameter from AR_routine used " << time_2 - time << std::endl;

	time = clock();
	float yaw = degree;
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
	time_2 = clock();
	std::cout << "define v_cam used " << time_2 - time << std::endl;

	time = clock();
	construct_depth_map(v_cam.inv(), depth_voxel_id, depth_map);
	time_2 = clock();
	std::cout << "construct depth map used " << time_2 - time << std::endl;

	time = clock();
	Mat v_image(480, 640, CV_8UC3);
	long sum = 0;
	long sum2 = 0;
	
	vector<float> c_values;
	for (int i = 0; i < camera_views->size(); i++) {
		float c = cal_3d_sqr_distance(camera_views->at(i) , v_cam);
		c_values.push_back(c);
	}
	time_2 = clock();
	std::cout << "preaparing c values " << time_2 - time << std::endl;
	if (voxel_2D_vec.empty()) init_acam_voxel2D();

	time = clock();
	//for each voxel appeared in depth map id
	for (int y = 0; y < prefHeight; y++) {
		for (int x = 0; x < prefWidth; x++) {
			if (depth_voxel_id[y][x] == -1) continue;
			//find the closest actual camera view and image
			int voxel_id = depth_voxel_id[y][x];
			clock_t time_l = clock();
			vector<int> c_index = find_closest_view_optimized(voxel_id, v_cam, -1, c_values);
			clock_t time_l2 = clock();
			sum += time_l2 - time_l;
			//if (!is_visible_in_acam_view(voxel_id, closest_acam_index)) continue;
			
			int r = 0, g = 0, b = 0;
			for (int i = 0; i < image_count; i++) {
				Mat closest_image = captured_images->at(c_index[i]);
				Mat voxel_2D = voxel_2D_vec[c_index[i]].col(voxel_id);
				int image_x_loc = round(voxel_2D.at<float>(0, 0));
				int image_y_loc = round(voxel_2D.at<float>(1, 0));
				if (image_x_loc < 0 || image_y_loc < 0 || image_x_loc > prefWidth - 1 || image_y_loc > prefHeight - 1) continue;

				Vec3b new_color = closest_image.at<Vec3b>(image_y_loc, image_x_loc);
				r += new_color.val[2] * image_weight[i];
				g += new_color.val[1] * image_weight[i];
				b += new_color.val[0] * image_weight[i];
			}

			Vec3b new_color(b / image_count, g / image_count, r / image_count);

			//get the color at <voxel_2D> on the image to color the voxel
			voxels_color.at<Vec3b>(0, voxel_id) = new_color;
			is_voxel_colored[voxel_id] = true; //set colored flag to true
			v_image.at<Vec3b>(y, x) = new_color;
		}
	}
	
	std::cout << "   find_closest_view " << sum << std::endl;
	//std::cout << "   cvt_3dPoints_2dPoints_cvmat " << sum2 << std::endl;
	time_2 = clock();
	std::cout << "loop though the depth map used " << time_2 - time << std::endl;

	time = clock();
	std::stringstream ss;
	//ss << yaw;
	show_image("v_image", v_image);
	waitKey(10);
	time_2 = clock();
	std::cout << "showing v_image used " << time_2 - time << std::endl;
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

//find the closest actual view from the v_cam
vector<int> find_closest_view_optimized(int voxel_id, Mat v_cam, int exclude_id, vector<float> c_values) {
	vector<float> closest_deg(image_count);
	vector<int> closest_index(image_count);
	for (int i = 0; i < image_count; i++) {
		closest_deg[i] = 999999;
	}
	//get all ACTUAL camera views and accquire 3D voxel location both of these are in MARKER CS
	vector<Mat>* cam_views = get_all_camera_view();
	Mat voxel_location = get_voxels().col(voxel_id);
	//voxel_location = get_shift_origin() * voxel_location;
	int closest_acam_index = 0;
	float cosine_deg = 999999;

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

/*void cal_depth_id_acam() {
	vector<Mat> acams = get_all_camera_view();
	std::cout << acam_depth_idmap.size()<< " " <<acams.size() << std::endl;
	if (acam_depth_idmap.empty()) {
		construct_depth_map(acams.at(0).inv());
		Mat depth_idmap_mat(480, 640, CV_32F, depth_voxel_id);
		acam_depth_idmap.push_back(depth_idmap_mat);
	}
	
	for (int i = acam_depth_idmap.size(); i < acams.size(); i++) {
		construct_depth_map(acams.at(i).inv());
		Mat depth_idmap_mat(480, 640, CV_32F, depth_voxel_id);
		acam_depth_idmap.push_back(depth_idmap_mat);
	}
}*/

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

//search through the depth id map to find the voxel_id
//return true if the voxel_id are in the map
//false otherwise
bool is_in_depth_map(int voxel_id) {
	if (voxel_id == -1) return false;
	for (int i = 0; i < prefHeight; i++) {
		for (int j = 0; j < prefWidth; j++) {
			if (depth_voxel_id[i][j] == voxel_id) return true;
		}
	}

	return false;
}

bool is_visible_in_acam_view(int voxel_id, int acam_id) {
	if (voxel_id == -1) return false;
	if (acam_id >= acam_depth_idmap.size()) {
		std::cout << acam_id << " overflowed " << acam_depth_idmap .size()<< std::endl;
	}
	for (int i = 0; i < prefHeight; i++) {
		for (int j = 0; j < prefWidth; j++) {
			if (acam_depth_idmap.at(acam_id).at<int>(i,j) == voxel_id) return true;
		}
	}

	return false;
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

float get_depth_at(int x, int y) {
	return depth_map[y][x];
}

int get_depth_id(int x, int y) {
	return depth_voxel_id[y][x];
}