#ifdef _WIN32
#include <windows.h>
#endif

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <thread>

#include <gl/GL.h>			// Header File For The OpenGL32 Library
#include <gl/GLU.h>			// Header File For The GLu32 Library
#include <GL/glut.h>

#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <AR/arMulti.h>

#include <opencv2/opencv.hpp>

#include "ar_routine.h"
#include "utilities.h"
#include "silhouette.h"
#include "voxel_construct.h"
#include "drawing.h"
#include "capturing.h"
#include "constants.h"

#ifdef _WIN32
char *vconf = "data/WDM_camera_flipV.xml";
#else
char *vconf = "";
#endif

using namespace cv;

ARParam camera_param;
Mat camera_mat(4, 4, CV_32F);

// Image acquisition.
ARUint8 *current_frame_ar = NULL;
VideoCapture vc;
Mat current_frame_cvmat;

// Marker detection.
bool patt_found = FALSE;

// Marker info
Mat shift_origin = (Mat_<float>(4, 4) << 1, 0, 0, 100, 0, 1, 0, -50, 0, 0, 1, 0, 0, 0, 0, 1); //shift the render origin
ARMultiMarkerInfoT *multimarker_info;
ARMarkerInfo *marker_info; // Pointer to array holding the details of detected markers.

// Drawing.
ARGL_CONTEXT_SETTINGS_REF argl_context_ref = NULL;
Mat voxels(4, total_voxel, CV_32F);
bool is_bg_ok = FALSE;
int suggested_direction;

// View dependence texture mapping
int closest_v_cam_index = -1;
bool is_mapping = false;


void camera_set_context(ARGL_CONTEXT_SETTINGS_REF ar_context) {
	argl_context_ref = ar_context;
}

void setup_voxels() {
	setup_default_bgColor();
	int i = 0;

	for (int x = first_voxel_x; x <= last_voxel_x; x++) {
		for (int y = first_voxel_y; y <= last_voxel_y; y++) {
			for (int z = first_voxel_z; z <= last_voxel_z; z++) {
				voxels.at<float>(0, i) = x; voxels.at<float>(1, i) = y; voxels.at<float>(2, i) = z; voxels.at<float>(3, i) = 1;
				i++;
			}
		}
	}
}

bool setup_camera() {
	ARParam oldparam;
	const char *param_name = "data/camera_para.dat";
	int w, h;

	if (arVideoOpen(vconf) < 0) return false;
	if (arVideoInqSize(&w, &h) < 0) return false;
	if (arParamLoad(param_name, 1, &oldparam) < 0) return false;
	arParamChangeSize(&oldparam, w, h, &camera_param);
	//arParamDisp(&param);
	arInitCparam(&camera_param);
	vc.open(0);

	camera_mat = cvt_trans_mat_to_cvmat(arParam.mat);
	return true;
}

bool setup_marker() {
	static const char *conf_name = "data/multi/marker.dat";

	if ((multimarker_info = arMultiReadConfigFile(conf_name)) == NULL) {
		printf("setup_marker() : Error loading config data %s for multiple markers\n", conf_name);
		return false;
	}
	return true;
}

void cleanup() {
	arVideoCapStop();
	arVideoClose();
	argCleanup();
}

void visibility_camera(int visible) {
	if (visible == GLUT_VISIBLE) glutIdleFunc(idle_camera);
	else glutIdleFunc(NULL);
}

void idle_camera(void) {

}

void display_camera(void) {
	int marker_num;	// Count of number of markers detected.
	Mat imgInCVMat_butARLikeThis;

	vc >> current_frame_cvmat;
	if (!current_frame_cvmat.empty()) {
		cvtColor(current_frame_cvmat, imgInCVMat_butARLikeThis, CV_BGR2BGRA);
		current_frame_ar = imgInCVMat_butARLikeThis.ptr();	// Save the fetched image.

		//detect the markers
		patt_found = FALSE;	// Invalidate any previous detected markers.

		//Detect the markers in the video frame.
		if (arDetectMarker(current_frame_ar, 100, &marker_info, &marker_num) < 0) cleanup();
		double err = arMultiGetTransMat(marker_info, marker_num, multimarker_info);
		if (err < 0.0 || err > 100.0) patt_found = FALSE;
		else patt_found = TRUE;
	}

	GLdouble p[16];
	GLdouble m[16];
	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.
	arglDispImage(current_frame_ar, &camera_param, 1.0, argl_context_ref);	// zoom = 1.0.
	arVideoCapNext();

	if (patt_found) {
		double finalPoint[4][4] = { {} };
		Mat multimarkerMat = cvt_trans_mat_to_cvmat(multimarker_info->trans);
		Mat temp = cvt_camCS_to_markerCS(multimarkerMat);
		cvt_trans_mat_to_array(multimarkerMat*shift_origin, finalPoint);

		// Projection transformation.
		arglCameraFrustumRH(&camera_param, VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, p);
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixd(p);
		glMatrixMode(GL_MODELVIEW);
		// Viewing transformation.
		glLoadIdentity();
		arglCameraViewRH(finalPoint, m, VIEW_SCALEFACTOR);
		glLoadMatrixd(m);

		draw();
	}
	drawHUD();

	glutSwapBuffers();
}

void reshape(int w, int h) {
	// Call through to anyone else who needs to know about window sizing here.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void drawHUD() {
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, prefWidth, prefHeight, 0, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPushMatrix();

	// marker detect indicator
	glBegin(GL_QUADS);
	if (patt_found) glColor3f(0.3f, 1.0f, 0.3f);
	else glColor3f(1.0f, 0.3f, 0.3f);
	glVertex2f(10, prefHeight - 10);
	glVertex2f(10, prefHeight - 30);
	glVertex2f(30, prefHeight - 30);
	glVertex2f(30, prefHeight - 10);

	// valid bg indicator
	if (is_bg_ok && patt_found) glColor3f(0.3f, 1.0f, 0.3f);
	else glColor3f(1.0f, 0.3f, 0.3f);
	glVertex2f(40, prefHeight - 10);
	glVertex2f(40, prefHeight - 30);
	glVertex2f(60, prefHeight - 30);
	glVertex2f(60, prefHeight - 10);
	glEnd();

	// suggest direction
	glPushMatrix();
	glTranslatef(prefWidth - 30, prefHeight - 30, 0);
	glRotatef(180, 1, 0, 0);
	glScalef(10.0, 10.0, 1);
	glLineWidth(5.0);
	//std::cout << direction << recommended_layer << std::endl;
	glCallList(*get_arrow(suggested_direction));
	glLineWidth(1.0);
	glPopMatrix();

	glPopMatrix();
	glEnable(GL_DEPTH_TEST);
}

void draw() {
	glPushMatrix();

	glColor3ub(255, 30, 30);
	glPointSize(10.0f);
	glBegin(GL_LINES);
	vector<Mat>* recommended_acam = get_recommended_acam();
	for (int i = 0; i < recommended_acam->size(); i++) {
		Mat v_cam = recommended_acam->at(i);
		if (get_is_captured_at(i)) continue;
		if (i == closest_v_cam_index) glColor3ub(255, 180, 180);
		else glColor3ub(255, 60, 60);
		glVertex3f(v_cam.at<float>(0, 3), v_cam.at<float>(1, 3), v_cam.at<float>(2, 3));
		glVertex3f(0, 0, 0);
	}
	glEnd();

	float yaw = 270;
	float pitch = v_cam_pitch;
	float x = v_cam_sphere_radius * cos(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
	float y = v_cam_sphere_radius * sin(deg_to_rad(yaw)) * sin(deg_to_rad(pitch));
	float z = v_cam_sphere_radius * cos(deg_to_rad(pitch));
	if (x <= 0.001 && x >=- 0.001) x = 0;
	if (y <= 0.001 && y >= -0.001) y = 0;
	if (z <= 0.001 && z >= -0.001) z = 0;
	Mat trans = cal_trans_mat(0, pitch, yaw, x, y, z);
	Mat to_origin = (Mat_<float>(4, 4) << 1, 0, 0, -x, 0, 1, 0, -y, 0, 0, 1, -z, 0, 0, 0, 1);
	Mat from_origin = (Mat_<float>(4, 4) << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1);
	trans = to_origin * trans;
	trans = trans.inv();
	trans = cal_trans_mat(0, 0, -90, 0, 0, 0) * trans;
	trans = cal_trans_mat(180, 0, 0, 0, 0, 0) * trans;
	trans = trans.inv();
	trans = from_origin * trans;
	//std::cout << yaw << " " << pitch << " " << x << " " << y << " " << z << std::endl;

	/*glPointSize(10.0f);
	glBegin(GL_POINTS);
	y = 300 * cos(deg_to_rad(carving_pitch_start));
	glColor3ub(0, 255, 0);
	glVertex3f(0, 0, y);
	y = 300 * cos(deg_to_rad(carving_pitch_end));
	glColor3ub(255, 0, 0);
	glVertex3f(0, 0, y);
	glEnd();*/

	/*trans = trans.t();
	glPushMatrix();
	glMultMatrixf(trans.ptr<float>());
	glutSolidCube(10);
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

	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glColor3ub(255, 0, 0);
	glVertex3f(20, 0, 0);
	glColor3ub(0, 255, 0);
	glVertex3f(0, 20, 0);
	glColor3ub(0, 0, 255);
	glVertex3f(0, 0, 20);
	glEnd();
	glPopMatrix();*/

	glPopMatrix();
}

//convert the AR multimarker translation matrix from
//MARKER position in CAMERA coordinates space to
//CAMERA position in MARKER coordinates space
Mat cvt_camCS_to_markerCS(Mat marker_position) {
	Mat cam_position = marker_position.clone();
	cam_position = cam_position * shift_origin;
	cam_position = cam_position.inv();
	return cam_position;
}

Mat get_voxels() {
	return voxels;
}

Mat get_shift_origin() {
	return shift_origin;
}

Mat get_camera_matrix() {
	return camera_mat;
}

Mat get_current_view() {
	Mat multimarkerMat = cvt_trans_mat_to_cvmat(multimarker_info->trans);
	return cvt_camCS_to_markerCS(multimarkerMat);
}

//- DEBUG -
int get_closest_view_index() {
	return closest_v_cam_index;
}

bool get_is_mapping() {
	return is_mapping;
}

void voxel_carving_routine() {
	for (;;) {
		if (patt_found && !is_mapping) {
			Mat points2D = cvt_3dPoints_2dPoints_array(voxels, multimarker_info->trans, arParam.mat);
			//if (determine_bgColor(current_frame_cvmat)) {
				is_bg_ok = TRUE;
				Mat silhouette = create_silhouette(current_frame_cvmat);
				shape_voxels(current_frame_cvmat, points2D, silhouette);
			//}
			//else is_bg_ok = FALSE;
		}
	}
}

void image_capturing_routine() {
	define_acam_recommendation();

	for (;;) {
		if (patt_found && !is_mapping) {
			Mat current_view = cvt_trans_mat_to_cvmat(multimarker_info->trans);
			current_view = cvt_camCS_to_markerCS(current_view);

			find_closest_acam(current_view, current_frame_cvmat.clone());
			suggested_direction = suggest_direction(current_view);
		}
	}
}

void debug_only(unsigned char key, int x, int y) {
	if (key == ' ') {
		//auto texture_mapping_t = std::thread(vd_texture_mapping);
		//texture_mapping_t.detach();
		//vd_texture_mapping();
		is_mapping = !is_mapping;
		update_voxel_validity();
	}
}