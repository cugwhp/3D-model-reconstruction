#ifdef _DEBUG
#pragma comment(lib, "libARd.lib")
#pragma comment(lib, "libARMultid.lib")
#pragma comment(lib, "libARgsub_lited.lib")
#pragma comment(lib, "libARgsubd.lib")
#pragma comment(lib, "libARvideo.lib")
#pragma comment(lib, "opencv_core249d.lib")
#pragma comment(lib, "opencv_highgui249d.lib")
#pragma comment(lib, "opencv_imgproc249d.lib")
#pragma comment(lib, "opencv_calib3d249d.lib")
#else 
#pragma comment(lib, "libAR.lib")
#pragma comment(lib, "libARMulti.lib")
#pragma comment(lib, "libARgsub_lite.lib")
#pragma comment(lib, "libARgsub.lib")
#pragma comment(lib, "libARvideo.lib")
#pragma comment(lib, "opencv_core249.lib")
#pragma comment(lib, "opencv_highgui249.lib")
#pragma comment(lib, "opencv_imgproc249.lib")
#pragma comment(lib, "opencv_calib3d249.lib")
#endif

#include <iostream>

#include <thread>
#include "ar_routine.h"
#include "voxel_construct.h"
#include "constants.h"
#include "capturing.h"

#include <AR/gsub_lite.h>
#include <GL/glut.h>

void TimeEvent(int te);

int camera_win_id;
int viewer_win_id;

int main(int argc, char** argv) {
	char glutGamemode[32];
	glutInit(&argc, argv);
	setup_camera();
	setup_marker();
	init_voxels_hp();

	// Set up GL context(s) for OpenGL to draw into.
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(prefWidth, prefHeight);
	camera_win_id = glutCreateWindow("Camera");
	glutTimerFunc(10, TimeEvent, 1);

	ARGL_CONTEXT_SETTINGS_REF argl_context_ref = NULL;
	// Setup argl library for current context.
	if ((argl_context_ref = arglSetupForCurrentContext()) == NULL) {
		std::cout << "main(): arglSetupForCurrentContext() returned error." << std::endl;
		exit(-1);
	}

	arUtilTimerReset();

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glutVisibilityFunc(visibility_camera);
	glutIdleFunc(idle_camera);
	glutDisplayFunc(display_camera);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(debug_only);
	setup_voxels();
	camera_set_context(argl_context_ref);
	
	viewer_win_id = glutCreateWindow("Viewer");
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0,0.0,0.0,1.0);
	glutVisibilityFunc(visibility_viewer);
	glutIdleFunc(idle_viewer);
	glutDisplayFunc(display_viewer);
	glutReshapeFunc(reshape_viewer);

	auto voxel_t = std::thread(voxel_carving_routine);
	auto capture_t = std::thread(image_capturing_routine);
	auto should_capture_t = std::thread(should_capture_routine);
	auto a_cam_buffer_t = std::thread(transfer);
	
	voxel_t.detach();
	capture_t.detach();
	should_capture_t.detach();
	a_cam_buffer_t.detach();
	glutMainLoop();

	return 0;
}

void TimeEvent(int te) {
	glutSetWindow(camera_win_id);
	glutPostRedisplay();

	glutSetWindow(viewer_win_id);
	glutPostRedisplay();

	glutTimerFunc(10, TimeEvent, 1);
}