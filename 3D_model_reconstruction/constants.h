#pragma once

/************************************************
        +-------------------------------+
 []=====|  ar_routine.cpp              /
        +-----------------------------+
************************************************/

//render parameters
#define VIEW_SCALEFACTOR		1			// 1.0 ARToolKit unit becomes 1 of my OpenGL units.
#define VIEW_DISTANCE_MIN		0.1			// Objects closer to the camera than this will not be displayed.
#define VIEW_DISTANCE_MAX		1000.0		// Objects further away from the camera than this will not be displayed.

//windows size and preferences
const int prefWidth = 640;					// Fullscreen mode width.
const int prefHeight = 480;					// Fullscreen mode height.
const int prefDepth = 32;					// Fullscreen mode bit depth.
const int prefRefresh = 0;					// Fullscreen mode refresh rate. Set to 0 to use default rate.

//voxels size
const int voxel_x = 101;					// total count of voxels in x axis
const int voxel_y = 101;					// total count of voxels in y axis
const int voxel_z = 101;					// total count of voxels in z axis
const int first_voxel_x = -voxel_x / 2;		// the co-ordinate on x axis of first voxels to be drawn
const int first_voxel_y = -voxel_y / 2;		// the co-ordinate on y axis of first voxels to be drawn
const int first_voxel_z = 0;				// the co-ordinate on z axis of first voxels to be drawn
const int last_voxel_x = (first_voxel_x + voxel_x) - 1;		// the co-ordinate on x axis of last voxels to be drawn
const int last_voxel_y = (first_voxel_y + voxel_y) - 1;		// the co-ordinate on y axis of last voxels to be drawn
const int last_voxel_z = (first_voxel_z + voxel_z) - 1;		// the co-ordinate on z axis of last voxels to be drawn
const int total_voxel = voxel_x * voxel_y * voxel_z;		// total number of voxels


/************************************************
        +-------------------------------+
 []=====|  silhouette.cpp              /
        +-----------------------------+
************************************************/

//silhoutte construction and background determination
const cv::Scalar fillColor(0, 0, 0);		// color to fill for pixel that consider as a background region
const int bg_remove_sensitivity = 96;		// sensitivity of removing background color, more => wider color range
const int bg_determine_sensitivity = 128;	// sensitivity of determining background color, more => wider color range
const int bgColor_determine_step = 30;		// pick a pixel at every bgColor_determine_step

// noise reduction - dilation and erosion
const int dilation_size = 2;				// size of the dilation
const int erosion_size = 3;					// size of the erosion


/************************************************
       +-------------------------------+
[]=====|  voxels_construct.cpp        /
       +-----------------------------+
************************************************/

//voxel model construction
const int voxel_max_hp = 10;				// 'hp' of each voxel
const char voxel_hp_regen_rate = 5;			// if regen hit this value, that voxel regenerate its hp by 1
const int voxel_size = 5;					// for doing a depth test, need to be odd number

//view dependence texture mapping
const float v_cam_sphere_radius = 350.0;	// the radius of the sphere centered at marker origin (middle of the marker)
const float v_cam_pitch = 90 - 45.0;		// pitch of the virtual camera, to change the pitch change the second number

const int image_count = 3;					// the number of images used for getting color of one voxel
const float image_weight[image_count] = { 1.5, 1.0, 0.5 };	// weight for each image for getting color of one voxel



/************************************************
	   +-------------------------------+
[]=====|  capturing.cpp               /
	   +-----------------------------+
************************************************/

//defining a recommended candidate
const float acam_rec_sphere_radius = 400.0;			// the radius of the sphere of all acam recommended position
const int yaw_points = 15;							// the number of points in each layer of pitch
const int pitch_points = 5;							// the number of points in each layer of yaw
const float starting_pitch = 15.0;					// pitch of the first layer of recommended actual camera
const int reduce_yaw_step = 3;						// subtract the number of points for every pitch layer drawn, the higher the pitch the lower amount of points

//camera movement suggestion
const int capture_trigger_distance = 50 * 50;		// the square distance between camera and acam recommended view need to be less than this value to trigger image capturing
const int carving_pitch = 2;						// the decided layer to guide the user for voxel carving process
const float pitch_threshold = 5;					// the width of each pitch layer, define by layer center plus/minus pitch threshold

//'should capture from this point or not' criteria
const int candidate_distance = 300 * 300;			// consider all of the candidate in this length away from user camera position
const int var_error_checking_percentile = 90;		// percentile of the variances that use to decide the color consistency
const float color_error_threshold = 50 * 50;		// [TODO] a view cosider as not need to capture if the variance is less than this