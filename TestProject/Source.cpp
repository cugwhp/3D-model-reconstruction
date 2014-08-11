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
#include <math.h>
#include <ctime>
#include <opencv2/opencv.hpp>

using namespace cv;

void test_multiMin(int num[15]);
void test_pow();
void test_array_indexing();
int get_element_at(int index);
int* get_array();

int foo[5] = { 5, 2, 3, 4, 1 };

void test_multiMin(int num[15]) {
	int image_count = 3;
	vector<float> closest_deg(image_count);
	vector<int> closest_index(image_count);
	for (int i = 0; i < image_count; i++) {
		closest_deg[i] = 999999;
	}

	for (int input = 0; input < 15; input++) {
		for (int j = 0; j < image_count; j++) {
			if (num[input] < closest_deg[j]) {
				if (closest_deg[j] != 999999) {
					for (int k = image_count - 1; k >= j; k--) {
						if (k + 1 >= image_count) continue;
						closest_deg[k + 1] = closest_deg[k];
						closest_index[k + 1] = closest_index[k];

					}
				}
				closest_deg[j] = num[input];
				closest_index[j] = input;
				break;
			}
		}
	}

	for (int i = 0; i < image_count; i++) {
		std::cout << closest_deg[i] << " ";
	}
	std::cout << std::endl;
	for (int i = 0; i < image_count; i++) {
		std::cout << closest_index[i] << " ";
	}
}

void test_pow() {
	float x1 = 1;
	float x2 = 3;
	float y1 = 1;
	float y2 = 3;
	float z1 = 2;
	float z2 = 5;

	float number = 0;
	clock_t time = clock();
	for (int i = 0; i < 100000000; i++) {
		number = pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2);
	}
	clock_t time2 = clock();
	std::cout << "pow " << time2 - time << std::endl;
	std::cout << number << std::endl;

	number = 2;
	time = clock();
	for (int i = 0; i < 100000000; i++) {
		number = ((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1));
	}
	time2 = clock();
	std::cout << "mul " << time2 - time << std::endl;
	std::cout << number << std::endl;

	
}

void test_array_indexing() {
	unsigned long sum = 0;
	clock_t time = clock();
	for (int i = 0; i < 100000000; i++) {
		sum += get_element_at(5);
	}
	clock_t time2 = clock();
	std::cout << sum << " get_element_at " << time2 - time << std::endl;

	sum = 0;
	time = clock();
	for (int i = 0; i < 100000000; i++) {
		sum += get_array()[5];
	}
	time2 = clock();
	std::cout << sum << " get_array " << time2 - time << std::endl;
}

int get_element_at(int index) {
	return foo[index];
}

int* get_array() {
	return foo;
}

int main(int argc, char** argv) {
	//8 5 0
	//1 2 3
	int nums[15] = { 3, 8, 9, 4, 6, 2, 70, 21, 1, 50, 33, 41, 90, 102, 64 };
	
	test_array_indexing();
	char c;
	std::cin >> c;
	return 0;
}

