#include <iostream>
#include <opencv2/opencv.hpp>
#include "silhouette.h"
#include "constants.h"

using namespace cv;

std::vector<Scalar> bgColors;

Mat create_silhouette(Mat image) {
	vector<Mat> reducedBG;
	Mat result = image.clone();

	for (int i = 0; i < bgColors.size(); i++) {
		Scalar bgColor = bgColors.at(i);
		Mat image_clone = image.clone();

		for (int i = 0; i < image_clone.rows; i++) {
			for (int j = 0; j < image_clone.cols; j++) {
				int b = image_clone.at<Vec3b>(i, j).val[0] - bgColor.val[0];
				int g = image_clone.at<Vec3b>(i, j).val[1] - bgColor.val[1];
				int r = image_clone.at<Vec3b>(i, j).val[2] - bgColor.val[2];

				b = abs(b); g = abs(g); r = abs(r);
				int totalDiff = b + g + r;

				if (totalDiff < bg_remove_sensitivity) {
					//fill the pixel considered as background with decided color
					Vec3b px = image_clone.at<Vec3b>(i, j);
					image_clone.row(i).col(j) = fillColor;
				}
			}
		}
		result = image_clone & result;
	}

	remove_noise(result);
	return result;
}

bool determine_bgColor(Mat image) {
	int r = image.at<Vec3b>(0, 0)[2], g = image.at<Vec3b>(0, 0)[1], b = image.at<Vec3b>(0, 0)[0];
	int totalHitSample = 1;
	for (int i = 0; i < image.rows; i += bgColor_determine_step) {
		for (int j = 0; j < image.cols; j += bgColor_determine_step) {
			
			int bCurrent = image.at<Vec3b>(i, j)[0];
			int gCurrent = image.at<Vec3b>(i, j)[1];
			int rCurrent = image.at<Vec3b>(i, j)[2];

			int bDiff = bCurrent - (b / totalHitSample);
			int gDiff = gCurrent - (g / totalHitSample);
			int rDiff = rCurrent - (r / totalHitSample);

			bDiff = abs(bDiff); gDiff = abs(gDiff); rDiff = abs(rDiff);
			int diff = bDiff + gDiff + rDiff;
			if (diff < bg_determine_sensitivity) {
				totalHitSample++;
				r += rCurrent; g += gCurrent; b += bCurrent;
			}
		}
	}

	r /= totalHitSample; g /= totalHitSample; b /= totalHitSample;

	Scalar newBG(b, g, r);
	int diff = (b - bgColors[1][0]) + (g - bgColors[1][1]) + (b - bgColors[1][2]);
	diff = abs(diff);
	if (diff < 64) {
		bgColors[1] = newBG;
		return true;
	}
	
	return false;
}

// remove noise of the image by appling erosion then dilation
void remove_noise(Mat image) {
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
	erode(image, image, element);

	element = getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size + 1), Point(dilation_size, dilation_size));
	dilate(image, image, element);
}

void setup_default_bgColor() {
	bgColors.push_back(Scalar(20, 20, 20));
	bgColors.push_back(Scalar(160, 160, 160));
}