#include <iostream>
#include "include/opencv2/core/core.hpp"
#include "include/opencv2/highgui/highgui.hpp"
#include "include/opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main() {
	Mat srcImg = imread("contour_test.png", IMREAD_GRAYSCALE);
	Canny(srcImg, srcImg, 100, 200, 3);

	//Mat dst = Mat::zeros(srcImg.rows, srcImg.cols, CV_8UC3);
	Rect bounding_rect;
	//vector<vector<Point>> contours; // Vector for storing contour
	//vector<Vec4i> hierarchy;
	
	vector<Mat> contours(1000);
	Mat hierarchy;

	int largest_area = 0;
	int largest_contour_index = 0;

	findContours(srcImg, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	// iterate through each contour.
	for (int i = 0; i< contours.size(); i++)
	{
		//  Find the area of contour
		double a = contourArea(contours[i], false);
		if (a>largest_area) {
			largest_area = a; cout << i << " area  " << a << endl;
			// Store the index of largest contour
			largest_contour_index = i;
			// Find the bounding rectangle for biggest contour
			bounding_rect = boundingRect(contours[i]);
		}
	}
	Scalar contourColor(255, 0, 0);
	drawContours(srcImg, contours, -1, contourColor, 2, 8, hierarchy);
	namedWindow("Display window", CV_WINDOW_AUTOSIZE);
	imshow("Display window", srcImg);
	waitKey(0);
	return 0;
}