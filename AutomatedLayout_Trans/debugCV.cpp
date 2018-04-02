#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
using namespace cv;
using namespace std;
int main()
{
	Mat src;
	// the first command-line parameter must be a filename of the binary
	// (black-n-white) image
	//if (argc != 2 || !(src = imread(argv[1], 0)).data)
	//	return -1;
	src = imread("E:/contour_test.png", 0);
	Mat dst = Mat::zeros(src.rows, src.cols, CV_8UC1);
	src = src > 1;
	namedWindow("Source", 1);
	imshow("Source", src);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	vector<Point> contour;
	contour.push_back(Point(0, 0));
	contour.push_back(Point(0, 20));
	contour.push_back(Point(20, 20));
	contour.push_back(Point(20, 0));
	contours.push_back(contour);


	//findContours(src, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	/*int idx = 0;
	for (; idx >= 0; idx = hierarchy[idx][0])
	{
		Scalar color(rand() & 255, rand() & 255, rand() & 255);
		drawContours(dst, contours, idx, color, FILLED, 8, hierarchy);
	}*/
	drawContours(dst, contours, -1, 1, FILLED, 8);
	cout << cv::sum(dst) << endl;
	namedWindow("Components", 1);
	imshow("Components", dst);
	waitKey(0);
}
