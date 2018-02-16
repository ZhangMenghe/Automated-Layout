#pragma once
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "cameraParam.h"
using namespace cv;
using namespace std;
namespace coordinatesUtils {
	//input: readin depth image(as mat)
	//output: same size float depth matrix
	inline void depth_rel2depth_abs(const Mat& rawDepth, Mat& absDepth) {

		for (int i = 0; i < rawDepth.rows; i++) {
			float * ptr_dst = absDepth.ptr<float>(i);
			for (int j = 0; j < rawDepth.cols; j++) {
				float tdepth = (float)rawDepth.at<uchar>(i, j);
				float cdepth = depthParam1 / (depthParam2 -  tdepth);
				ptr_dst[j] = (cdepth > maxDepth) ? maxDepth : cdepth;
				ptr_dst[j] = (cdepth < 0) ? 0 : cdepth;				
			}
		}
	}
	//input:float depth matrix
	//output: Nx3 3d points
	inline void depth_plane2depth_world(const Mat& absDepth, Mat& points3d) {
		Mat_<float>result(absDepth.rows*absDepth.cols,3, absDepth.type());
		int rIdx = 0;
		for (int yy = 0; yy < absDepth.rows; yy++) {
			for (int xx = 0; xx < absDepth.cols; xx++) {
				float depth_abs = absDepth.at<float>(yy, xx);
				float X = (xx - cx_d) * depth_abs / fx_d;
				float Y = (yy - cy_d) * depth_abs / fy_d;

				result[rIdx][0] = X;
				result[rIdx][1] = Y;
				result[rIdx][2] = depth_abs;
				rIdx++;
			}
		}
		points3d = result;
	}
	//input: Nx3 3d points
	//output
	inline void depth_world2rgb_world(Mat& points3d_depth, Mat& points3d_rgb) {
		//Mat_<float> R(3, 3, R_raw);
		//R = R.t().inv();
		float data[9] = { -1.0000,    0.0050,    0.0043,
			- 0.0051, - 1.0000, - 0.0037,
			- 0.0043 ,   0.0037, - 1.0000 };
		Mat_<float> R(3, 3, data);
		Mat_<float> T(3, 1, T_raw);
		Mat_ <float>result(points3d_depth.rows, 1, points3d_depth.type());
		for (int idx = 0; idx < points3d_depth.rows; idx++) {
			float * ptr_dst = points3d_depth.ptr<float>(idx);
			Mat dstPoint = R* Mat_<float>(3, 1, ptr_dst) + T;
			result[idx][0] = dstPoint.at<float>(1, 0);
			//result[idx][1] = dstPoint.at<float>(1, 0);
			//result[idx][2] = dstPoint.at<float>(2, 0);
		}
		points3d_rgb = result;
	}
}
