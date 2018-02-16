#include <iostream>
#include <math.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "disjointSet.h"
#include "cameraParam.h"
#include "coordinateUtils.h"
using namespace std;
using namespace cv;


#define IMG_PATH_PREFIX string("../imgs/rgbd/img-")
#define DEPTH_PATH_PREFIX string("../imgs/rgbd/depth-")
#define IMG_POST_FIX string(".jpg")
#define DEPTH_POST_FIX string(".png")
#define RESULT_PREFIX string("../imgs/")
// threshold function
#define THRESHOLD(num, threshold) (threshold/num)

namespace felzenszwalb_Seg
{
	template <class T>
	inline T square(const T &x) { return x*x; };
	static inline float diff(const Mat& r, const Mat& g, Mat& b,
		int x1, int y1, int x2, int y2) {
		return sqrt(square(r.at<uchar>(y1, x1) - r.at<uchar>(y2, x2)) +
			square(g.at<uchar>(y1, x1) - g.at<uchar>(y2, x2)) +
			square(b.at<uchar>(y1, x1) - b.at<uchar>(y2, x2)));
	}
	typedef struct {
		int src;
		int dst;
		float weight;
	}Edge;
	bool weightSortFunc(Edge a, Edge b) { return (a.weight < b.weight); }
	universe* do_segmentation_on_graph(Edge* edges, int vertice_num, int edge_num, float threshold) {
		//sort edges by weight
		sort(edges, edges + edge_num, weightSortFunc);
		//create new disjoint-set group 
		universe * u = new universe(vertice_num);
		float * threshold_lst = new float[vertice_num];
		
		//initialize threshold
		for (int i = 0; i < vertice_num; i++)
			threshold_lst[i] = THRESHOLD(1, threshold);
		
		for (int i = 0; i < edge_num; i++) {
			Edge* e = &edges[i];
			int src = u->find(e->src);
			int dst = u->find(e->dst);
			if (src != dst) {
				if ((e->weight <= threshold_lst[src]) && (e->weight <= threshold_lst[dst])) {
					u->join(src, dst);
					src = u->find(src);
					threshold_lst[src] = e->weight+ THRESHOLD(u->size(src), threshold);
				}
			}
		}
		delete threshold_lst;
		return u;
	}
	/*
	* Segment an rgbd image
	*
	* Returns a color image representing the segmentation.
	*
	* srcImg: image to segment.(in RGB format)
	* depthImg: depth of image
	* output: segmented image
	* sigma: to smooth the image.
	* threshold: constant for threshold function.
	* min_size: minimum component size (enforced by post-processing stage).
	* num_ccs: number of connected components in the segmentation.
	* depth_threshold: we won't consider a pair of pixels if the depth difference
	* exceeds this
	*/
	void felzenswalb_segmentation(const Mat& srcImg, const Mat& depthImg, Mat& output, float sigma, float threshold,
		int min_size, float depth_threshold) {
		int width = srcImg.cols;
		int height = srcImg.rows;
		Mat srcRGB[3];
		split(srcImg, srcRGB);//split source
		//cout <<(int) srcRGB[0].at<uchar>(0, 0) << endl;
		//cout << diff(srcRGB[0], srcRGB[1], srcRGB[2], 0,0,0,1)<<endl;
		//Step 1: Do Gaussian Blur on each channel
		GaussianBlur(srcRGB[0], srcRGB[0], Size(3, 3), sigma, sigma);
		GaussianBlur(srcRGB[1], srcRGB[1], Size(3, 3), sigma, sigma);
		GaussianBlur(srcRGB[2], srcRGB[2], Size(3, 3), sigma, sigma);

		//Step 2: Build Graph
		Edge* edges = new Edge[height * width * 4];
		int edge_num = 0;
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				if (y < height - 1 && 
					fabs(depthImg.at<uchar>(y, x) - depthImg.at<uchar>(y + 1, x)) <= depth_threshold) {
						
					edges[edge_num].src = y * width + x;
					edges[edge_num].dst = (y + 1) * width + x;
					edges[edge_num].weight = diff(srcRGB[0], srcRGB[1], srcRGB[2], x, y, x, y + 1);
					edge_num++;
				}
				if (x < width - 1 &&y<height-1&&
					fabs(depthImg.at<uchar>(y, x) - depthImg.at<uchar>(y + 1, x + 1)) <= depth_threshold) {
						
					edges[edge_num].src = y * width + x;
					edges[edge_num].dst = (y + 1) * width + x + 1;
					edges[edge_num].weight = diff(srcRGB[0], srcRGB[1], srcRGB[2], x, y, x + 1, y + 1);
					edge_num++;
				}
				if (x < width - 1 &&
					fabs(depthImg.at<uchar>(y, x) - depthImg.at<uchar>(y, x + 1)) <= depth_threshold) {
						
					edges[edge_num].src = y * width + x;
					edges[edge_num].dst = y * width + x + 1;
					edges[edge_num].weight = diff(srcRGB[0], srcRGB[1], srcRGB[2], x, y, x + 1, y);
					edge_num++;
				}
				if (x < width - 1 && y>0 && 
					fabs(depthImg.at<uchar>(y, x) - depthImg.at<uchar>(y - 1, x + 1)) <= depth_threshold) {
						
					edges[edge_num].src = y * width + x;
					edges[edge_num].dst = (y - 1) * width + x + 1;
					edges[edge_num].weight = diff(srcRGB[0], srcRGB[1], srcRGB[2], x, y, x + 1, y - 1);
					edge_num++;
				}
			}
		}

		//Step 3. Do Segmentation:
		universe* u = do_segmentation_on_graph(edges, width*height, edge_num, threshold);

		for (int i = 0; i < edge_num; i++) {
			int src = u->find(edges[i].src);
			int dst = u->find(edges[i].dst);
			if ((src != dst) &&
				(u->size(src) < min_size ||	u->size(dst) < min_size))
				u->join(src, dst);
		}
		delete[] edges;
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				int comp = u->find(y*width + x);
				output.at<uchar>(y, x) = (uchar)comp;
			}
		}
		delete u;
	}
}
using namespace felzenszwalb_Seg;

void ComputeHeightMap(string extFileName, const Mat_<float>& intri, const Mat& depthImg, Mat& output) {
	float data[9] = { 9.9997798940829263e-01, 5.0518419386157446e-03,
		4.3011152014118693e-03, -5.0359919480810989e-03,
		9.9998051861143999e-01, -3.6879781309514218e-03,
		- 4.3196624923060242e-03, 3.6662365748484798e-03,
		9.9998394948385538e-01 };

	Mat R(3, 3, CV_32F, data);
	R = R.t().inv();

	Vec3f t(2.5031875059141302e-02, 6.6238747008330102e-04,-2.9342312935846411e-04);
	//getExtrinMatrix(extFileName, R, t);
	Mat_<float> tmp = -R.inv();
	Mat_<float> cameraCenter = R *Mat_<float>(t);

	//uext = [1 0 0; 0 0 1; 0 - 1 0] * R*[1 0 0; 0 0 - 1; 0 1 0];
	Mat_<float> M = (intri * R).inv();
	Mat_<float>result(output.size(), output.type());
	//cout <<"M=" <<M << endl;
	for (int i = 0; i < depthImg.rows; i++) {
		//const uchar * ptr_src = depthImg.ptr<uchar>(i);
		//float * ptr_dst = result.ptr<float>(i);
		for (int j = 0; j < depthImg.cols; j++) {
			float tmp = (float)depthImg.at<uchar>(i, j);

			float point[3] = { i, j, tmp};

			Mat_<float> dstPoint = M * Mat_<float>(Mat(3, 1, CV_32F, point)) + cameraCenter;

			result[i][j] = dstPoint[2][0];
		}
	}

	//TODO: need to findout
	/*double minimal = output.at<double>(400, 200);
	for (int i = 0; i < output.rows; i++)
	{
		double* Mi = output.ptr<double>(i);
		for (int j = 0; j < output.cols; j++)
			Mi[j] -= minimal;
	}*/
	//cout << result;
	double min, max;
	int* minIdx, maxIdx;

	cv::minMaxLoc(result, &min, &max);
	double gap = max - min;
	//float minimal = result[400][200];
	//float minimal = (float)output.at<uchar>(400, 200);
	for (int i = 0; i < depthImg.rows; i++) {
		uchar * ptr_dst = output.ptr<uchar>(i);
		for (int j = 0; j < depthImg.cols; j++) {
			ptr_dst[j] =(uchar) ((result[i][j] - min) / gap * 255);

		}
	}
}

void GraphSegmentation(string line_img, string line_depth, string line_ext, const Mat_ < float > & intri, int count) {
	//source image in RGB format
	Mat srcImg = imread(line_img);
	//depth image in grayscale
	Mat depthImg = imread(line_depth,CV_LOAD_IMAGE_GRAYSCALE);
	//namedWindow("the original image", WINDOW_NORMAL);
	//imshow("the original image", srcImg);
	if (depthImg.empty() || srcImg.empty())
	{
		cout << "Could not open or find the image" << endl;
	}

	Mat output_seg(depthImg.size(), depthImg.type());
	felzenswalb_segmentation(srcImg, depthImg, output_seg, 1.0f, 5000, 3000, 10000);
	Mat output_height(depthImg.size(), depthImg.type());
	ComputeHeightMap(line_ext,  intri, depthImg, output_height);

	/*imwrite(RESULT_PREFIX + to_string(count) + string("-r.bmp"), output_seg);
	imwrite(RESULT_PREFIX + to_string(count) + string("-o.bmp"), srcImg);
	imwrite(RESULT_PREFIX +to_string(count)+ string("-d.bmp"), depthImg);
	imwrite(RESULT_PREFIX + to_string(count) + string("-h.bmp"), output_height);*/

	namedWindow("height", CV_WINDOW_AUTOSIZE);
	imshow("height", output_height);

	namedWindow("depth", CV_WINDOW_AUTOSIZE);
	imshow("depth", depthImg);

	namedWindow("output_seg", CV_WINDOW_AUTOSIZE);
	imshow("output_seg", output_seg);
	while (char(waitKey(0))) {};

}

int nyu2d_single() {
	string imgName("../imgs/raw/1.ppm");
	string depthName("../imgs/raw/1.pgm");
	Mat srcImg = imread(imgName);
	Mat rawDepth_c = imread(depthName, CV_LOAD_IMAGE_COLOR);
	Mat rawDepth;
	cvtColor(rawDepth_c, rawDepth, CV_RGB2GRAY);

	Mat absDepth(rawDepth.size(), CV_32F);
	int N = rawDepth.rows * rawDepth.cols;
	Mat point3d(N, 3, CV_32F);
	Mat point3d_final(N, 1, CV_32F);
	coordinatesUtils::depth_rel2depth_abs(rawDepth, absDepth);
	coordinatesUtils::depth_plane2depth_world(absDepth, point3d);
	coordinatesUtils::depth_world2rgb_world(point3d, point3d_final);

	Mat result = point3d_final.reshape(0, rawDepth.rows);


	Mat_<uchar>output_height(rawDepth.size());
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;

	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
	double gap = maxVal - minVal;
	for (int i = 0; i < rawDepth.rows; i++) {
		uchar * ptr_dst = output_height.ptr<uchar>(i);
		for (int j = 0; j < rawDepth.cols; j++) {
			ptr_dst[j] = (uchar)((result.at<float>(i,j) - minVal) / gap * 255);
		}
	}
	Mat output_seg(rawDepth.size(), rawDepth.type());
	felzenswalb_segmentation(srcImg, rawDepth, output_seg, 1.0f, 5000, 3000, 10000);

	namedWindow("source Image", CV_WINDOW_AUTOSIZE);
	imshow("source Image", srcImg);
	namedWindow("segmentation", CV_WINDOW_AUTOSIZE);
	imshow("segmentation", output_seg);
	namedWindow("height", CV_WINDOW_AUTOSIZE);
	imshow("height", output_height);

	imwrite(RESULT_PREFIX + string("-r.bmp"), output_seg);
	imwrite(RESULT_PREFIX + string("-o.bmp"), srcImg);
	imwrite(RESULT_PREFIX + string("-d.bmp"), rawDepth);
	imwrite(RESULT_PREFIX  + string("-h.bmp"), output_height);
	while (char(waitKey(0))) {};

	return 0;
}
/*int main() {
	return nyu2d_single();
}*/
/*int main() {
	ifstream infile("../imgs/imageName.txt");
	string line_img, line_depth, line_ext, line_intri;
	vector<string>image_lines;
	vector<string>depth_lines;
	vector<string>ext_lines;
	int num = 0;
	float int_data[9] = { 574.334300, 0.000000, 320.000000,
		0.000000, 574.334300, 240.000000,
		0.000000, 0.000000, 1.000000};
	Mat intri_raw(3, 3, CV_32F, int_data);
	Mat_<float> intri(intri_raw);
	//cout << intri << endl;
	if (infile.is_open()) {
		getline(infile, line_intri);
		while (infile.good()) {
			getline(infile, line_ext);
			getline(infile, line_img);
			getline(infile, line_depth);
			ext_lines.push_back(line_ext);
			image_lines.push_back(line_img);
			depth_lines.push_back(line_depth);
			//cout << line_img << endl;
			//cout << line_depth << endl;
			num++;
		}
	}

	for (int count = 0; count < 2; count++) {
		GraphSegmentation(image_lines[count], depth_lines[count], ext_lines[count],intri, count);
		
	}
		
	return 0;
}*/
/*int main() {
	string imgName("../imgs/raw/1.ppm");
	string depthName("../imgs/raw/1.png");
	//string imgName(IMG_PATH_PREFIX + string("1") + IMG_POST_FIX);
	//string depthName(DEPTH_PATH_PREFIX + string("1") + DEPTH_POST_FIX);
	//source image in RGB format
	Mat srcImg = imread(imgName);
	//depth image in grayscale
	Mat depthImg = imread(depthName, CV_LOAD_IMAGE_GRAYSCALE);
	//cout << depthImg;
	//Mat depthImg2 = imread(depthName);
	//cout << depthImg2;
	if (depthImg.empty() || srcImg.empty())
	{
		cout << "Could not open or find the image" << endl;
		return -1;
	}

	Mat output(depthImg.size(), depthImg.type());
	felzenswalb_segmentation(srcImg, depthImg, output, 1.0f, 5000, 3000, 10000);
	/*bool test = imwrite(IMG_PATH_PREFIX + to_string(sigma) + string("-") + to_string(threshold) + string("-") + to_string(min_size) + string("-") + to_string(depth_threshold) + string(".bmp"), output);
	for (float sigma = .0f; sigma < 5.0f; sigma+=1) {
		for (float threshold = 500.f; threshold < 2000; threshold += 200) {
			for (int min_size = 1000; min_size < 3000; min_size += 200) {
				for (float depth_threshold = 1000.f; depth_threshold < 3000; depth_threshold += 200) {
					felzenswalb_segmentation(srcImg, depthImg, output, sigma, threshold, min_size, depth_threshold);
					bool test = imwrite(IMG_PATH_PREFIX + to_string(sigma) + string("-") + to_string(threshold) + string("-") + to_string(min_size) + string("-") + to_string(depth_threshold) + string(".bmp"), output);
				}
			}
		}
	}*/

	

	//namedWindow("the original image", WINDOW_NORMAL);
	//imshow("the original image", srcImg);
	/*namedWindow("the Segmented image", WINDOW_NORMAL);
	imshow("the Segmented image", output);

	while (char(waitKey(0))) {};
	return 0;
}*/