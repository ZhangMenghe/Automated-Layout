#pragma once
#include "include\opencv2\core\core.hpp"
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;

void graph_to_card(float half_width, float half_height, vector<Point2f> & rect) {
	for (vector<Point2f>::iterator it = rect.begin(); it != rect.end(); it++) {
		it->x -= half_width;
		it->y = half_height - it->y;
	}
}
void line_equation_2point(const Point2f &p1, const Point2f &p2, float& k, float&b) {
	//(y-y2)/(y1-y2) = (x-x2)/(x1-x2)
	k = (p2.y - p1.y) / (p2.x - p1.x);
	b = p1.y - k * p1.x;
}

Point2f point_proj_line(const Point2f &p, const float k, const float b) {
	float tx = (k*(p.y - b) + p.x) / (k*k + 1);
	return Point2f(tx, k*tx + b);
}
void get_bounding_xy(vector<Point2f> &r, const float cx, const int boundIdxStart, const float& k, const float& b, float* boundValue, int* boundIdx) {
	for (int i = 0; i < 4; i++) {
		// comapre project y 
		float projY = point_proj_line(r[i], k, b).y;
		if (projY > boundValue[3]) {
			boundValue[3] = projY;
			boundIdx[3] = boundIdxStart + i;
		}
		if (projY < boundValue[2]) {
			boundValue[2] = projY;
			boundIdx[2] = boundIdxStart + i;
		}
		//compare dist X
		float cdist = fabs(k * r[i].x - r[i].y + b);
		if (r[i].x > cx && cdist>boundValue[1]) {
			boundValue[1] = cdist;
			boundIdx[1] = boundIdxStart + i;
		}
		if (r[i].x < cx && cdist>boundValue[0]) {
			boundValue[0] = cdist;
			boundIdx[0] = boundIdxStart + i;
		}
	}
}
void get_bounding_vertices(const float * bound_ks, const float *bound_bs, vector<Point2f>& vertices) {
	int realIdx[4] = { 0,3,1,2 };
	for (int i = 0; i < 4; i++) {
		float interX = -(bound_bs[realIdx[(i + 1) % 4]] - bound_bs[realIdx[i]]) / (bound_ks[realIdx[(i + 1) % 4]] - bound_ks[realIdx[i]]);
		float interY = bound_ks[realIdx[i]] * interX + bound_bs[realIdx[i]];
		vertices.push_back(Point2f(interX, interY));
	}
}
void write_out_file(vector<vector<Point2f>> rects) {
	ofstream outfile;
	outfile.open("test.txt", 'w');
	if (outfile.is_open()) {
		outfile << "DEBUG_DRAW\t|\tvertices\r\n";
		for (vector<vector<Point2f>>::iterator rect = rects.begin(); rect != rects.end(); rect++) {
			for (vector<Point2f>::iterator point = rect->begin(); point != rect->end(); point++)
				outfile << "[" << to_string(point->x) << ", " << to_string(point->y) << "]\t|\t";
			outfile << "\r\n";
		}
	}
	outfile.close();
}

vector<float> processFixedObjects(const vector<float>& parameter) {

}

vector<float> mergeAgroup(const vector<vector<float>> &parameters, const vector<float>groupIds) {

}
void merge2Obj(const vector<float> &parameter) {
	vector<Point2f> rect1 = { Point2f(150,100), Point2f(350, 100), Point2f(350,200),Point2f(150,200) };
	vector<Point2f> rect2 = { Point2f(400,200), Point2f(500,200), Point2f(500,300), Point2f(400,300) };
	vector<Point2f> res;
	vector<vector<Point2f>> rectVector;

	float half_width = 300, half_height = 200;
	float equal_thresh = 10.0;

	graph_to_card(half_width, half_height, rect1);
	graph_to_card(half_width, half_height, rect2);
	Point2f c1 = (rect1[0] + rect1[2]) / 2.0f;
	Point2f c2 = (rect2[0] + rect2[2]) / 2.0f;
	rectVector.push_back(rect1);
	rectVector.push_back(rect2);

	// construct line equation from c1 c2
	if (fabs(c1.x - c2.x)<equal_thresh || fabs(c1.y - c2.y)<equal_thresh) {
		float minX = std::min({ rect1[0].x, rect1[3].x, rect2[0].x, rect2[3].x });
		float maxX = std::max({ rect1[1].x, rect1[2].x, rect2[1].x, rect2[2].x });
		float minY = std::min({ rect1[0].y, rect1[1].y, rect2[0].y, rect2[1].y });
		float maxY = std::max({ rect1[2].y, rect1[3].y, rect2[2].y, rect2[3].y });
		res = { Point2f(minX, maxY), Point2f(maxX, maxY), Point2f(maxX, minY),Point2f(minX,minY) };
	}
	else {
		// minX, maxX, minY, maxY
		float boundValue[4] = { -INFINITY, -INFINITY, INFINITY,-INFINITY };
		int boundIdx[4] = { -1 };
		float pk; float pb;
		line_equation_2point(c1, c2, pk, pb);

		float vk = -1 / pk; float vb;
		//float vb = b = p.y + 1 / k * p.x;
		get_bounding_xy(rect1, c1.x, 0, pk, pb, boundValue, boundIdx);
		get_bounding_xy(rect2, c2.x, 4, pk, pb, boundValue, boundIdx);
		//get four bounding line equation
		Point2f pointBounding[4];
		for (int i = 0; i<4; i++)
			pointBounding[i] = rectVector[boundIdx[i] / 4][boundIdx[i] % 4];
		float bound_bs[4];
		float bound_ks[4] = { pk,pk,vk,vk };
		for (int i = 0; i < 4; i++)
			bound_bs[i] = pointBounding[i].y - bound_ks[i] * pointBounding[i].x;
		get_bounding_vertices(bound_ks, bound_bs, res);
	}
	cout << res << endl;
	rectVector.push_back(res);
	write_out_file(rectVector);
}