#pragma once
#include <vector>
#include <map>
#include <utility>
#include "include/opencv2/core.hpp"
#include "predefinedConstrains.h"

//#include "utils.h"
using namespace std;
using namespace cv;
#ifndef __ROOM_H__
#define __ROOM_H__

struct singleObj
{
	int id;
	bool isFixed;
	Rect2f boundingBox;
	vector<Vec2f> vertices;
	Vec3f translation;
	float zrotation;
	float objWidth, objHeight;
	float zheight;
	int nearestWall;
	int catalogId;
	float area;
};

struct wall
{
	int id;
	Vec3f position;
	float zrotation;
	float width;
	float a, b, c;//represent as ax+by+c=0
	float zheight;
	vector<Vec2f> vertices;
};

class Room {
private :
	Mat_<uchar> furnitureMask_initial;

	void initialize_vertices_wall(wall & nw) {
		float half_length = nw.width / 2;
		if (nw.a == 0) {
			nw.vertices.push_back(Vec2f(nw.position[0] - half_length, nw.position[1]));
			nw.vertices.push_back(Vec2f(nw.position[0] + half_length, nw.position[1]));
		}
		else if (nw.b == 0) {
			nw.vertices.push_back(Vec2f(nw.position[0], nw.position[1] - half_length));
			nw.vertices.push_back(Vec2f(nw.position[0], nw.position[1] + half_length));
		}
		else {
			float half_len_proj_x = cosf((90 + nw.zrotation)*ANGLE_TO_RAD_F) *half_length;
			float half_len_proj_y = sinf((90 + nw.zrotation)*ANGLE_TO_RAD_F) *half_length;
			nw.vertices.push_back(Vec2f(nw.position[0] + half_len_proj_x, nw.position[1] + half_len_proj_y));
			nw.vertices.push_back(Vec2f(nw.position[0] - half_len_proj_x, nw.position[1] - half_len_proj_y));
		}

	}
	void setup_wall_equation(Vec3f m_position, float rot, float & a, float & b, float & c) {
		a = b = c = .0f;
		//x+c=0
		if (remainder(rot, 180) == 0) {
			a = 1;
			c = -m_position[0];
		}
		else if (remainder(rot, 90) == 0) {
			b = 1;
			c = -m_position[1];
		}
		else {
			rot = remainder(rot, 180);
			a = tanf((90 + rot)*ANGLE_TO_RAD_F);
			b = -1;
			c = m_position[1] - a * m_position[0];
		}
	}
	// 4*2 vertices, 2 center, 2 size, angle, label, zheight
	void initial_object_by_parameters(vector<float>params, bool isFixed = false) {
		singleObj obj;
		obj.id = objects.size();
		//vertices
		for (int i = 0; i < 4; i++)
			obj.vertices.push_back(Vec2f(params[2 * i], params[2 * i + 1]));

		obj.translation = Vec3f(params[8], params[9], .0f);
		obj.objWidth = params[10];
		obj.objHeight = params[11];

		obj.zrotation = params[12] * ANGLE_TO_RAD_F;
		obj.catalogId = params[13];
		obj.zheight = params[14];
		obj.area = obj.objWidth * obj.objHeight;
		indepenFurArea += obj.area;
		update_obj_boundingBox_and_vertices(obj);

		//obj.nearestWall = find_nearest_wall(obj.translation[0], obj.translation[1]);

		objGroupMap[0].push_back(obj.id);

		objects.push_back(obj);
		objctNum++;
		if (!isFixed) {
			update_mask_by_object(&obj, furnitureMask);
			freeObjIds.push_back(obj.id);
		}
		else
			update_mask_by_object(&obj, furnitureMask_initial);//is a fixed object
		float test = cv::sum(furnitureMask)[0];
	}
	void update_mask_by_object(const singleObj* obj, Mat_<uchar> & target, float movex = -1, float movey=-1) {
		vector<Point> contour;
		vector<vector<Point>> contours;
		for (int i = 0; i < 4; i++)
			contour.push_back(card_to_graph_point(obj->vertices[i][0], obj->vertices[i][1]));
		contours.push_back(contour);
		if (movex != -1) {
			drawContours(target, contours, 0, 0, FILLED, 8);
			vector<Point> contour2;
			for (int i = 0; i < 4; i++)
				contour2.push_back(card_to_graph_point(movex + obj->vertices[i][0], movey + obj->vertices[i][1]));
			contours.push_back(contour2);
			drawContours(target, contours, 1, 1, FILLED, 8);
		}
		else
			drawContours(target, contours,-1, 1, FILLED, 8);
	}

public:
	//Rect2f boundingBox;
	Vec3f center;
	vector<singleObj> objects;
	vector<wall> walls;
	map<int, vector<int>> objGroupMap;
	map<int, vector<pair<int, Vec2f>>> pairMap;
	map<int, Vec3f> focalPoint_map;
	int objctNum;
	int wallNum;
	Mat_<uchar> furnitureMask;
	float half_width;
	float half_height;
	float indepenFurArea;
	float obstacleArea;
	vector<int> freeObjIds;
	vector<vector<float>> obstacles;
	Room(float s_width=800.0f, float s_height=600.0f) {
		center = Vec3f(.0f, .0f, .0f);
		objctNum = 0;
		wallNum = 0;
		half_width = s_width/2;
		half_height = s_height/2;
		indepenFurArea = 0;
		obstacleArea = 0;
		set_pairwise_map();
		furnitureMask_initial = Mat::zeros(int(s_width +1), int(s_height +1), CV_8UC1);
		furnitureMask = furnitureMask_initial.clone();
	}
	Point card_to_graph_point(float x, float y) {
		return Point(int(half_width + x), int(half_height - y));
	}
	void rot_around_point(const Vec3f& center, Vec2f& pos, float s, float c) {
		// translate point back to origin:
		pos[0] -= center[0];
		pos[1] -= center[1];

		// rotate point
		float xnew = pos[0] * c - pos[1] * s;
		float ynew = pos[0] * s + pos[1] * c;

		// translate point back:
		pos[0] = xnew + center[0];
		pos[1] = ynew + center[1];
	}
	void set_obj_zrotation(float new_rotation, int id) {
		objects[id].zrotation = fmod(new_rotation, PI);
		update_obj_boundingBox_and_vertices(objects[id]);
	}

	bool set_obj_translation(float tx, float ty, int id) {
		Mat_<uchar> tmpCanvas = furnitureMask;
		singleObj * obj = &objects[id];

		float movex = tx - objects[id].translation[0];
		float movey = ty - objects[id].translation[1];
		if (obj->boundingBox.x + movex <= -half_width || obj->boundingBox.x + obj->boundingBox.width + movex >= half_width
			|| obj->boundingBox.y + movey >= half_height || obj->boundingBox.y - obj->boundingBox.height + movey <= -half_height) {
			return false;
		}
		update_mask_by_object(obj, tmpCanvas, movex, movey);

		if (cv::sum(furnitureMask)[0] + obj->area < cv::sum(tmpCanvas)[0])
			return false;

		obj->translation[0] = tx;
		obj->translation[1] = ty;
		for (int i = 0; i < 4; i++) {
			obj->vertices[i][0] += movex;
			obj->vertices[i][1] += movey;
		}
		return true;
	}

	void set_objs_rotation(vector<float> rotation) {
		for (int i = 0; i < objctNum; i++)
			objects[i].zrotation = rotation[i];
	}

	void set_pairwise_map() {
		vector<pair<int, Vec2f>> chair;
		// seat to seat
		chair.push_back(pair <int, Vec2f>(0, Vec2f(0, 50)));
		//coffee table to seat
		chair.push_back(pair <int, Vec2f>(1, Vec2f(40, 46)));
		//seat to end table
		chair.push_back(pair <int, Vec2f>(3, Vec2f(0, 30)));

		vector<pair<int, Vec2f>> bed;
		// bed TO nightstand
		bed.push_back(pair <int, Vec2f>(5, Vec2f(0, 30)));
		// bed to wall
		bed.push_back(pair <int, Vec2f>(100, Vec2f(0, 0)));

		vector<pair<int, Vec2f>> shelf;
		shelf.push_back(pair<int, Vec2f>(100, Vec2f(0, 0)));

		pairMap[TYPE_CHAIR] = chair;
		pairMap[TYPE_BED] = bed;
		pairMap[TYPE_SHELF] = shelf;
	}

	vector<Vec3f> get_objs_transformation() {
		vector<Vec3f> res;
		for (int i = 0; i < objctNum; i++)
			res.push_back( objects[i].translation );
		return res;
	}

	vector<float> get_objs_rotation() {
		vector<float> res;
		for (int i = 0; i < objctNum; i++)
			res.push_back(objects[i].zrotation);
		return res;
	}
	float get_nearest_wall_dist(singleObj * obj) {
		float x = obj->translation[0], y = obj->translation[1];
		float min_dist = INFINITY, dist;
		//int min_id = -1;
		for (int i = 0; i < wallNum; i++) {
			dist = abs(walls[i].a * x + walls[i].b * y + walls[i].c) / sqrt(walls[i].a * walls[i].a + walls[i].b * walls[i].b);
			if (dist < min_dist) {
				min_dist = dist;
				obj->nearestWall = i;
			}
		}
		return min_dist;
	}
	void add_a_focal_point(vector<float> fp) {
		if(fp.size()>3)
			focalPoint_map[fp[3]] = Vec3f(fp[0], fp[1], fp[2]);
		else
			focalPoint_map[0] = Vec3f(fp[0], fp[1], fp[2]);
	}
	// params: Vec3f m_position, float rot, float w_width, float w_height) {
	void add_a_wall(vector<float> params){
		wall newWall;
		newWall.id = walls.size();
		newWall.position = Vec3f(params[0], params[1], params[2]);
		newWall.zrotation = params[3] * ANGLE_TO_RAD_F;
		newWall.width = params[4];
		newWall.zheight = params[5];
		setup_wall_equation(newWall.position, params[3], newWall.a, newWall.b, newWall.c);
		initialize_vertices_wall(newWall);
		walls.push_back(newWall);
		wallNum++;
	}
	void add_an_object(vector<float> params, bool isFixed = false) {
		if (params.size() < 15) {
			float hw = params[2] / 2, hh = params[3] / 2;
			float cx = params[0], cy = params[1];
			float res[8] = { -hw + cx, hh + cy, hw + cx, hh + cy, hw + cx, -hh + cy, -hw + cx, -hh + cy };
			vector<float>vertices(res, res + 8);// get_vertices_by_pos(params[0], params[1], params[2] / 2, params[3] / 2);
			params.insert(params.begin(), vertices.begin(), vertices.end());
		}
		if(params.size()<16)
			params.push_back(0);
		initial_object_by_parameters(params, isFixed);
	}

	void add_an_obstacle(vector<float> vertices) {
		vector<Point> contour;
		vector<vector<Point>> contours;

		for (int i = 0; i < 4; i++)
			contour.push_back(card_to_graph_point(vertices[2 * i], vertices[2 * i + 1]));
		contours.push_back(contour);
		drawContours(furnitureMask_initial, contours, -1, 1, FILLED, 8);
		obstacleArea = cv::sum(furnitureMask_initial)[0];
		cout << "obstacleArea:  " << obstacleArea<<endl;
		obstacles.push_back(vertices);
	}
			
	void update_obj_boundingBox_and_vertices(singleObj& obj) {
		float s = sin(obj.zrotation);
		float c = cos(obj.zrotation);
		for (int i = 0; i < 4; i++)
			rot_around_point(obj.translation, obj.vertices[i],s,c);
		vector<float> xbox = { obj.vertices[0][0], obj.vertices[1][0], obj.vertices[2][0], obj.vertices[3][0]};
		vector<float> ybox = { obj.vertices[0][1], obj.vertices[1][1], obj.vertices[2][1], obj.vertices[3][1] };
		
		vector<float>::iterator it = min_element(xbox.begin(), xbox.end());
		//make sure bounding box start vertex is at the begining
		int startIdx = distance(xbox.begin(), it);
		vector<Vec2f> sub(obj.vertices.begin(), obj.vertices.begin() + startIdx);
		obj.vertices.insert(obj.vertices.end(), sub.begin(), sub.end());
		obj.vertices.erase(obj.vertices.begin(), obj.vertices.begin() + startIdx);

		float min_x = *it;
		float min_y = *min_element(ybox.begin(), ybox.end());
		float max_x = *max_element(xbox.begin(), xbox.end());
		float max_y = *max_element(ybox.begin(), ybox.end());

		obj.boundingBox = Rect2f(min_x, max_y, (max_x - min_x), (max_y - min_y));

		float offsetx = .0f, offsety = .0f;

		if (obj.boundingBox.x < -half_width)
			offsetx = -half_width - obj.boundingBox.x+1;
		else if (obj.boundingBox.x + obj.boundingBox.width > half_width)
			offsetx = half_width+1- obj.boundingBox.x - obj.boundingBox.width;
		if (obj.boundingBox.y > half_height)
			offsety = half_height- obj.boundingBox.y-1;
		else if (obj.boundingBox.y - obj.boundingBox.height < -half_height)
			offsety = -half_height - obj.boundingBox.y + obj.boundingBox.height+1;
		if (offsetx != 0 || offsety != 0) {
			obj.translation += Vec3f(offsetx, offsety, .0f);
			for (int i = 0; i < 4; i++) {
				obj.vertices[i] += Vec2f(offsetx, offsety);
			}
			obj.boundingBox.x += offsetx;
			obj.boundingBox.y += offsety;
		}
	}

	void update_furniture_mask() {
		furnitureMask = furnitureMask_initial.clone();
		vector<vector<Point>> contours;
		for (int i = 0; i < objctNum; i++) {
			singleObj * obj = &objects[i];
			vector<Point> contour;
			for (int n = 0; n < 4; n++)
				contour.push_back(card_to_graph_point(obj->vertices[n][0], obj->vertices[n][1]));
			contours.push_back(contour);
		}
		drawContours(furnitureMask, contours, -1, 1, FILLED, 8);
	}
	void change_obj_freeState(singleObj* obj) {
		if (obj->isFixed)
			freeObjIds.erase(remove(freeObjIds.begin(), freeObjIds.end(), obj->id));
		else
			freeObjIds.push_back(obj->id);
		obj->isFixed = !obj->isFixed;
	}
};
#endif
