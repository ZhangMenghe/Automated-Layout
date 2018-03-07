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

	int find_nearest_wall(float x, float y) {
		float min_dist = INFINITY, dist;
		int min_id = -1;
		for (int i = 0; i < wallNum; i++) {
			dist = abs(walls[i].a * x + walls[i].b * y + walls[i].c) / sqrt(walls[i].a * walls[i].a + walls[i].b * walls[i].b);
			if (dist < min_dist) {
				min_dist = dist;
				min_id = i;
			}
		}
		return min_id;
	}
	void initialize_default_pairwise_map() {

	}
	float* get_vertices_by_pos(float cx, float cy, float half_width, float half_height) {
		float res[8] =  { -half_width + cx, half_height + cy, half_width + cx, half_height + cy, half_width + cx, -half_height + cy, -half_width + cx, -half_height + cy };
		return res;
	}
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


		update_obj_boundingBox_and_vertices(obj);

		obj.nearestWall = find_nearest_wall(obj.translation[0], obj.translation[1]);

		objGroupMap[0].push_back(obj.id);

		objects.push_back(obj);
		objctNum++;
		if (!isFixed)
			freeObjIds.push_back(obj.id);
		else
			update_mask_by_object(&obj, true);
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
	void update_mask_by_object(const singleObj* obj, bool changeInitial = false) {
		vector<Point> contour;
		vector<vector<Point>> contours;

		for (int i = 0; i < 4; i++)
			contour.push_back(Point(int(obj->vertices[i][0]), int(obj->vertices[i][1])));
		contours.push_back(contour);
		if(changeInitial)
			drawContours(furnitureMask_initial, contours, -1, 255, FILLED, 8);
		else
			drawContours(furnitureMask, contours, -1, 255, FILLED, 8);
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
	float width;
	float height;
	vector<int> freeObjIds;
	vector<vector<float>> obstacles;
	Room(float s_width=800.0f, float s_height=600.0f) {
		center = Vec3f(.0f, .0f, .0f);
		objctNum = 0;
		wallNum = 0;
		initialize_default_pairwise_map();
		width = s_width;
		height = s_height;
		set_pairwise_map();
		furnitureMask_initial = Mat::zeros(int(height+1), int(width+1), CV_8UC1);
		furnitureMask = furnitureMask_initial;
	}
	Point2f card_to_graphics_coord_Point(float half_width, float half_height, Vec2f vertex) {
		return Point2f(half_width + vertex[0], half_height - vertex[1]);
	}
	Vec2i card_to_graphics_coord(float half_width, float half_height, float px, float py) {

		int gx = int(floor(half_width + px));
		int gy = int(floor(half_height - py));
		//cout << gy<<"---"<<py<<endl;
		return Vec2i(gx, gy);
	}
	float cal_overlapping_area(const Rect r1, const Rect r2) {
		if (r1.x > r2.x + r2.width || r1.x + r1.width < r2.x)
			return 0;
		if (r1.y > r2.y + r2.height || r1.y + r1.height < r2.y)
			return 0;
		float dx = std::min(r1.x + r1.width, r2.x + r2.width) - std::max(r1.x, r2.x);
		float dy = std::min(r1.y, r2.y) - std::max(r1.y - r1.height, r2.y - r2.height);
		if ((dx >= 0) && (dy >= 0))
			return dx*dy;
		return 0;
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
		float ori_boundx = objects[id].boundingBox.x;
		float ori_boundy = objects[id].boundingBox.y;

		float movex = tx - objects[id].translation[0];
		float movey = ty - objects[id].translation[1];
		objects[id].boundingBox.x += movex;
		objects[id].boundingBox.y += movey;
		

		for (int i = 0; i < objctNum; i++) {
			if (i != id) {
				float overlapping_ratio = cal_overlapping_area(objects[i].boundingBox, objects[id].boundingBox) / objects[id].boundingBox.area();
				if (overlapping_ratio != 0){
					objects[id].boundingBox.x = ori_boundx;
					objects[id].boundingBox.y = ori_boundy;
					return false;
				}	
			}
		}
		objects[id].translation[0] = tx;
		objects[id].translation[1] = ty;

		for (int i = 0; i < 4; i++) {
			objects[id].vertices[i][0] += movex;
			objects[id].vertices[i][1] += movey;
		}
		return true;
	}
	void set_objs_rotation(vector<float> rotation) {
		for (int i = 0; i < objctNum; i++)
			objects[i].zrotation = rotation[i];
	}
	void set_pairwise_map() {
		vector<pair<int, Vec2f>> chair;
		chair.push_back(pair <int, Vec2f>(0, Vec2f(1, 5)));
		chair.push_back(pair <int, Vec2f>(1, Vec2f(0, COFFEETABLE_TO_SEAT)));
		chair.push_back(pair <int, Vec2f>(3, Vec2f(0, ENDTABLE_TO_SEAT_MAX)));

		vector<pair<int, Vec2f>> nightStand;
		nightStand.push_back(pair <int, Vec2f>(5, Vec2f(1, NIGHTSTAND_TO_BED_MAX)));
		pairMap[TYPE_CHAIR] = chair;
		pairMap[TYPE_NIGHTSTAND] = nightStand;
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

	void add_a_focal_point(Vec3f focalpoint, int groupId = 0) {
		focalPoint_map[groupId] = focalpoint;
	}
	void add_a_wall(Vec3f m_position, float rot, float w_width, float w_height) {
		wall newWall;
		newWall.id = walls.size();
		newWall.position = m_position;
		newWall.zrotation = rot * ANGLE_TO_RAD_F;
		newWall.width = w_width;
		newWall.zheight = w_height;
		setup_wall_equation(m_position, rot, newWall.a, newWall.b, newWall.c);
		initialize_vertices_wall(newWall);
		walls.push_back(newWall);
		wallNum++;

	}

	void add_an_object(vector<float> params, int groupId = 0, bool isFixed = false) {
		float* vertices;
		if (params.size() < 15) {
			vertices = get_vertices_by_pos(params[0], params[1], params[2], params[3]);
			params.insert(params.begin(), vertices, vertices + 8);
		}
		params.push_back(groupId);
		initial_object_by_parameters(params, isFixed);
	}

	void add_an_obstacle(vector<float> vertices) {
		vector<Point> contour;
		vector<vector<Point>> contours;

		for (int i = 0; i < 4; i++)
			contour.push_back(Point(int(vertices[2*i]), int(vertices[2 * i + 1])));
		contours.push_back(contour);
		drawContours(furnitureMask_initial, contours, -1, 255, FILLED, 8);
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
		float half_width = width / 2, half_height = height / 2;

		if (obj.boundingBox.x < -half_width)
			offsetx = -half_width - obj.boundingBox.x+1;
		else if (obj.boundingBox.x + obj.boundingBox.width > half_width)
			offsetx = obj.boundingBox.x + obj.boundingBox.width - half_width-1;
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
		furnitureMask = furnitureMask_initial;	
		vector<vector<Point>> contours;
		for (int i = 0; i < objctNum; i++) {
			singleObj * obj = &objects[i];
			vector<Point> contour;
			for (int n = 0; n < 4; n++)
				contour.push_back(Point(int(obj->vertices[i][0]), int(obj->vertices[i][1])));
			contours.push_back(contour);
		}
		drawContours(furnitureMask, contours, -1, 255, FILLED, 8);
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
