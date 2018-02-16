#pragma once
#include <vector>
#include <map>
#include <utility>
#include "include\opencv2\core.hpp"
#include "predefinedConstrains.h"
using namespace std;
using namespace cv;
#ifndef __ROOM_H__
#define __ROOM_H__

struct singleObj
{
	int id;
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
	Mat furnitureMask;
	float width;
	float height;

	Room(float s_width=80.0f, float s_height=60.0f) {
		center = Vec3f(.0f, .0f, .0f);
		objctNum = 0;
		wallNum = 0;
		initialize_default_pairwise_map();
		width = s_width;
		height = s_height;
		setup_pairwise_map();
	}
	void set_obj_translation(float tx, float ty, int id) {
		float movex = tx - objects[id].translation[0];
		float movey = ty - objects[id].translation[1];
		objects[id].boundingBox.x += movex;
		objects[id].boundingBox.y += movey;

		objects[id].translation[0] = tx;
		objects[id].translation[1] = ty;

		for (int i = 0; i < 4; i++) {
			objects[id].vertices[i][0] += movex;
			objects[id].vertices[i][1] += movey;
		}
	}
	void set_objs_transformation(vector<Vec3f> transform) {
		for (int i = 0; i < objctNum; i++)
			objects[i].translation = transform[i];
	}
	void set_objs_rotation(vector<float> rotation) {
		for (int i = 0; i < objctNum; i++)
			objects[i].zrotation = rotation[i];
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
			a = tanf((90+rot)*ANGLE_TO_RAD_F);
			b = -1;
			c =  m_position[1] - a*m_position[0];
		}
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
	void initialize_vertices(singleObj & obj) {
		float half_width = obj.objWidth / 2;
		float half_height = obj.objHeight / 2;
		float cx = obj.translation[0];
		float cy = obj.translation[1];
		obj.vertices.push_back(Vec2f(-half_width + cx, half_height + cy));
		obj.vertices.push_back(Vec2f(half_width + cx, half_height + cy));
		obj.vertices.push_back(Vec2f(half_width + cx, -half_height + cy));
		obj.vertices.push_back(Vec2f(-half_width + cx, -half_height + cy));
		
		
	}
	void add_an_object(Vec3f position, float rot, float obj_width, float obj_height, float obj_zheight, int furnitureType, int nearestWall=-1, int groupId = 0) {
		singleObj obj;
		obj.id = objects.size();
		//obj.boundingBox = Rect(tx, ty, obj_width, obj_height);
		obj.objWidth = obj_width;
		obj.objHeight = obj_height;
		obj.translation = position;
		obj.zrotation = rot * ANGLE_TO_RAD_F;
		obj.zheight = obj_zheight;
		obj.catalogId = furnitureType;
		

		//setup bounding box and vertices
		initialize_vertices(obj);
		update_obj_boundingBox_and_vertices(obj);
		if(nearestWall == -1)
			obj.nearestWall = find_nearest_wall(obj.translation[0], obj.translation[1]);
		else
			obj.nearestWall = nearestWall;

		objGroupMap[groupId].push_back(obj.id);

		objects.push_back(obj);
		objctNum++;
	}
	void add_a_focal_point(Vec3f focalpoint, int groupId = 0) {
		focalPoint_map[groupId] = focalpoint;
	}
	Vec2i card_to_graphics_coord(float half_width, float half_height, float px, float py) {
		int gx = int(floor(half_width + px));
		int gy = int(floor(half_height - py));
		return Vec2i(gx, gy);
	}
	Point2f card_to_graphics_coord_Point(float half_width, float half_height, Vec2f vertex) {
		return Point2f(half_width + vertex[0], half_height - vertex[1]);
	}
	void update_furniture_mask() {
		furnitureMask = Mat::zeros(height, width, CV_8U);
		float half_width = width / 2;
		float half_height = height / 2;
		for (vector<singleObj>::iterator it = objects.begin(); it != objects.end(); it++) {
			if (remainder(it->zrotation, PI/2) < 0.001) {
				float min_y = it->boundingBox.y - it->boundingBox.height;
				float bound_x = std::min(it->boundingBox.x + half_width +it->boundingBox.width, width);

				for (float ty = it->boundingBox.y; ty >min_y; ty--) {
					Vec2i ps = card_to_graphics_coord(half_width, half_height, it->boundingBox.x, ty);
					for (int tx = ps[0]; tx < bound_x; tx++) {
						//Vec2i index = card_to_graphics_coord(half_width, half_height, ty, tx);
						furnitureMask.at<uchar>(ps[1], tx) = 1;
					}
				}
			}
			else {
				// find 4 line's slope
				vector<float> k(4);
				vector<float> b(4);
				
				//x = ky+b
				Vec2f p1, p2;

				for (int i = 0; i < 4; i++) {
					p1 = it->vertices[i]; p2 = it->vertices[(i + 1) % 4];
					k[i] =  (p2[0] - p1[0])/(p2[1] - p1[1]);
					b[i] = p1[0] - k[i] * p1[1];
				}
				if (it->vertices[0][1] <= it->vertices[2][1]) {
					fill_part_mask(k[0],k[1],b[0],b[1],it->boundingBox.y, it->vertices[0][1], half_width, half_height);
					fill_part_mask(k[3], k[1], b[3], b[1], it->vertices[0][1], it->vertices[2][1], half_width, half_height);
					fill_part_mask(k[3], k[2], b[3], b[2], it->vertices[2][1], it->vertices[3][1], half_width, half_height);
				}
				else {
					fill_part_mask(k[0], k[1], b[0], b[1], it->boundingBox.y, it->vertices[2][1], half_width, half_height);
					fill_part_mask(k[0], k[2], b[0], b[2], it->vertices[2][1], it->vertices[0][1], half_width, half_height);
					fill_part_mask(k[3], k[2], b[3], b[2], it->vertices[0][1], it->vertices[3][1], half_width, half_height);
				}
			}
		}
	}
	void fill_part_mask(float k1, float k2, float b1, float b2, float y_max, float y_min, float half_width, float half_height) {
		for (float ty = y_min; ty < y_max; ty++) {
			Vec2i ps = card_to_graphics_coord(half_width, half_height, k1*ty + b1, ty);
			Vec2i pe = card_to_graphics_coord(half_width, half_height, k2*ty + b2, ty);
			for (int tx = ps[0]; tx < pe[0]; tx++)
				furnitureMask.at<uchar>(ps[1], tx) = 1;
		}
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

	void setup_pairwise_map() {
		vector<pair<int, Vec2f>> chair;
		chair.push_back(pair <int, Vec2f>(0, Vec2f(1, 5)));
		chair.push_back(pair <int, Vec2f>(1, Vec2f(0, COFFEETABLE_TO_SEAT)));
		chair.push_back(pair <int, Vec2f>(3, Vec2f(0, ENDTABLE_TO_SEAT_MAX)));

		vector<pair<int, Vec2f>> nightStand;
		nightStand.push_back(pair <int, Vec2f>(5, Vec2f(1, NIGHTSTAND_TO_BED_MAX)));
		pairMap[TYPE_CHAIR] = chair;
		pairMap[TYPE_NIGHTSTAND] = nightStand;
	}
};
#endif
