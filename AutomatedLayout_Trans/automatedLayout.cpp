#include "automatedLayout.h"
#include "predefinedConstrains.h"
#include <chrono>
#include <random>

using namespace std;
using namespace cv;


float automatedLayout::cost_function() {
	float cost = 0;
	vector<float> constrain_params;
	if (!constrains->get_all_constrain_terms(constrain_params))
		return -1;
	//float mcv, mci, mpd, mpa, mcd, mca, mvb, mfa, mwa, mef, msy
	//float wcv = 1, wci = 0.01, wpd = 1.0, wpa = 1.0, wcd = 1.0, wca = 1.0, wvb = 0.7, wfa = 1.5, wwa = 1.5, wsy = 1.0, wef = 3.5;
	//float weights[] = {wcv,wci,wpd,wpa,wcd,wca,wvb,wfa,wwa,wsy,wef };
	for (int i = 0; i < 11; i++)
		cost += weights[i] * constrain_params[i];
	return cost;
}

float automatedLayout::density_function(float cost) {
	float beta = 0.85;
	return exp2f(-beta * cost);
}
void automatedLayout::random_along_wall(int furnitureID) {
	singleObj *selectedObj = &room->objects[furnitureID];
	wall* selecWall = &room->walls[rand() % room->wallNum];
	//wall* selecWall = &room->walls[3];
	srand(time(NULL));
	int rotOrNot = rand() % 2;
	float mwidth, mheight;
	if (rotOrNot) {
		mwidth = selectedObj->objHeight; mheight = selectedObj->objWidth;
	}
	else {
		mwidth = selectedObj->objWidth; mheight = selectedObj->objHeight;
	}

		
	float rotRand = (selecWall->zrotation<0)? 90- selecWall->zrotation + rotOrNot * 90 : selecWall->zrotation + rotOrNot * 90;
	room->set_obj_zrotation(rotRand* ANGLE_TO_RAD_F, furnitureID);
	//room->set_obj_zrotation(0, furnitureID);

	Vec2f p1 = selecWall->vertices[0], p2 = selecWall->vertices[1];
	float w_ran = p2[0] - p1[0] - mwidth, h_ran = p2[1] - p1[1] - mheight;
	float rh , rw;

	int mp;
	if (selecWall->position[0] > 0 || selecWall->position[1] > 0)
		mp = -1;
	else
		mp = 1;
	//todo:not bounding box...
	if (fabs(p2[0]-p1[0]) < 0.001) {
		rh = p1[1] + rand() % int(h_ran)+ selectedObj->boundingBox.height / 2;
		room->set_obj_translation(selecWall->position[0] + mp * (mwidth / 2 + 0.01), rh, furnitureID);
			//cout << "fail" << endl;
	}
	else if (fabs(p2[1]-p1[1]) < 0.001) {
		rw = p1[0] + rand() % int(w_ran)+ selectedObj->boundingBox.width / 2;
		!room->set_obj_translation(rw, selecWall->position[1] + mp * (mheight / 2 + 0.01), furnitureID);
			//cout << "fail" << endl;
	}
	else {
		//float length = (rotOrNot) ? selectedObj->objHeight : selectedObj->objWidth;
		rw = p1[0] + rand() % int(w_ran) + mwidth / 2;
		// calculate corresponding position on wall(x->y)
		float a = selecWall->a, b = selecWall->b, c = selecWall->c;
		float y = (-c - a * rw) / b;
		float tk = b / a, tb = y - tk * rw;

		float axbyc= mwidth / 2 * sqrtf(a*a + b * b);
		float tx = (axbyc - c - b * tb) / (a+b*tk);
		float ty = tk * tx + tb;
		if (ty > room->half_height || ty < -room->half_height) {
			tx = -tx;
			ty = tk * tx + tb;
		}
		room->set_obj_translation(tx, ty, furnitureID);
	}
	//todo!!!!
}
void automatedLayout::random_translation(int furnitureID, default_random_engine generator) {
	singleObj *selectedObj = &room->objects[furnitureID];

	std::normal_distribution<float> distribution_width(0, room->half_width / 5);
	std::normal_distribution<float> distribution_height(0, room->half_height / 5);

	Rect2f * boundingbox = &selectedObj->boundingBox;
	float tx, ty;
	//cout << "translate:" << endl;
	while (1) {
		tx = distribution_width(generator);
		ty = distribution_height(generator);

		while (boundingbox->x + tx <= -room->half_width || boundingbox->x + boundingbox->width + tx >= room->half_width
			|| boundingbox->y + ty >= room->half_height || boundingbox->y - boundingbox->height + ty <= -room->half_height) {
			tx = distribution_width(generator);
			ty = distribution_height(generator);
		}

		//update boundingbox, translation, vertices
		if (room->set_obj_translation(selectedObj->translation[0] + tx, selectedObj->translation[1] + ty, furnitureID))
			break;
	}
}
void automatedLayout::randomly_perturb(vector<Vec3f>& ori_trans, vector<float>& ori_rot, vector<int>& selectedid, int flag = -1) {
	if (flag == -1) {
		srand(time(NULL));
		flag = rand() % 3;
	}
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	int furnitureID = room->freeObjIds[rand() % room->freeObjIds.size()];

	ori_trans.push_back(room->objects[furnitureID].translation);
	ori_rot.push_back(room->objects[furnitureID].zrotation);
	selectedid.push_back(furnitureID);
	

	if (room->objects[furnitureID].adjoinWall)
		random_along_wall(furnitureID);

	else if (flag == 0) {
		random_translation(furnitureID, generator);
	}

	else if (flag == 1) {
		//perturb_orientation();
		std::normal_distribution<float> distribution_rot(0, CV_PI/6);

		singleObj* selectedObj = &room->objects[furnitureID];
		if(selectedObj->alignedTheWall)
			room->set_obj_zrotation(room->walls[rand()%room->wallNum].zrotation, furnitureID);
		else
			room->set_obj_zrotation(selectedObj->zrotation + distribution_rot(generator), furnitureID);
	}
	else{
		//swap_random();
		int objId2 = furnitureID;
		while(objId2 == furnitureID)
			objId2 = room->freeObjIds[rand() % room->freeObjIds.size()];
		ori_trans.push_back(room->objects[objId2].translation);
		ori_rot.push_back(room->objects[objId2].zrotation);
		selectedid.push_back(objId2);

		singleObj * obj1, *obj2;
		obj1 = &room->objects[furnitureID];
		obj2 = &room->objects[objId2];

		// store obj1 pos/rot temperately
		float ori1_rot = obj1->zrotation;
		float ori1x = obj1->translation[0];
		float ori1y = obj1->translation[1];
		if (!obj1->alignedTheWall && !obj2->alignedTheWall) {
			room->set_obj_zrotation(obj2->zrotation, furnitureID);
			room->set_obj_zrotation(ori1_rot, objId2);
		}
		if (!obj2->adjoinWall) {
			if (!room->set_obj_translation(obj2->translation[0], obj2->translation[1], furnitureID))
				random_translation(furnitureID, generator);
			if (!room->set_obj_translation(ori1x, ori1y, objId2))
				random_translation(objId2, generator);
		}

	}
	room->update_furniture_mask();
}
void automatedLayout::initial_assignment(){
	for (int i = 0; i < room->freeObjIds.size(); i++) {
		singleObj* obj = &room->objects[room->freeObjIds[i]];
		if (obj->adjoinWall)
			random_along_wall(room->freeObjIds[i]);
		else if (obj->alignedTheWall)
			room->set_obj_zrotation(room->walls[rand() % room->wallNum].zrotation, room->freeObjIds[i]);
	}
	room->update_furniture_mask();
	min_cost = cost_function();
	if (min_cost == -1)
		min_cost = INFINITY;
	res_rotation.push(room->get_objs_rotation());
	res_transform.push(room->get_objs_transformation());
	
}
void automatedLayout::Metropolis_Hastings() {
	float p0, cost, p, alpha;
	vector<Vec3f> perturb_ori_trans;
	vector<float> perturb_ori_rot;
	vector<int> perturb_id;

	if (room->freeObjIds.size() == 0)
		return;
	cost = cost_function();
	p0 = density_function(cost);

	randomly_perturb(perturb_ori_trans, perturb_ori_rot, perturb_id);
	cost = cost_function();
	while (cost == -1) {
		perturb_ori_trans.clear(); perturb_ori_rot.clear(); perturb_id.clear();
		randomly_perturb(perturb_ori_trans, perturb_ori_rot, perturb_id, 0);
		cost = cost_function();
	}



	p = density_function(cost);
	float t = ((float)rand() + 1.0) / ((float)RAND_MAX + 2.0);
	alpha = p / p0;
	if (alpha > 1)
		alpha = 1.0;

	// change back to original one
	if (alpha > t) {
		for (int i = 0; i < perturb_id.size(); i++) {
			room->set_obj_zrotation(perturb_ori_rot[i], perturb_id[i]);
			room->set_obj_translation(perturb_ori_trans[i][0], perturb_ori_trans[i][1], perturb_id[i]);
		}
		room->update_furniture_mask();
	}
	if (cost < min_cost) {
		vector<float> resRots = room->get_objs_rotation();
		vector<Vec3f> resTrans = room->get_objs_transformation();
		for (int i = 0; i < resRots.size(); i++) {
			cout << resRots[i] << " === " << resTrans[i] << endl;
		}
		res_rotation.push(resRots);
		res_transform.push(resTrans);
		if (res_rotation.size() > resNum) {
			res_rotation.pop();
			res_transform.pop();
		}
		min_cost = cost;
		cout << min_cost << endl;
	}
}

void automatedLayout::generate_suggestions() {
	initial_assignment();
	for (int i = 0; i < 1000; i++) {
		cout << "Times:" << i << endl;
		Metropolis_Hastings();
	}

}

string getPureNumFromVector(vector<Vec2f> vec) {
	string res = "";
	for (int i = 0; i < vec.size(); i++)
		res += to_string(vec[i][0]) + " "+ to_string(vec[i][1]) +" ";
	return res;
}
string getPureNumFromVector(Vec3f vec) {
	return to_string(vec[0]) + " " + to_string(vec[1]);
}

void automatedLayout::display_suggestions() {
	vector<vector<Vec3f>> trans_result;
	vector<vector<float>> rot_result;

	int resSize = res_rotation.size();
	for (int i = resSize; i > 0; i--) {
		//cout << "Rank: " << i << " recommendation: " << endl;
		trans_result.push_back(res_transform.front());
		rot_result.push_back(res_rotation.front());
		res_transform.pop();
		res_rotation.pop();
		//for (int n = 0; n < room->objctNum; n++)
		//	cout << "obj " << n << "position:	( " << trans_result[n][0] << "," << translation[n][1] << " )			Rotation: " << rotation[n] << endl;
	}
	cout << "Min Cost is :"<<min_cost << endl;

	ofstream outfile;
	outfile.open("E:/recommendation.txt", 'w');
	outfile << "RoomSize: "<<to_string(int(room->half_width*2))<<" "<<to_string(int(room->half_height*2))<<"\r\n";
	if (outfile.is_open()) {
		outfile << "WALL_Id zheight vertices zrotation\r\n";
		for (int i = 0; i < room->wallNum; i++) {
			wall * tmp = &room->walls[i];
			outfile << to_string(tmp->id) << " " <<tmp->zheight<<" "<< getPureNumFromVector (tmp->vertices)<<tmp->zrotation<<"\r\n";
		}
		for (int i = 0; i < room->objctNum; i++) {
			outfile << "FURNITURE_Id Category Height ObjWidth ObjHeight\r\n";
			singleObj *tmp = &room->objects[i];
			outfile << tmp->id << " " << tmp->catalogId  << " "<<tmp->zheight<< " "<<tmp->objWidth<< " "<<tmp->objHeight;

			outfile << "\r\n";

			for (int res = resSize; res > 0; res--)
				outfile << "Recommendation" << res << " " << getPureNumFromVector(trans_result[res - 1][i])<<" "<<rot_result[res-1][i] << "\r\n";
		}
		outfile << "Obstacle Vertices\r\n";
		string obstacleContent = "";
		for (int i = 0; i < room->obstacles.size(); i++) {

			for (int j = 0; j < 8; j++)
				obstacleContent += to_string(room->obstacles[i][j]) + " ";
			obstacleContent += "\r\n";
		}
		outfile << obstacleContent;
		outfile << "Point_Focal Position\r\n";
		for (map<int, Vec3f>::iterator it = room->focalPoint_map.begin(); it != room->focalPoint_map.end(); it++)
			outfile << it->first << " " << getPureNumFromVector(it->second) << "\r\n";
		outfile.close();
	}
	else
		exit(-1);

}
