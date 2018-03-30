#include "automatedLayout.h"
#include "predefinedConstrains.h"
#include <chrono>
#include <random>

using namespace std;
using namespace cv;


float automatedLayout::cost_function() {
	float cost = 0;
	vector<float> constrain_params = constrains->get_all_constrain_terms();
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
	//wall* selecWall = &room->walls[0];
	srand(time(NULL));
	int rotOrNot = rand() % 2;
	float rotRand = rotOrNot * 90 ;
	room->set_obj_zrotation((selecWall->zrotation + rotRand)* ANGLE_TO_RAD_F, furnitureID);

	Vec2f p1 = selecWall->vertices[0], p2 = selecWall->vertices[1];
	float w_ran = p2[0] - p1[0] - selectedObj->boundingBox.width, h_ran = p2[1] - p1[1] - selectedObj->boundingBox.height;
	float rh = p1[1] + rand() % int(h_ran), rw = p1[0]+rand()%int(w_ran);

	int mp;
	if (selecWall->position[0] > 0 || selecWall->position[1] > 0)
		mp = -1;
	else
		mp = 1;
	
	if (fabs(p2[0]-p1[0]) < 0.001) {
		rh += selectedObj->boundingBox.height / 2;
		if (!room->set_obj_translation(selecWall->position[0] + mp * (selectedObj->boundingBox.width / 2 + 0.01), rh, furnitureID))
			cout << "fail" << endl;
	}
		
	else if (fabs(p2[1]-p1[1]) < 0.001) {
		rw += selectedObj->boundingBox.width / 2;
		if(!room->set_obj_translation(rw, selecWall->position[1] + mp * (selectedObj->boundingBox.height / 2+0.01), furnitureID))
			cout << "fail" << endl;
	}
		
	else {
	
		room->set_obj_translation(rw, rh, furnitureID);
	}
		
	//todo!!!!
}
void automatedLayout::random_translation(int furnitureID, default_random_engine generator) {
	singleObj *selectedObj = &room->objects[furnitureID];

	std::normal_distribution<float> distribution_width(0, room->half_width / 3);
	std::normal_distribution<float> distribution_height(0, room->half_height / 3);

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
void automatedLayout::randomly_perturb(vector<Vec3f>& ori_trans, vector<float>& ori_rot, vector<int>& selectedid) {
	srand(time(NULL));
	int flag = rand() %3;
	//int flag = 1;
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
		std::normal_distribution<float> distribution_rot(0, PI/6);
		
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
}
void automatedLayout::initial_assignment(){
	for (int i = 0; i < room->freeObjIds.size(); i++) {
		singleObj* obj = &room->objects[room->freeObjIds[i]];
		if (obj->adjoinWall)
			random_along_wall(room->freeObjIds[i]);
		else if(obj->alignedTheWall)
			room->set_obj_zrotation(room->walls[rand() % room->wallNum].zrotation, room->freeObjIds[i]);
	}
	res_rotation.push(room->get_objs_rotation());
	res_transform.push(room->get_objs_transformation());
	min_cost = cost_function();
	room->update_furniture_mask();
}
void automatedLayout::Metropolis_Hastings() {
	float p0, cost, p, alpha;
	vector<Vec3f> perturb_ori_trans;
	vector<float> perturb_ori_rot;
	vector<int> perturb_id;

	cost = cost_function();
	p0 = density_function(cost);
	
	randomly_perturb(perturb_ori_trans, perturb_ori_rot, perturb_id);

	cost = cost_function();
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
	}
	if (cost <= min_cost) {
		res_rotation.push(room->get_objs_rotation());
		res_transform.push(room->get_objs_transformation());
		if (res_rotation.size() > resNum) {
			res_rotation.pop();
			res_transform.pop();
		}
		min_cost = cost;
		room->update_furniture_mask();
	}
}

void automatedLayout::generate_suggestions() {
	initial_assignment();
	for (int i = 0; i < 1000; i++) {
		cout << "Times:" << i << endl;
		Metropolis_Hastings();
	}
				
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
		outfile << "WALL_Id\t|\tzheight\t|\tvertices\r\n";
		for (int i = 0; i < room->wallNum; i++) {
			wall * tmp = &room->walls[i];
			outfile << to_string(tmp->id) << "\t|\t" << to_string(tmp->zheight) << "\t|\t" <<tmp->vertices[0] << "\t|\t" << tmp->vertices[1]<<"\r\n";
		}

		outfile << "OBJ_Id\t|\tCategory\t|\tHeight\t|\tObjWidth\t|\tObjHeight\r\n";
		for (int i = 0; i < room->objctNum; i++) {
			singleObj *tmp = &room->objects[i];
			outfile << tmp->id << "\t|\t" << tmp->catalogId  << "\t|\t"<<tmp->zheight<< "\t|\t"<<tmp->objWidth<< "\t|\t"<<tmp->objHeight;
			
			outfile << "\r\n";
			
			for (int res = resSize; res > 0; res--)
				outfile << "Recommendation" << res << "\t|\t" << trans_result[res - 1][i] << "\t|\t" << rot_result[res - 1][i] << "\r\n";
		}
		outfile << "Obstacle\t|\tVertices\r\n";
		string obstacleContent = "";
		for (int i = 0; i < room->obstacles.size(); i++) {
			
			for (int j = 0; j < 8; j++)
				obstacleContent += to_string(room->obstacles[i][j]) + "\t|\t";
			obstacleContent += "\r\n";
		}
		outfile << obstacleContent;
		outfile << "FocalPoint\t|\tPosition\r\n";
		for (map<int, Vec3f>::iterator it = room->focalPoint_map.begin(); it != room->focalPoint_map.end(); it++)
			outfile << it->first << "\t|\t" << it->second << "\r\n";
		outfile.close();
	}
	else
		exit(-1);

}