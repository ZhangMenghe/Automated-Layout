#include "automatedLayout.h"
#include "predefinedConstrains.h"


using namespace std;
using namespace cv;


float automatedLayout::cost_function() {
	float cost = 0;
	vector<float> constrain_params = constrains->get_all_constrain_terms();
	//float mcv, mci, mpd, mpa, mcd, mca, mvb, mfa, mwa, mef, msy
	float wcv = 1, wci = 0.01, wpd = 1.0, wpa = 1.0, wcd = 1.0, wca = 1.0, wvb = 0.7, wfa = 1.5, wwa = 1.5, wsy = 1.0, wef = 0.5;
	float weights[] = {wcv,wci,wpd,wpa,wcd,wca,wvb,wfa,wwa,wsy,wef };
	for (int i = 0; i < 11; i++)
		cost += weights[i] * constrain_params[i];
	return cost;
}

float automatedLayout::density_function(float cost) {
	float beta = 0.85;
	return exp2f(-beta * cost);
}

void automatedLayout::randomly_perturb(vector<Vec3f>& ori_trans, vector<float>& ori_rot, vector<int>& selectedid) {
	int flag = rand() % 3;
	//int flag = 1;
	std::default_random_engine generator;

	float half_width = room->width / 2;
	float half_height = room->height / 2;

	if (flag == 0) {
		//perturb_position();
		std::normal_distribution<float> distribution_width(0, room->width / 6);
		std::normal_distribution<float> distribution_height(0, room->height / 6);
		int furnitureID = rand() % room->objctNum;
		singleObj *selectedObj = &room->objects[furnitureID];
		Rect2f * boundingbox = &selectedObj->boundingBox;
		float tx, ty;
		while (1) {
			tx = distribution_width(generator);
			ty = distribution_height(generator);
			
			while (boundingbox->x + tx < -half_width || boundingbox->x + boundingbox->width+tx > half_width 
				|| boundingbox->y + ty > half_height || boundingbox->y - boundingbox->height + ty < -half_height) {
				tx = distribution_width(generator);
				ty = distribution_height(generator);
			}

			ori_trans.push_back(selectedObj->translation);
			ori_rot.push_back(selectedObj->zrotation);
			selectedid.push_back(furnitureID);

			//update boundingbox, translation, vertices
			room->set_obj_translation(selectedObj->translation[0]+tx, selectedObj->translation[1] + ty,furnitureID);
			int i = 0;
			for (; i < room->objctNum; i++) {
				if (i != furnitureID) {
					if (constrains->cal_overlapping_area(room->objects[furnitureID].boundingBox, room->objects[i].boundingBox) != 0)
						break;
				}
			}
			if (i == room->objctNum)
				return;
		}
	}
	else if (flag == 1) {
		//perturb_orientation();
		std::normal_distribution<float> distribution_rot(0, PI/3);
		int furnitureID = rand() % room->objctNum;
		singleObj* selectedObj = &room->objects[furnitureID];
		ori_trans.push_back(selectedObj->translation);
		ori_rot.push_back(selectedObj->zrotation);
		selectedid.push_back(furnitureID);
		selectedObj->zrotation = fmod(selectedObj->zrotation + distribution_rot(generator), PI);
		cout << selectedObj->zrotation << endl;
		room->update_obj_boundingBox_and_vertices(room->objects[furnitureID]);
	}
	else if (flag == 2) {
		//swap_random();
		int objId1 = rand() % room->objctNum;
		int objId2 = objId1;
		while(objId2 == objId1)
			objId2 = rand() % room->objctNum;

		ori_trans.push_back(room->objects[objId1].translation);
		ori_rot.push_back(room->objects[objId1].zrotation);
		selectedid.push_back(objId1);
		ori_trans.push_back(room->objects[objId2].translation);
		ori_rot.push_back(room->objects[objId2].zrotation);
		selectedid.push_back(objId2);

		swap(room->objects[objId1].translation, room->objects[objId2].translation);
		swap(room->objects[objId1].zrotation, room->objects[objId2].zrotation);
		float tmpx, tmpy;
		tmpx = room->objects[objId1].boundingBox.x;
		tmpy = room->objects[objId1].boundingBox.y;

		room->objects[objId1].boundingBox.x = room->objects[objId2].boundingBox.x;
		room->objects[objId1].boundingBox.y = room->objects[objId2].boundingBox.y;

		room->objects[objId2].boundingBox.x = tmpx;
		room->objects[objId2].boundingBox.y = tmpy;
	}
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
			room->set_obj_translation(perturb_ori_trans[i][0], perturb_ori_trans[i][1], perturb_id[i]);
			room->objects[perturb_id[i]].zrotation = perturb_ori_rot[i];
		}
	}
	/*else {
		cout << "move" << endl;
	}*/
	//cout << cost<<endl;
	if (cost < min_cost) {
		res_rotation.push(room->get_objs_rotation());
		res_transform.push(room->get_objs_transformation());
		if (res_rotation.size() > resNum) {
			res_rotation.pop();
			res_transform.pop();
		}
		min_cost = cost;
	}
}

void automatedLayout::generate_suggestions() {
	for (int i = 0; i < 1000; i++)
		Metropolis_Hastings();		
}

void automatedLayout::setup_default_furniture() {
	room = new Room();
	//TODO:CHECK OUTBOUND
	room->add_a_wall(Vec3f(40,0,0), 0, 60, 10);
	room->add_an_object(Vec3f(0, 0, 0), 90, 10, 10, 10, TYPE_CHAIR);
	room->add_a_focal_point(Vec3f(0, 30, 0));
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
	outfile.open("recommendation.txt", 'w');
	if (outfile.is_open()) {
		outfile << "WALL_Id\t|\tzheight\t|\tvertices\r\n";
		for (int i = 0; i < room->wallNum; i++) {
			wall * tmp = &room->walls[i];
			outfile << to_string(tmp->id) << "\t|\t" << to_string(tmp->zheight) << "\t|\t" <<tmp->vertices[0] << "\t|\t" << tmp->vertices[1]<<"\r\n";
		}
		outfile << "OBJ_Id\t|\tCategory\t|\tBoundingBox\t|\tHeight\t|\tVertices\r\n";
		for (int i = 0; i < room->objctNum; i++) {
			singleObj * tmp = &room->objects[i];
			outfile << tmp->id << "\t|\t" << tmp->catalogId << "\t|\t" << tmp->boundingBox << "\t|\t" << tmp->zheight;
			for (int i = 0; i < 4; i++)
				outfile << "\t|\t" << tmp->vertices[i];
			outfile << "\r\n";
			for (int res = resSize; res > 0; res--)
				outfile << "Recommendation"<< res <<"\t|\t" << trans_result[res-1][i] << "\t|\t" <<rot_result[res-1][i]<<"\r\n";
			
		}
			
		outfile.close();
	}
	else
		exit(-1);

}