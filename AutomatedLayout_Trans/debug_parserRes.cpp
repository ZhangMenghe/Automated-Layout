#include <iostream>
#include <fstream>
#include <vector>
#include <string>
using namespace std;
int currentid = -1;
float objsx, objsy, objsz;
float dist_of_points(float x1, float y1, float x2, float y2) {
	return sqrtf(pow((x2 - x1), 2) + pow((y2- y1), 2));
}
void draw_single_stuff(int cate, vector<float>param) {
	float cx, cy, cz;
	float sx = 10, sy = 10, sz = 30;
	float rot=0;
	switch (cate)
	{
	case 0:
		cx = (param[2] + param[4]) / 2;
		cy = (param[3] + param[5]) / 2;
		rot = param[6];
		sx = dist_of_points(param[2], param[3], param[4], param[5]);
		sz = 100;
		break;
	case 1:
		sx = objsx; sy = objsy; sz = objsz;
		cx = param[1]; cy = param[2];
		rot = param[3];
		break;
	case 2:
		cx = param[1];
		cy = param[2];
		sx = sy = sz = 50;
		break;
	case 3:
		cx = (param[0] + param[4]) / 2;
		cy = (param[1] + param[5]) / 2;
		rot = atanf((param[7]-param[3])/(param[6]-param[2]))*180/3.14;
		sx = dist_of_points(param[0], param[1], param[2], param[3]);
		sy = dist_of_points(param[2], param[3], param[4], param[5]);
		break;
	case 4://indicate it's an obj
		currentid = int(param[0]);
		objsz = param[2];
		objsx = param[3];
		objsy = param[4];
		return;
	default:
		break;
	}
	sx /= 10; sy /= 10; sz /= 10;
	cx /= 10; cy /= 10; cz = sz*50;
	
}
void parser_resfile(const char* filename) {
	ifstream instream(filename);
	string str;
	vector<vector<float>> parameters;
	char  delims[] = " :,\t\n|[]";
	char* context = nullptr;
	int state = -1;
	vector<float> currentObj;
	while (instream && getline(instream, str)) {
		if (!str.length())
			continue;
		char * charline = new char[300];
		int r = strcpy_s(charline, 300, str.c_str());
		vector<float>param;
		char * token = strtok_s(charline, delims, &context);
		switch (token[0])
		{
		case 'W'://wall
			state = 0;
			break;
		case 'F'://furniture
			state = 4;
			break;
		case 'P'://focal point
			state = 2;
			break;
		case 'O'://obstacle
			state = 3;
			break;
		case 'R'://furniture pos recommendation
			state = 1;
		default:
			while (token != nullptr) {
				param.push_back(atof(token));
				token = strtok_s(nullptr, delims, &context);
			}
			draw_single_stuff(state, param);
			break;
		}
	}
	instream.close();
}
int main() {
	parser_resfile("E:/recommendation.txt");
	system("pause");
	return 0;
}