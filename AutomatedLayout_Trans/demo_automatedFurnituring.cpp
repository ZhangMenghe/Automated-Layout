#include <iostream>
#include "automatedLayout.h"
#include "room.h"


using namespace cv;
using namespace std;


void parser_inputfile(const char* filename, Room * room) {
	ifstream instream(filename);
	string str;
	vector<vector<float>> parameters;
	vector<char> cateType;
	char  delims[] = " :,\t\n";
	char* context = nullptr;
	while (instream && getline(instream, str)) {
		if (!str.length())
			continue;
		char * charline = new char[300];
		int r = strcpy_s(charline, 300, str.c_str());
		char * itemCate = strtok_s(charline,delims,&context);
		vector<float>param;
		char * token = strtok_s(nullptr, delims, &context);
		while (token != nullptr) {
			param.push_back(atof(token));
			token = strtok_s(nullptr, delims, &context);
		}
		parameters.push_back(param);
		cateType.push_back(itemCate[0]);
	}
	
	instream.close();
	int itemNum = cateType.size();
	int groupid;
	for (int i = 0; i < itemNum; i++) {
		switch (cateType[i])
		{
		//add a new wall
		case 'w':
			room->add_a_wall(Vec3f(parameters[i][0], parameters[i][1], parameters[i][2]), parameters[i][3], parameters[i][4], parameters[i][5]);
			break;
		case 'f':
			groupid = parameters[i].size() == 8 ? 0 : parameters[i][8];
			room->add_an_object(Vec3f(parameters[i][0], parameters[i][1], parameters[i][2]), parameters[i][3], parameters[i][4], parameters[i][5], parameters[i][6], int(parameters[i][7]), groupid);
			break;
		case 'p':
			groupid = parameters[i].size() == 3 ? 0 : parameters[i][3];
			room->add_a_focal_point(Vec3f(parameters[i][0], parameters[i][1], parameters[i][2]), groupid);
			break;
		case 'o':
			groupid = parameters[i].size() == 8 ? 0 : parameters[i][8];
			room->add_a_fixedObject(Vec3f(parameters[i][0], parameters[i][1], parameters[i][2]), parameters[i][3], parameters[i][4], parameters[i][5], parameters[i][6], int(parameters[i][7]), groupid);
			break;
		default:
			break;
		}
	}


	//TODO:CHECK OUTBOUND
	/*room->add_a_wall(Vec3f(0, 300, 0), 90, 800, 10);
	room->add_a_wall(Vec3f(400, 0, 0), 0, 600, 10);
	room->add_an_object(Vec3f(0, 0, 0), 90, 100, 200, 10, TYPE_CHAIR);
	room->add_an_object(Vec3f(100, 0, 0), 90, 100, 200, 10, TYPE_CHAIR);
	room->add_a_focal_point(Vec3f(0, 300, 0));*/

}
//int main(int argc, char** argv) {
int main(){
	char* filename;
	/*if (argc < 2) {
		filename = new char[9];
		strcpy(filename, "input.txt");
	}
	else
		filename = argv[1];*/
	filename = new char[11];
	int r = strcpy_s(filename,11, "input.txt");
	Room* room = new Room();
	parser_inputfile(filename, room);	
	if (room != nullptr && (room->objctNum != 0 || room->wallNum != 0)) {
		automatedLayout* layout = new automatedLayout(room);
		layout->generate_suggestions();
		layout->display_suggestions();
	}

	system("pause");
	return 0;
}
