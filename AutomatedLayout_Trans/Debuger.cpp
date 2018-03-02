#include <iostream>
#include <fstream>
#include "include\opencv2\core.hpp"
using namespace cv;
using namespace std;

//Mat heightMap;
//const char * heightMap_filepath = "E:/heightMaptxt.txt";
/*void parser_heightDataFile() {
	ifstream instream(heightMap_filepath);
	string str;
	vector<vector<float>> contents;
	char  delims[] = " :[],\t\n";
	char * context = nullptr;
	while (instream && getline(instream, str)) {
		if (!str.length())
			continue;
		int len = 2*str.length();
		char * charline = new char[len];
		int r = strcpy_s(charline, len, str.c_str());
		vector<float> row;
		char * token = strtok_s(charline, delims, &context);
		while (token != nullptr) {
			if (token[0] == 'i')
				row.push_back(.0f);
			else
				row.push_back(atof(token));
			token = strtok_s(nullptr, delims, &context);
		}
		contents.push_back(row);
	}
	instream.close();
	heightMap = Mat(contents.size(), contents[0].size(), CV_32F);
	for (int i = 0; i < heightMap.rows; i++)
		heightMap.row(i) = Mat(contents[i]).t();
}
*/
/*int main() {
	system("python C:/Projects/rgbd-processor-python/depth2mask.py");
	system("pause");
}*/
void graph_to_card(float width, float height, vector<Vec2f> & rect) {

}
int main() {
	vector<Vec2f> rect1 = {Vec2f(150,100), Vec2f(350, 100), Vec2f(350,200),Vec2f(150,200) };
	vector<Vec2f> rect2 = {Vec2f(400,200), Vec2f(500,200), Vec2f(500,300), Vec2f(400,300) };
	vector<Vec2f> res;
	vector<vector<Vec2f>> rectVector;
	float width = 600, height = 400;

	graph_to_card(width, height, rect1);
	graph_to_card(width, height, rect2);
	Vec2f c1 = (rect1[0] + rect1[2]) / 2.0f;
	Vec2f c2 = (rect2[0] + rect2[2]) / 2.0f;
	rectVector.push_back(rect1);
	rectVector.push_back(rect2);
	
	int rectNum = 2;
	float rectX[8], rectY[8];
	for (int i = 0; i < rectVector.size(); i++) {
		vector<Vec2f> trect = rectVector[i];
		for (int j = 0; j < 4; j++) {
			rectX[4 * i + j] = trect[0][0];
			rectY[4 * i + j] = trect[0][1];
		}
	}
	
	// construct line equation from c1 c2
	if (c1[0] == c2[0]) {
		
		res.push_back(*min_element(rectX, rectX+8));
	}
	
	return 0;
}