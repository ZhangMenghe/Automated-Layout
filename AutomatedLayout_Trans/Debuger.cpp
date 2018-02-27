#include <iostream>
#include <fstream>
//using namespace cv;
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
int main() {
	system("python C:/Projects/rgbd-processor-python/depth2mask.py");
	system("pause");
}
/*int main() {
	FILE * file;
	//const char* filename = "C:/Users/menghe/Downloads/hello.py";
	const char* filename = "C:/Projects/rgbd-processor-python/depth2mask.py";

	int argc = 5;
	const char ** argv;
	Py_Initialize();
	//PyRun_SimpleString("import numpy as np");
	//PyRun_SimpleString("print('hello world')");

	PyImport_ImportModule("sys");
	if (PyImport_ImportModule("numpy.core.multiarray") == nullptr) {
		PyErr_Print();
		cout << "can not import" << endl;
	}
		
	//PyRun_SimpleString("import matplotlib");
	//fopen_s(&file, filename, "r");
	//if(file!=nullptr)
	//	PyRun_SimpleFile(file, filename);
	//Py_Finalize();
	system("pause");
	//namedWindow("Display window", CV_WINDOW_AUTOSIZE);
	//imshow("Display window", img);
	//waitKey(0);
	return 0;
}*/