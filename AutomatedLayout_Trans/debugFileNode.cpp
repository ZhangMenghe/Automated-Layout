#include "include/opencv2/core/core.hpp"
#include "include/opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	string filename = "heightMapData.yml"; //argv[1]
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	FileNode node = fs["floatdata"];
	Mat test = node.mat();
	namedWindow("test", 1);
	imshow("test", test);
	waitKey(0);
	system("pause");
	return 0;
}