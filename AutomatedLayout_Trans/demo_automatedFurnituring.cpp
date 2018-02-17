#include <iostream>
#include "automatedLayout.h"
#include "room.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

/*void plotLayoutResult(const automatedLayout * layout) {
	Mat img = Mat::ones(600, 800, CV_32F);
	// You can also read in an existing place
	/*Mat original = imread("1.png", CV_LOAD_IMAGE_GRAYSCALE);
	if (original.empty())
	{
		cout << "Could not open or find the image" << endl;
		return ;
	}*/
	/*Room * lr = layout->room;
	for (int i = 0; i < lr->wallNum; i++) {
		wall* tmp = &lr->walls[i];
		Point2f p1 = lr->card_to_graphics_coord_Point(lr->width / 2, lr->height / 2, tmp->vertices[0]);
		Point2f p2 = lr->card_to_graphics_coord_Point(lr->width / 2, lr->height / 2, tmp->vertices[1]);
		line(img, p1, p2, Scalar(255, 0, 0), 2, 8, 0);
	}//draw a bunch of walls
	for (int i = 0; i < lr->objctNum; i++) {
		singleObj * tmp = &lr->objects[i];
		rectangle(img, tmp->boundingBox.tl(), tmp->boundingBox.br(), Scalar(255, 0, 0), 2, 8, 0);
	}//draw a bunch of objects
	//Rect RectangleToDraw(10.5f, 10.5f, 100, 100);
	//rectangle(img, RectangleToDraw.tl(), RectangleToDraw.br(), Scalar(0, 0, 255), 2, 8, 0);
	imshow("DrawRectangle", img);
	while (char(waitKey(0))) {};
}*/

int main() {
	Room* room = new Room();
	//TODO:CHECK OUTBOUND
	room->add_a_wall(Vec3f(0, 300, 0), 90, 800, 10);
	room->add_a_wall(Vec3f(400, 0, 0), 0, 600, 10);
	room->add_an_object(Vec3f(0, 0, 0), 90, 100, 200, 10, TYPE_CHAIR);
	room->add_an_object(Vec3f(100, 0, 0), 90, 100, 200, 10, TYPE_CHAIR);
	room->add_a_focal_point(Vec3f(0, 300, 0));

	automatedLayout* layout = new automatedLayout(room);
	layout->generate_suggestions();
	layout->display_suggestions();
	//plotLayoutResult(layout);
	system("pause");
	return 0;
}
