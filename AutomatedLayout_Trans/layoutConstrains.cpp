#include "layoutConstrains.h"
#include "predefinedConstrains.h"

#include <algorithm> 
#include <math.h>
using namespace std;
using namespace cv;

float dist_between_Vectors(const Vec3f& v1, const Vec3f& v2) {
	Vec3f tmp = v1 - v2;
	return sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1] + tmp[2] * tmp[2]);
}

/*float angle_between_points(const Point &v1, const Point &v2)
{
	float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
	float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

	float dot = v1.x * v2.x + v1.y * v2.y;

	float a = dot / (len1 * len2);

	if (a >= 1.0)
		return 0.0;
	else if (a <= -1.0)
		return PI;
	else
		return acos(a); // 0..PI
}*/

void layoutConstrains::get_pairwise_relation(const singleObj& obj1, const singleObj& obj2, int&pfg, float&m, float&M) {
	//std::map<int, std::pair<int, float>> pairMap;
	pfg = 0;
	int indexType = std::min(obj1.catalogId ,obj2.catalogId);
	int compType = std::max(obj1.catalogId, obj2.catalogId);
	for (map<int, vector<pair<int, cv::Vec2f>>>::iterator it = room->pairMap.begin(); it != room->pairMap.end(); it++) {
		if (it->first == indexType) {
			for (int i = 0; i < it->second.size(); i++) {
				if (get<0>(it->second[i]) == compType) {
					pfg = 1;
					m = get<1>(it->second[i])[0];
					M = get<1>(it->second[i])[1];
					return;
				}
			}
		}
	}
}

float layoutConstrains::t(float d, float m, float M, int a) {
	if (d < m)
		return pow((d / m), a);
	else if (d > M)
		return pow((M / d), a);
	else
		return 1.0f;
}

//Clearance : 
// Mcv(I) that minimize the overlap between furniture(with space)
// the minimal the better
void layoutConstrains::cal_clearance_violation(float& mcv) {
	mcv = 0;
	for (int i = 0; i < room->objctNum; i++) {		
		for (int j = i + 1; j < room->objctNum; j++) {
			mcv += room->cal_overlapping_area(room->objects[i].boundingBox, room->objects[j].boundingBox);
		}
	}
}
void Dilation(const Mat& src, Mat& dilation_dst, int dilation_size, int dilation_type = 0) {
	/*if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
*/
	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	dilate(src, dilation_dst, element);
}
//Circulation:
//Mci support circulation through the room and access to all of the furniture.
//Compute free configuration space of a person on the ground plane of the room
//represent a person with radius = 18
void layoutConstrains::cal_circulation_term(float& mci) {
	Mat dilation_dst;
	Dilation(room->furnitureMask, dilation_dst, PersonArea);
	mci = sum(dilation_dst)[0];
}

//Pairwise relationships:
//Mpd: for example  coffee table and seat
//mpa: relative direction constraints
void layoutConstrains::cal_pairwise_relationship(float& mpd, float& mpa) {
	mpd = 0, mpa = 0;
	int pfg;
	float m, M;
	for (int i = 0; i < room->objctNum; i++) {
		for (int j = i + 1; j < room->objctNum; j++) {
			get_pairwise_relation(room->objects[i], room->objects[j], pfg, m, M);
			if (pfg) {
				mpd -= t(dist_between_Vectors(room->objects[i].translation, room->objects[j].translation), m, M);
				Vec3f f(-sin(room->objects[i].zrotation), cos(room->objects[i].zrotation), 0.0);
				Vec3f g(-sin(room->objects[j].zrotation), cos(room->objects[j].zrotation), 0.0);
				float cosfg = f.dot(g);
				mpa -= 8 * cosfg*cosfg*cosfg*cosfg - 8 * cosfg*cosfg + 1;
			}			
		}
	}
}

//Conversation
//Mcd:group a collection of furniture items into a conversation area
void layoutConstrains::cal_conversation_term(float& mcd, float& mca) {
	vector<int>itemIdx;
	mcd = 0;
	mca = 0;
	singleObj tobj;
	float cosfg, cosgf;
	for (map<int, vector<int>>::iterator it = room->objGroupMap.begin(); it != room->objGroupMap.end(); ++it) {
		itemIdx = it->second;
		for (vector<int>::iterator itx = itemIdx.begin(); itx != itemIdx.end(); ++itx) {
			//tobj = room->objects[*itx];
			singleObj * tobj = &room->objects[*itx];

			if (tobj->catalogId == TYPE_CHAIR) {
				for (vector<int>::iterator ity = itx + 1; ity != itemIdx.end(); ++ity) {
					singleObj * tmp = &room->objects[*ity];
					
					if (tmp->catalogId == TYPE_CHAIR) {
						mcd += t(dist_between_Vectors(tobj->translation, tmp->translation), CONVERSATION_M_MIN, CONVERSATION_M_MAX);
						//compute phi_fg and phi_gf
						Vec3f n1(cos(tobj->zrotation), sin(tobj->zrotation), 0.0);
						Vec3f n2(cos(tmp->zrotation), sin(tmp->zrotation), 0.0);
						Vec3f d = tobj->translation - tmp->translation;
						cosfg = n1.dot(-d); cosgf = n2.dot(d);
						mca -= (cosfg + 1) * (cosgf + 1);
					}
				}
			}
		}
	}
}

//balance:
//place the mean of the distribution of visual weight at the center of the composition
void layoutConstrains::cal_balance_term(float &mvb) {
	float sumArea = .0f, oneArea;
	Vec3f centroid(.0f, .0f, .0f);
	for (int i = 0; i < room->objctNum; i++) {
		oneArea = room->objects[i].boundingBox.area();
		centroid += oneArea * room->objects[i].translation;
		sumArea += oneArea;
	}
	centroid /= sumArea;
	mvb = dist_between_Vectors(centroid, room->center);
}

//Alignment:
//compute furniture alignment term
void layoutConstrains::cal_alignment_term(float& mfa, float&mwa) {
	vector<int>itemIdx;
	mfa = 0;
	mwa = 0;
	singleObj tobj;
	for (map<int, vector<int>>::iterator it = room->objGroupMap.begin(); it != room->objGroupMap.end(); ++it) {
		itemIdx = it->second;
		for (vector<int>::iterator itx = itemIdx.begin(); itx != itemIdx.end(); ++itx) {
			//tobj = room->objects[*itx];
			singleObj * tobj = &room->objects[*itx];

			for (vector<int>::iterator ity = itx + 1; ity != itemIdx.end(); ++ity) {
				singleObj * tmp = &room->objects[*ity];
				mfa -= cos(4 * (tobj->zrotation - tmp->zrotation));
			}
			mwa -= cos(4 * (tobj->zrotation - room->walls[tobj->nearestWall].zrotation - PI/2));
		}	
	}
}

void layoutConstrains::get_all_reflection(map<int, Vec3f> focalPoint_map, vector<Vec3f> &reflectTranslate, vector<float> & reflectZrot, float refk) {
	vector<int>itemIdx;
	int idx = 0;
	Vec3f objPos;
	float invk = .0f;
	if(refk != 0 && refk!= INFINITY)
		invk = 1 / refk;
	for (map<int, Vec3f>::iterator it = focalPoint_map.begin(); it != focalPoint_map.end(); it++) {
		itemIdx = room->objGroupMap[it->first];
		Vec3f focalPoint = it->second;
		float b = focalPoint[1] - focalPoint[0] * refk;
		for (vector<int>::iterator itx = itemIdx.begin(); itx != itemIdx.end(); ++itx) {
			singleObj * tobj = &room->objects[*itx];
			objPos = tobj->translation;


			if (refk == INFINITY) {
				reflectTranslate.push_back(Vec3f(2*focalPoint[0]- objPos[0], objPos[1], .0f));
				reflectZrot.push_back(-tobj->zrotation);
				idx++;
			}
			else if (refk == 0) {
				reflectTranslate.push_back(Vec3f(objPos[0], 2 * focalPoint[1] - objPos[1], .0f));
				reflectZrot.push_back(PI - tobj->zrotation);
				idx++;
			}
			else {
				float x = 2 * objPos[1] + (invk - refk)*objPos[0] - 2 * b;
				float y = -invk * x + objPos[1] + invk*objPos[0];
				reflectTranslate.push_back(Vec3f(x, y, .0f));
				reflectZrot.push_back(PI - tobj->zrotation - 2 * atan2f(objPos[1] - y, objPos[0] - x));
				idx++;
			}
				
		}
	}
}
//Emphasis:
//compute focal center
void layoutConstrains::cal_emphasis_term(float& mef, float& msy, float gamma) {
	vector<int>itemIdx;

	mef = 0; msy = 0;
	float phi_gps;
	vector<Vec3f> reflectTranslate;
	vector<float> reflectZrot;
	get_all_reflection(room->focalPoint_map, reflectTranslate, reflectZrot);

	for (map<int, vector<int>>::iterator it = room->objGroupMap.begin(); it != room->objGroupMap.end(); ++it) {
		itemIdx = it->second;
		float maxS = -INFINITY;
		//find each group's focal point
		Vec3f focalPoint = room->focalPoint_map[it->first];

		for (vector<int>::iterator itx = itemIdx.begin(); itx != itemIdx.end(); ++itx) {
			//if (*itx >= 1000)
			//	continue;
			//singleObj * tobj = &room->objects[*itx];
			singleObj * tobj = &room->objects[*itx];

			Vec3f pd = focalPoint - tobj->translation;
			float dist = dist_between_Vectors(focalPoint, tobj->translation);
			pd /= dist;
			Vec3f n1(cos(tobj->zrotation), sin(tobj->zrotation), .0f);
			mef -= pd.dot(n1);

			for (vector<int>::iterator ity = itx + 1; ity != itemIdx.end(); ++ity) {
				//if (*ity >= 1000)
				//	continue;
				int idy = (*ity >= 1000) ? *ity - 1000 : *ity;
				float tmps = cos(tobj->zrotation - reflectZrot[idy]) - gamma*dist_between_Vectors(focalPoint, reflectTranslate[idy]);
				maxS = (tmps > maxS)? tmps:maxS;
			}
			msy -= maxS;
		}
	}
}

vector<float> layoutConstrains::get_all_constrain_terms() {
	float mcv =0, mci=0, mpd=0, mpa=0, mcd=0, mca=0, mvb=0, mfa=0, mwa=0, mef=0, msy=0;
	cal_clearance_violation(mcv);
	cal_circulation_term(mci);//single check works
	cal_pairwise_relationship(mpd, mpa);
	cal_conversation_term(mcd, mca);
	cal_balance_term(mvb);
	if(room->wallNum!=0)
		cal_alignment_term(mfa, mwa);
	if(!room->focalPoint_map.empty())
		cal_emphasis_term(mef, msy);
	float parameters[] = { mcv, mci, mpd, mpa, mcd, mca, mvb, mfa, mwa, mef, msy };
	constrain_terms.assign(parameters, parameters + 11);
	return constrain_terms;
}
