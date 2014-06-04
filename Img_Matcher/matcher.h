//#ifndef MATCHER_H
//#define MATCHER_H



//#ifdef SIMPLEDLL_EXPORT
#define DLL_EXPORT __declspec(dllexport)
//#else
//#define DLL_EXPORT __declspec(dllimport)
//#endif

#include <opencv/cv.h>    
#include <opencv/highgui.h>

#include <nonfree/features2d.hpp>
#include <legacy/legacy.hpp>
#include <vector>

typedef unsigned char byte;

using namespace std;
using namespace cv;

typedef struct{
	Point2f pt;
	float angle;
} POINT_ANGLE;

typedef struct{
	Point2f pt;
	float angle;
	int index;
} POINT_ANGLE_INDEX;

typedef struct{
	Point2f pt;
	int index;
} POINT_INDEX;

typedef struct{
	int x;
	int y;
} DIRECTION;

typedef struct{
	int x;
	int y;
} LOCATION;

//vector<KeyPoint> keyPoints_1;

class Matcher
{
public:
	Matcher();
	~Matcher();

private:

	//vector<int> ModelIndexLIb;
	//
	//vector<int*> template_point_count;
	//vector<LOCATION***> template_location;
	//vector<DIRECTION***> template_direction;
	//vector<float**> template_direction_length;

	//vector<int*> angle_size;
	//vector<float*> step;
	//vector<int> m_size;

	int *template_point_count;
	LOCATION*** template_location;
	DIRECTION*** template_direction;
	float** template_direction_length;

	int* angle_size;
	float* step;
	int m_size;


	//Point point1;
	//Point point2;
	//Point point3;
	//Point point4;


public:
	int CreateModel(byte *image, int width, int height, int MaxLevel, int sobel_size, float step_length, double threshold1, double threshold2);
	int Match(byte *input_image, int width, int height, int MaxLevel, float min_angle, float max_angle, int max_instance, float score_threshold, float* x, float* y, float* angle, float* score);
};

extern "C" class DLL_EXPORT MatcherAdapter
{
public:
	static int CreateModel(unsigned char* model, int width, int height, int MaxLevel, int sobel_size, float step_length, double threshold1, double threshold2, int MatcherIndex);
	static int Match(unsigned char* image, int width, int height, int MaxLevel, float min_angle, float max_angle,int max_instance, float score_threshold, float* x, float* y, float* angle, float* score, int MatcherIndex);

private:
	static Matcher matcher[999];
};






//#endif // MATCHER_H

