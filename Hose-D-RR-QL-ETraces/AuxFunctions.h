#ifndef __AUXFUNCTIONS__
#define __AUXFUNCTIONS__

#define PI 3.14159265

void GetActionName(int actionId,char *pActionName,int bufferSize);
bool SegmentsIntersect(float start1X,float start1Y,float end1X,float end1Y
	,float start2X,float start2Y,float end2X,float end2Y);
bool PointInSegment(float startX,float startY,float endX,float endY,float pointX,float pointY);
bool PointInsideTriangle(float tri1_x,float tri1_y,float tri2_x,float tri2_y
											 ,float tri3_x,float tri3_y,float p_x,float p_y);
float TriangleArea(float tri1_x,float tri1_y,float tri2_x,float tri2_y,float tri3_x,float tri3_y);
bool TriangleSegmentIntersect(float tri1_x,float tri1_y,float tri2_x,float tri2_y,float tri3_x,float tri3_y
	,float seg1_x,float seg1_y,float seg2_x,float seg2_y);
float Sign(float tri1_x,float tri1_y,float tri2_x,float tri2_y,float tri3_x,float tri3_y);

#define DISCRETE_ANGLE_COUNT 8
#define DISCRETE_DIST_COUNT 10
#define NO_ANGLE 666.

int GetDiscretizedAngleId(double angle);
int GetDiscretizedDistId(double dist);

#endif
