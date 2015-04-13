#include "stdafx.h"
#include "AuxFunctions.h"

void GetActionName(int actionId,char *pActionName,int bufferSize)
{
/*	switch (actionId)
	{
	case ACTION_UP: strcpy_s(pActionName,bufferSize,"UP");break;
	case ACTION_RIGHT: strcpy_s(pActionName,bufferSize,"RIGHT");break;
	case ACTION_DOWN: strcpy_s(pActionName,bufferSize,"DOWN");break;
	case ACTION_LEFT: strcpy_s(pActionName,bufferSize,"LEFT");break;
	case ACTION_LEFT_UP: strcpy_s(pActionName,bufferSize,"LEFT_UP");break;
	case ACTION_LEFT_DOWN: strcpy_s(pActionName,bufferSize,"LEFT_DOWN");break;
	case ACTION_RIGHT_UP: strcpy_s(pActionName,bufferSize,"RIGHT_UP");break;
	case ACTION_RIGHT_DOWN: strcpy_s(pActionName,bufferSize,"RIGHT_DOWN");break;
	case ACTION_NONE: strcpy_s(pActionName,bufferSize,"NONE");break;
	default: strcpy_s(pActionName,bufferSize,"INVALID_ACTION");break;
	}*/
}



bool PointInSegment(float startX,float startY,float endX,float endY,float pointX,float pointY)
{
	float v_x= endX - startX;
	float v_y= endY - startY;

	bool bPointOnLine= fabs(v_x * (pointY - endY) - (pointX - endX) * v_y) < 0.0000001;

	if (!bPointOnLine)
		return false;


	if (v_x!=0)
	{
		float u_x= (pointX-startX)/(endX-startX);
		if (u_x<0.f || u_x>1.f)
			return false;
	}
	else if (pointX!=startX)
		return false;

	if (v_y!=0)
	{
		float u_y= (pointY-startY)/(endY-startY);
		if (u_y<0.f || u_y>1.f)
			return false;
	}
	else if (pointY!=startY)
		return false;

    return true;
}

bool SegmentsIntersect(float start1X,float start1Y,float end1X,float end1Y
		,float start2X,float start2Y,float end2X,float end2Y)
{
/*	vector u = end1 -start1;
	vector v= end2 - start2;
	float D= u.x * v.y - u.y*v.x;
	if (fabs(D)<0.0f) return false;
	vector w= start1-start2;
	float s= v.x*w.y - v.y*w.x;
	float sd=s/D;
	if (sd<0.0f || sd>1.f) return false;
	float t= u.x * w.y - u.y *w.x;
	float td= t/D;
	if (td<0.0f || td>1.f) return false;
	return true;*/
	float u_x= end1X - start1X;
	float u_y= end1Y - start1Y;
	float v_x= end2X - start2X;
	float v_y= end2Y - start2Y;
	float D= u_x * v_y - u_y * v_x;
	if (fabs(D)<=0.0f) return false;
	float w_x= start1X - start2X;
	float w_y= start1Y - start2Y;
	float s= v_x*w_y - v_y*w_x;
	float sd=s/D;
	if (sd<0.0f || sd>1.f) return false;
	float t= u_x * w_y - u_y *w_x;
	float td= t/D;
	if (td<0.0f || td>1.f) return false;
	return true;
}

float Sign(float tri1_x,float tri1_y,float tri2_x,float tri2_y,float tri3_x,float tri3_y)
{
	float sign= (tri1_x - tri3_x) * (tri2_y - tri3_y) - (tri2_x - tri3_x) * (tri1_y - tri3_y);
	return sign;
}

bool PointInsideTriangle(float tri1_x,float tri1_y,float tri2_x,float tri2_y
												 ,float tri3_x,float tri3_y,float p_x,float p_y)
{
	/*float area_orig,area_new;

	area_orig= TriangleArea(tri1_x,tri1_y,tri2_x,tri2_y,tri3_x,tri3_y);
	area_new= TriangleArea(tri1_x,tri1_y,tri2_x,tri2_y,p_x,p_y)
		+ TriangleArea(tri1_x,tri1_y,tri3_x,tri3_y,p_x,p_y)
		+ TriangleArea(tri3_x,tri3_y,tri2_x,tri2_y,p_x,p_y);

	return fabs(area_orig-area_new) < 0.00001;*/
	float b1, b2, b3;

	b1 = Sign(p_x,p_y, tri1_x,tri1_y, tri2_x,tri2_y);
	if (b1==0.f && PointInSegment(tri1_x,tri1_y,tri2_x,tri2_y,p_x,p_y))
		return true;
	b2 = Sign(p_x,p_y, tri2_x,tri2_y, tri3_x,tri3_y);
	if (b2==0.f && PointInSegment(tri2_x,tri2_y,tri3_x,tri3_y,p_x,p_y))
		return true;
	b3 = Sign(p_x,p_y, tri3_x,tri3_y, tri1_x,tri1_y);
	if (b3==0.f && PointInSegment(tri3_x,tri3_y,tri1_x,tri1_y,p_x,p_y))
		return true;
	return ((b1*b2>=0.f) && (b2*b3>=0.f) && (b3*b1>=0.f));
}

float TriangleArea(float tri1_x,float tri1_y,float tri2_x,float tri2_y
										   ,float tri3_x,float tri3_y)
{
	return fabs(tri1_x*tri2_y+tri2_x*tri3_y+tri3_x*tri1_y-tri1_x*tri3_y-tri3_x*tri2_y-tri2_x*tri1_y)/2 ;
}

bool TriangleSegmentIntersect(float tri1_x,float tri1_y,float tri2_x,float tri2_y,float tri3_x,float tri3_y
		,float seg1_x,float seg1_y,float seg2_x,float seg2_y)
{
	//vertices inside the triangle?
	if (PointInsideTriangle(tri1_x,tri1_y,tri2_x,tri2_y,tri3_x,tri3_y,seg1_x,seg1_y))
		return true;
	if (PointInsideTriangle(tri1_x,tri1_y,tri2_x,tri2_y,tri3_x,tri3_y,seg2_x,seg2_y))
		return true;

	//segments intersect?
	if (SegmentsIntersect(tri1_x,tri1_y,tri2_x,tri2_y,seg1_x,seg1_y,seg2_x,seg2_y))
		return true;
	if (SegmentsIntersect(tri2_x,tri2_y,tri3_x,tri3_y,seg1_x,seg1_y,seg2_x,seg2_y))
		return true;
	if (SegmentsIntersect(tri3_x,tri3_y,tri1_x,tri1_y,seg1_x,seg1_y,seg2_x,seg2_y))
		return true;
	return false;
}



int GetDiscretizedAngleId(double angle)
{
/*	double angleZoneSize= 2*PI/DISCRETE_ANGLE_COUNT;
	double angleZone= -7*PI/DISCRETE_ANGLE_COUNT;

	if (angle<angleZone) return ACTION_LEFT;
	else
	{
		angleZone+= angleZoneSize;
		if (angle<angleZone) return ACTION_LEFT_DOWN;
		else
		{
			angleZone+= angleZoneSize;
			if (angle<angleZone) return ACTION_DOWN;
			else
			{
				angleZone+= angleZoneSize;
				if (angle<angleZone) return ACTION_RIGHT_DOWN;
				else
				{
					angleZone+= angleZoneSize;
					if (angle<angleZone) return ACTION_RIGHT;
					else
					{
						angleZone+= angleZoneSize;
						if (angle<angleZone) return ACTION_RIGHT_UP;
						else
						{
							angleZone+= angleZoneSize;
							if (angle<angleZone) return ACTION_UP;
							else
							{
								angleZone+= angleZoneSize;
								if (angle<angleZone) return ACTION_LEFT_UP;
								else
								{
									angleZone+= angleZoneSize;
									if (angle<angleZone) return ACTION_LEFT;
									else
									{
										angleZone+= angleZoneSize;
										if (angle<angleZone) return ACTION_DOWN;
										else return ACTION_NONE;
									}
								}
							}
						}
					}
				}
			}
		}
	}*/
	return 0;

}

int GetDiscretizedDistId(double dist)
{
	int distId= 0;
	int distZone= 0;
	while (distId<DISCRETE_DIST_COUNT-2 && distZone<dist)
	{
		distId++;
		distZone+= distId;
	}
	return distId;
}