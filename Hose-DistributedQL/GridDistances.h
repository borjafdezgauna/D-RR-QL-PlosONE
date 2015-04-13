#ifndef __GRID_DISTANCES__
#define __GRID_DISTANCES__

extern int g_relPosCountInRange;
extern int g_absPosCountInRange;

void InitDistMatrix(int maxRelX,int maxRelY,float maxRelDist,bool bAllowNullDist= false);
void ReleaseDisMatrix();

int CountRelPosInRange(float maxDist);

int GetRelPosIndex(int x,int y,int maxI);
void GetIthRelPos(int i,int &outX,int &outY);

#endif