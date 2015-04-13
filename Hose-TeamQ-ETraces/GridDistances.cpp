#include "stdafx.h"
#include "GridDistances.h"
#include "ParameterManager.h"

int g_relPosCountInRange= 0;
int g_absPosCountInRange= 0;

struct RelPos
{
	int relX;
	int relY;
	float relDist;
};

RelPos *g_pDistMatrix= 0;
int g_entryCount= 0;

void ReleaseDistMatrix()
{
	if (g_pDistMatrix)
	{
		delete [] g_pDistMatrix;
		g_pDistMatrix= 0;
	}
}

void InitDistMatrix(int maxRelX,int maxRelY,float maxRelDist,bool bAllowNullDist)
{
	ReleaseDistMatrix();

	g_entryCount= (maxRelX*2+1)*(maxRelY*2+1);
	if (!bAllowNullDist) g_entryCount--;

	g_pDistMatrix= new RelPos[g_entryCount];

	int i= 0;

	//construct all possible relative displacements
	for (int x=-maxRelX; x<=maxRelX;x++)
	{
		for (int y=-maxRelY; y<=maxRelY; y++)
		{
			if (x!=0 || y!=0 || bAllowNullDist)
			{
				g_pDistMatrix[i].relX= x;
				g_pDistMatrix[i].relY= y;
				g_pDistMatrix[i].relDist= sqrt((float)(x*x+y*y));

				i++;
			}
		}
	}

	//sort the relative displacements
	RelPos aux;
	for (int a= 0; a<g_entryCount-1; a++)
	{
		for (int b= a+1; b<g_entryCount; b++)
		{
			if (g_pDistMatrix[b].relDist<g_pDistMatrix[a].relDist)
			{
				//swap
				//aux<-matrix(a)
				aux.relX= g_pDistMatrix[a].relX;
				aux.relY= g_pDistMatrix[a].relY;
				aux.relDist= g_pDistMatrix[a].relDist;
				//matrix(a)<-matrix(b)
				g_pDistMatrix[a].relX= g_pDistMatrix[b].relX;
				g_pDistMatrix[a].relY= g_pDistMatrix[b].relY;
				g_pDistMatrix[a].relDist= g_pDistMatrix[b].relDist;
				//matrix(b)<-aux
				g_pDistMatrix[b].relX= aux.relX;
				g_pDistMatrix[b].relY= aux.relY;
				g_pDistMatrix[b].relDist= aux.relDist;
			}
		}
	}

	//MACHINE LEARNING - HOSE EXPERIMENT MOD/////////////////
	g_relPosCountInRange= CountRelPosInRange(maxRelDist);
	/////////////////////////////////////////////////////////

}

int CountRelPosInRange(float maxDist)
{
	int count= 0;
	int i= 0;
	while (i<g_entryCount && g_pDistMatrix[i].relDist<=maxDist)
	{
		count++;
		i++;
	}
	return count;
}

void GetIthRelPos(int i,int &outX,int &outY)
{
	if (i<0 || i>=g_entryCount) return;
	
	outX= g_pDistMatrix[i].relX;
	outY= g_pDistMatrix[i].relY;
}

int GetRelPosIndex(int x,int y,int maxI)
{
	for (int i= 0; i<g_entryCount && i<maxI; i++)
	{
		if (x==g_pDistMatrix[i].relX && y==g_pDistMatrix[i].relY) return i;
	}
	return -1;
}