#ifndef __QMATRIX__
#define __QMATRIX__

#include "GlobalDefines.h"



class CQMatrix
{
	Q_VALUE_TYPE *m_pQEntries;
	int m_entryCount;
	int m_actionCount;
public:
	CQMatrix();
	~CQMatrix();

	void Init(int entryCount,int actionCount);
	void RandomlyPopulate();

	void Release();

	Q_VALUE_TYPE* GetQEntry(int stateId)
	{
		if (stateId>=0 && stateId<m_entryCount && m_pQEntries) return &m_pQEntries[stateId*m_actionCount];
		return 0;
	}
	Q_VALUE_TYPE GetMaxQ(int stateId);
	int GetMaxQAction(int stateId);
	int GetEntryCount(){return m_entryCount;}
};

#endif