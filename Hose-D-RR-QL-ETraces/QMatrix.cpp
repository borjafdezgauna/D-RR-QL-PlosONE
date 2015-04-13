#include "stdafx.h"
#include "QMatrix.h"
#include "ParameterManager.h"

/*
////////////////////////////////////////////////////////////////////////////////////////////
//CQENTRY
/////////////////////////////////////////////////////////////////////////////////////////////
CQEntry::CQEntry()
{

}

CQEntry::~CQEntry()
{
}

float CQEntry::GetMaxQ()
{
	float maxQ=m_qValue[0];
	for (int i= 1; i<ROBOT_ACTION_SPACE; i++)
	{
		if (m_qValue[i]>maxQ)
		{
			maxQ= m_qValue[i];
		}
	}
	return maxQ;
}

int CQEntry::GetMaxQAction()
{
	float maxQ=m_qValue[0];
	int maxQAction= 0;
	for (int i= 1; i<ROBOT_ACTION_SPACE; i++)
	{
		if (m_qValue[i]>maxQ)
		{
			maxQ= m_qValue[i];
			maxQAction= i;
		}
	}
	return maxQAction;
}*/


////////////////////////////////////////////////////////////////////////////////////////////
//CQMATRIX
/////////////////////////////////////////////////////////////////////////////////////////////
CQMatrix::CQMatrix()
{
	m_pQEntries= 0;
	m_entryCount= 0;
	m_actionCount= 0;
}

CQMatrix::~CQMatrix()
{
}

void CQMatrix::Init(int entryCount,int actionCount)
{
	m_entryCount= entryCount;
	m_actionCount= actionCount;

	m_pQEntries= new Q_VALUE_TYPE[m_entryCount*m_actionCount];
}

void CQMatrix::RandomlyPopulate()
{
	if (g_pParameterManager->GetIntParameter("REWARD_RANDOM_AMPLITUDE")!=0)
		for (int i= 0; i<m_entryCount*m_actionCount; i++)
		{
			m_pQEntries[i]= 0;//rand()% g_pParameterManager->GetIntParameter("REWARD_RANDOM_AMPLITUDE");
		}
}

void CQMatrix::Release()
{
	if (m_pQEntries)
	{
		delete [] m_pQEntries;
		m_pQEntries= 0;
	}
}

Q_VALUE_TYPE CQMatrix::GetMaxQ(int stateId)
{
	Q_VALUE_TYPE *pQEntry= &m_pQEntries[stateId*m_actionCount];
	Q_VALUE_TYPE maxQ=pQEntry[0];

	for (int i= 1; i<m_actionCount; i++)
	{
		if (pQEntry[i]>maxQ)
		{
			maxQ= pQEntry[i];
		}
	}
	return maxQ;
}

int CQMatrix::GetMaxQAction(int stateId)
{
	Q_VALUE_TYPE *pQEntry= &m_pQEntries[stateId*m_actionCount];
	Q_VALUE_TYPE maxQ=pQEntry[0];
	int maxQAction= 0;

	for (int i= 1; i<m_actionCount; i++)
	{
		if (pQEntry[i]>maxQ)
		{
			maxQ= pQEntry[i];
			maxQAction= i;
		}
	}
	return maxQAction;
}

/*
CQEntry* CQMatrix::GetQEntry(int stateId)
{
	if (stateId>=0 && stateId<m_entryCount && m_pQEntries) return &m_pQEntries[stateId];
	return 0;
}*/

