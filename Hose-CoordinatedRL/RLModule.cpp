#include "stdafx.h"
#include "RLModule.h"
#include "QMatrix.h"
#include "ParameterManager.h"
#include "RLManager.h"

double CRLModule::m_negativeReward= 0.;
double CRLModule::m_positiveReward= 0.;
double CRLModule::m_neutralReward;

int CRLModule::m_robotCount= 0;
double CRLModule::m_hoseSegmentLength= 0;
int CRLModule::m_gridSizeX= 0;
int CRLModule::m_gridSizeY= 0;
int CRLModule::m_gridMaxX= 0;
int CRLModule::m_gridMaxY= 0;
int CRLModule::m_maxAbsDist= 0;
int CRLModule::m_actionCount= 0;


CRLModule::CRLModule()
{
	m_pQMatrix= new CQMatrix();
}

CRLModule::~CRLModule()
{
	delete m_pQMatrix;
}

void CRLModule::SetId(int id)
{
	m_id= id;
}

void CRLModule::StateFromStateId(int stateId,bool bCurrentState)
{
	IStateView* pState= GetStateHandle(bCurrentState);
	pState->StateFromStateId(stateId);
}

int CRLModule::GetStateSpace()
{
	return m_pState->GetStateSpace();
}

void CRLModule::Init()
{

	m_negativeReward= (float)g_pParameterManager->GetDoubleParameter("REWARD_NEGATIVE");
	m_positiveReward= (float)g_pParameterManager->GetDoubleParameter("REWARD_POSITIVE");
	m_neutralReward= (float)g_pParameterManager->GetDoubleParameter("REWARD_NEUTRAL");
	m_hoseSegmentLength= g_pParameterManager->GetDoubleParameter("HOSE_LENGTH")
		/g_pParameterManager->GetDoubleParameter("ROBOT_COUNT");
	m_robotCount= g_pParameterManager->GetIntParameter("ROBOT_COUNT");
	m_gridSizeX= g_pParameterManager->GetIntParameter("GRID_SIZE_X");
	m_gridSizeY= g_pParameterManager->GetIntParameter("GRID_SIZE_Y");
	m_gridMaxX= g_pParameterManager->GetIntParameter("GRID_MAX_X");
	m_gridMaxY= g_pParameterManager->GetIntParameter("GRID_MAX_Y");
	m_maxAbsDist= g_pParameterManager->GetIntParameter("MAX_ABSOLUTE_DIST");
	m_actionCount= g_pParameterManager->GetIntParameter("ACTION_COUNT");
	
	//matrices
	m_pQMatrix->Init(m_pState->GetStateSpace(),m_actionCount);
	m_pQMatrix->RandomlyPopulate();
	m_pDummyQEntry= new Q_VALUE_TYPE[m_actionCount];
	for (int a= 0; a<m_actionCount; a++)
		m_pDummyQEntry[a]= m_negativeReward;

	m_lastAction= ACTION_NONE;
}

void CRLModule::GenerateStateTransition(int action)
{
	m_pNextState->GenerateStateTransition(m_pState,action);
}

void CRLModule::SaveQMatrix(char *pFilenameFormat)
{
	FILE *pFile;
	char filename[256];
	int robotId;

	if (m_bGenericModule) robotId= 0;
	else robotId= m_id;

	sprintf_s(filename,pFilenameFormat
		,g_pParameterManager->GetIntParameter("EXPERIMENT_ID")
		,robotId
		,GetModuleId());
	if (0==fopen_s(&pFile,filename,"wb"))
	{
		int entryCount= m_pQMatrix->GetEntryCount();
		fwrite(&entryCount,sizeof(int),1,pFile);
		fwrite(&m_actionCount,sizeof(int),1,pFile);
		int harl= fwrite(m_pQMatrix->GetQEntry(0),sizeof(Q_VALUE_TYPE),entryCount*m_actionCount,pFile);
		fclose(pFile);
	}
	
}

void CRLModule::LoadQMatrix(char *pFilenameFormat)
{
	FILE *pFile;
	char filename[256];
	bool bLoaded= false;
	int robotId;

	if (m_bGenericModule) robotId= 0;
	else robotId= m_id;

	sprintf_s(filename,pFilenameFormat
		,g_pParameterManager->GetIntParameter("EXPERIMENT_ID")
		,robotId
		,GetModuleId());
	if (0==fopen_s(&pFile,filename,"rb"))
	{
		int entryCount;
		int actionCount;
		fread(&entryCount,sizeof(int),1,pFile);
		fread(&actionCount,sizeof(int),1,pFile);
		if (entryCount!= m_pQMatrix->GetEntryCount() || actionCount!=m_actionCount)
			printf("ERROR IMPORTING QMATRIX. DIFFERENT MATRIX SIZE");
		else
		{
			fread(m_pQMatrix->GetQEntry(0),sizeof(Q_VALUE_TYPE),entryCount*m_actionCount,pFile);
			bLoaded= true;
		}
		fclose(pFile);
	}
//	if (!bLoaded) m_pQMatrix->RandomlyPopulate();
}

int CRLModule::GetMaxQAction(Q_VALUE_TYPE *pQEntry)
{
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

Q_VALUE_TYPE* CRLModule::GetQEntry(bool bCurrentState)
{
	int stateId;
	
	if (bCurrentState)
		stateId= m_pState->GetStateId();
	else
		stateId= m_pNextState->GetStateId();

	if (stateId<0)
		return m_pDummyQEntry;


	return m_pQMatrix->GetQEntry(stateId);
}

bool CRLModule::IsStateValid(bool bCurrentState)
{
	if (bCurrentState)
		return m_pState->GetStateId() >= 0;
	else
		return m_pNextState->GetStateId() >= 0;
}

void CRLModule::UpdateQEntry(CWorldState *pWorldState, bool bInmediateReward)
{
	int idQ_s_a, idQ_s_a2;
	Q_VALUE_TYPE *pQ_s_a= 0;
	double maxQ;
	double reward;

	
	if (bInmediateReward)//IsConstraint() && bConstraints)
	{
		reward= GetReward(pWorldState);
		m_lastReward= reward;

		idQ_s_a= m_pState->GetStateId();

		pQ_s_a= m_pQMatrix->GetQEntry(idQ_s_a);

		idQ_s_a2= m_pNextState->GetStateId();

		if (idQ_s_a2>=0) maxQ= m_pQMatrix->GetMaxQ(idQ_s_a2);
		else maxQ= m_negativeReward;

		pQ_s_a[m_lastAction]+= m_alpha*(reward + m_gamma*maxQ - pQ_s_a[m_lastAction]);

		UpdateState();
	}
	else if (!IsConstraint())// && !bConstraints)
	{
		reward= GetReward(pWorldState);
		m_lastReward= reward;

		idQ_s_a= m_lastStateId;//m_pState->GetStateId();
		if (idQ_s_a>=0)
			pQ_s_a= m_pQMatrix->GetQEntry(idQ_s_a);

		idQ_s_a2= m_pState->GetStateId();//m_pNextState->GetStateId();


		if (idQ_s_a2>=0 && pQ_s_a)
		{
			maxQ= m_pQMatrix->GetMaxQ(idQ_s_a2);
			pQ_s_a[m_lastAction]+= m_alpha*(reward + m_gamma*maxQ - pQ_s_a[m_lastAction]);
		}
		m_lastStateId= idQ_s_a2;
	}
}

void CRLModule::UpdateState()
{
	m_pState->CopyFrom(m_pNextState);
}

void CRLModule::SetAction(int actionId)
{
	m_lastAction= actionId;
}

void CRLModule::SetEpsilon(float epsilon)
{
	m_epsilon= epsilon;
}

bool CRLModule::IsStateTerminal(int &terminationCode)
{
	if (m_bStateIsTerminal)
	{
		terminationCode=m_terminationCode;
		return true;
	}
	return false;
}