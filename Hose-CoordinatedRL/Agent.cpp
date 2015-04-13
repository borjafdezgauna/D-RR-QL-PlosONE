#include "stdafx.h"
#include "Agent.h"
//RL Modules

#include "GoalReachModule.h"

#include "QMatrix.h"
#include "WorldState.h"
#include "ParameterManager.h"


CAgent::CAgent()
{
	m_id= -1;
	m_RLModuleCount= 0;//g_pParameterManager->GetIntParameter("RL_MODULE_COUNT");
	m_lastAction= ACTION_NONE;


	//m_pRLModules[3]= new CGoalReachModule();
	m_actionCount = g_pParameterManager->GetIntParameter("ACTION_COUNT");

	m_pQBuffer= new Q_VALUE_TYPE[m_actionCount];
	m_pVetoBuffer= new bool [m_actionCount];
}

CAgent::~CAgent()
{
	for (int module= 0; module<m_RLModuleCount; module++)
		delete m_pRLModules[module];
	delete [] m_pRLModules;

	delete [] m_pQBuffer;
	delete [] m_pVetoBuffer;
}

void CAgent::SaveQMatrices(char* pFilenameFormat)
{
	for (int module= 0; module<m_RLModuleCount; module++)
		m_pRLModules[module]->SaveQMatrix(pFilenameFormat);
}

void CAgent::LoadQMatrices(char* pFilenameFormat)
{
	for (int module= 0; module<m_RLModuleCount; module++)
		m_pRLModules[module]->LoadQMatrix(pFilenameFormat);
}

void CAgent::Init(int id)
{
	m_id= id;
	int usevetos= g_pParameterManager->GetIntParameter("USE_VETOS");
	m_bUseVetos= 0!=g_pParameterManager->GetIntParameter("USE_VETOS");
	//if (g_pParameterManager->GetIntParameter("HOMOBOTS")==1)
	{
		m_RLModuleCount= 1;
		m_pRLModules= new CRLModule* [m_RLModuleCount];
		m_pRLModules[0]= new CGoalReachModule();

	}
	for (int module= 0; module<m_RLModuleCount; module++)
	{
		m_pRLModules[module]->SetId(id);
		m_pRLModules[module]->Init();
	}
}


void CAgent::SetEpsilon(float epsilon)
{
	m_epsilon= epsilon;

	for (int module= 0; module<m_RLModuleCount; module++)
		m_pRLModules[module]->SetEpsilon(epsilon);
}

bool CAgent::IsStateTerminal(int &blamedModuleId,int &blamedRobotId)//TerminationInfo& termInfo)
{
	int termCode;
	for (int module= 0; module<m_RLModuleCount; module++)
	{
		if (m_pRLModules[module]->IsStateTerminal(termCode))
		{
			blamedModuleId= module;//m_pRLModules[module]->GetModuleId();
			blamedRobotId= m_id;
			/*
			termInfo.module= m_pRLModules[module]->GetModuleId();
			termInfo.robotId= m_id+1;*/
			return true;
		}
	}
	return false;
}

bool CAgent::IsStateValid()
{
	for (int module= 0; module<m_RLModuleCount; module++)
	{
		if (!m_pRLModules[module]->IsStateValid(true))
		{
			return false;
		}
	}
	return true;
}

int CAgent::TimeStep(CWorldState* pWorldState)
{
	FilterState(pWorldState,true);	//acquire state again as other agents might have changed it

	if (!IsStateValid()) return -1;

	UpdateQMatrices(pWorldState, false);	//update q matrices from all modules
									//world state is passed as an argument to calculate rewards

	int action= SelectAction();		//greedy, e-greedy, ...

	TakeAction(action,pWorldState); //update world

	FilterState(pWorldState,false);	//acquire state again as other agents might have changed it

	UpdateQMatrices(pWorldState, true);
//	FilterState(pWorldState,false);	//spread new state among all modules

//	if (!IsStateValid()) return -1;

	return action;
}

void CAgent::FilterState(CWorldState* pWorldState, bool currentState)
{
	for (int module= 0; module<m_RLModuleCount; module++)
	{
		m_pRLModules[module]->FilterWorldState(pWorldState,currentState);
	}
}

void CAgent::UpdateQMatrices(CWorldState *pWorldState,bool bConstraints)
{
	for (int module= 0; module<m_RLModuleCount; module++)
	{
		m_pRLModules[module]->UpdateQEntry(pWorldState,bConstraints);
	}
}

#define NEG_THRESHOLD -10.
int CAgent::SelectAction()
{
	int module,action;

	//initialize buffer
	for (action= 0; action<m_actionCount; action++)
	{
		m_pQBuffer[action]= 0.;
		m_pVetoBuffer[action]= false;
	}
	//ask for action preferences and update vetos
	bool bConstraint;
	int vetoCount= 0;
	int possibleActionCount;
	for (module= 0; module<m_RLModuleCount; module++)
	{
		bConstraint= m_pRLModules[module]->IsConstraint();
		Q_VALUE_TYPE* pQEntry= m_pRLModules[module]->GetQEntry(true);
		for (action= 0; action<m_actionCount; action++)
		{
			if (bConstraint && pQEntry[action]<NEG_THRESHOLD)
			{

				{
					if (!m_pVetoBuffer[action]) vetoCount++;
					m_pVetoBuffer[action]= true;
				}
			}
			else m_pQBuffer[action]+= pQEntry[action];
		}
	}
	possibleActionCount= m_actionCount - vetoCount;
	if (possibleActionCount==0)
		return 0;

	return SelectActionAmongPossibleOnes(possibleActionCount);
}

int CAgent::SelectActionAmongPossibleOnes(int possibleActionCount)
{
	int selectedAction= -1;
	int action;
	Q_VALUE_TYPE maxQ;
	if (IsProbablyTimeTo(m_epsilon))
	{
		selectedAction= SelectRandomAction(possibleActionCount);
		m_bLastActionWasRandom= true;
	}
	else
	{
		maxQ= NEG_THRESHOLD;
		for (action= 0; action<m_actionCount; action++)
		{
			if (!m_pVetoBuffer[action] && m_pQBuffer[action]>maxQ)
			{
				selectedAction= action;
				maxQ= m_pQBuffer[action];
			}
		}
		m_bLastActionWasRandom= false;
	}

	return selectedAction;
}

void CAgent::TakeAction(int actionId,CWorldState *pWorldState)
{
	int module;

	switch(actionId)
	{
	case ACTION_UP: pWorldState->m_pRobotPos[m_id].m_y++;break;
	case ACTION_RIGHT: pWorldState->m_pRobotPos[m_id].m_x++;break;
	case ACTION_LEFT: pWorldState->m_pRobotPos[m_id].m_x--;break;
	case ACTION_DOWN: pWorldState->m_pRobotPos[m_id].m_y--;break;
	case ACTION_LEFT_UP: pWorldState->m_pRobotPos[m_id].m_x--;pWorldState->m_pRobotPos[m_id].m_y++;break;
	case ACTION_LEFT_DOWN: pWorldState->m_pRobotPos[m_id].m_x--;pWorldState->m_pRobotPos[m_id].m_y--;break;
	case ACTION_RIGHT_UP: pWorldState->m_pRobotPos[m_id].m_x++;pWorldState->m_pRobotPos[m_id].m_y++;break;
	case ACTION_RIGHT_DOWN: pWorldState->m_pRobotPos[m_id].m_x++;pWorldState->m_pRobotPos[m_id].m_y--;break;
	}
	m_lastAction= actionId;
	//spread the taken action to all RL modules
	for (module= 0; module<m_RLModuleCount; module++)
		m_pRLModules[module]->SetAction(actionId);
}

bool CAgent::IsProbablyTimeTo(float probability)
{
	int randomNumber;
	probability*=1000;
	randomNumber= rand()%1001;
	if (randomNumber < (int)probability)
		return true;
	return false;
}


int CAgent::SelectRandomAction(int actionCount)
{
	int randomActionOrder;

	if (!m_bUseVetos) return rand()%m_actionCount;
	//else use vetos
	randomActionOrder= rand()%actionCount;

	int action= 0;int actionOrder= -1;
	while (randomActionOrder!=actionOrder)
	{
		if (!m_pVetoBuffer[action]) actionOrder++;
		if (randomActionOrder!=actionOrder) action++;
	}
	return action;
}

void CAgent::StartEpisode()
{
	for (int module= 0; module<m_RLModuleCount; module++)
		m_pRLModules[module]->StartEpisode();
}