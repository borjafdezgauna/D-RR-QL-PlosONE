#include "stdafx.h"
#include "GoalReachModule.h"
#include "WorldState.h"
#include "GridDistances.h"
#include "ParameterManager.h"

///////////////////
//CDistKeepState
///////////////////
/*
CGoalReachState::CGoalReachState()
{
}

CGoalReachState::~CGoalReachState()
{
}*/

int CGoalReachState::GetStateId()
{
	int stateId= GetRelPosIndex(m_relGoalPosX,m_relGoalPosY,g_absPosCountInRange);

	m_stateId= stateId;

	return stateId;
}

void CGoalReachState::StateFromStateId(int stateId)
{
}

void CGoalReachState::GenerateStateTransition(IStateView* pState,int action){}

void CGoalReachState::CopyFrom(IStateView *pNewState)
{
	m_relGoalPosX= ((CGoalReachState*)pNewState)->m_relGoalPosX;
	m_relGoalPosY= ((CGoalReachState*)pNewState)->m_relGoalPosY;

}

int CGoalReachState::GetStateSpace()
{
	//		goal-x/y	
	return g_absPosCountInRange;
}



///////////////////
//CDistKeepModule
///////////////////
CGoalReachModule::CGoalReachModule()
{
	m_moduleId= GOAL_REACH_MODULE_ID;
	m_pState= new CGoalReachState;
	m_pNextState= new CGoalReachState;
	m_bGenericModule= false;
	m_bConstraintModule= false;

	m_alpha= g_pParameterManager->GetDoubleParameter("GOAL_REACH_ALPHA"); // step-size parameter. How fast do we want the system to learn?
	m_gamma= g_pParameterManager->GetDoubleParameter("GOAL_REACH_GAMMA"); // discount rate. Present value of future rewards.
}

CGoalReachModule::~CGoalReachModule()
{
	delete m_pState;
	delete m_pNextState;
}

REWARD_TYPE CGoalReachModule::GetReward(CWorldState *pWorldState)
{
	//goal reached?
	if (pWorldState->m_pRobotPos[0].m_x==pWorldState->m_goal.m_x
		&& pWorldState->m_pRobotPos[0].m_y==pWorldState->m_goal.m_y)
	{
		m_terminationCode= 0;
		m_bStateIsTerminal= true;
		return m_positiveReward;
	}

	
	m_bStateIsTerminal= false;
	return m_neutralReward;
}

void CGoalReachModule::FilterWorldState(CWorldState *pWorldState,bool bCurrentState)
{
	CGoalReachState *pState;
	
	pState= (CGoalReachState*) GetStateHandle(bCurrentState);

	pState->m_relGoalPosX= pWorldState->m_goal.m_x - pWorldState->m_pRobotPos[m_id].m_x;
	pState->m_relGoalPosY= pWorldState->m_goal.m_y - pWorldState->m_pRobotPos[m_id].m_y;;

}