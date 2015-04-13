#ifndef __GOAL_REACH_MODULE__
#define __GOAL_REACH_MODULE__

#include "RLModule.h"


class CGoalReachState:public IStateView
{
public:
	//goal reaching info
	//v2
	int m_relGoalPosX,m_relGoalPosY;

	int GetStateId();
	void CopyFrom(IStateView *pNewState);
	int GetStateSpace();
	void StateFromStateId(int stateId);
	void GenerateStateTransition(IStateView* pState,int action);
};


class CGoalReachModule:public CRLModule
{
public:
	CGoalReachModule();
	~CGoalReachModule();

	REWARD_TYPE GetReward(CWorldState *pWorldState);
	void FilterWorldState(CWorldState *pWorldState,bool currentState);
};

#endif