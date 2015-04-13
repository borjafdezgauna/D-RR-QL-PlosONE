#ifndef __RLMANAGER__
#define __RLMANAGER__

#include "WorldState.h"


class CStatManager;

struct Q_VALUE;

#define ALPHA 0.4
#define GAMMA 0.9


struct CG_NODE
{
	int m_numChildren;
	int m_children[NUM_AGENTS];
	CG_NODE()
	{m_numChildren= 0;}
};



class MASK
{
public:
	bool bActive[NUM_AGENTS];
	MASK(){}
	~MASK(){}

	void Reset()
	{
		for (int i= 0; i<NUM_AGENTS; i++)
			bActive[i]= false;
	}
};



class JOINT_ACTION
{
public:
	char localAction[NUM_AGENTS];

	JOINT_ACTION(){} 
	~JOINT_ACTION(){}
	JOINT_ACTION (JOINT_ACTION& j_action)
	{
		for (int i= 0; i<NUM_AGENTS; i++)
			localAction[i]= j_action.localAction[i];
	}
	bool operator== (JOINT_ACTION& j_action)
	{
		for(int i= 0; i<NUM_AGENTS; i++)
			if (j_action.localAction[i]!=localAction[i]) return false;
		return true;
	}
	void Reset()
	{
		for (int i= 0; i<NUM_AGENTS; i++) localAction[i]= -1;
	}
	JOINT_ACTION Mask(MASK &mask)
	{
		JOINT_ACTION tmp;
		for (int i= 0; i<NUM_AGENTS; i++)
		{
			if (mask.bActive[i])
				tmp.localAction[i]= localAction[i];
			else tmp.localAction[i]= -1;
		}
		return tmp;
	}
	bool Agrees(JOINT_ACTION& j_action,MASK& mask)
	{
		for (int i= 0; i<NUM_AGENTS; i++)
		{
			if (mask.bActive[i] && (localAction[i]!= j_action.localAction[i]) && j_action.localAction[i]>0)
				return false;
		}
		return true;
	}
};

///COORDINATED-RL SPECIFIC STUFF
//////////////////////////////////////////////////
struct Q_VALUE
{
	int s;
	JOINT_ACTION a;
	double Q_value;
	Q_VALUE()
	{
		Q_value= 0;
	}
};
#define MAX_NUM_Q_VALUES_PER_AGENT 500000

class CRLManager
{
	int m_episodeCount;
	int m_stateId;

	Q_VALUE *m_pQValues[NUM_AGENTS];
	int m_numQValues[NUM_AGENTS];

//	bool *m_bAddedToCG;
	bool m_bDoneCG[NUM_AGENTS];
	CG_NODE m_pCG[NUM_AGENTS];

	JOINT_ACTION m_jointAction;
	JOINT_ACTION m_lastJointAction;
	MASK m_clusterMask[NUM_AGENTS];

	CWorldState *m_pWorldState;
	CStatManager *m_pStatManager;

	float m_alpha;
	float m_gamma;
	float m_epsilon;


	int m_robotCount;
	int m_stateCount;
	int m_actionCount;

	double TakeJointAction(JOINT_ACTION *pJointAction,bool& bTerminalState);
	int SelectAction(int robotId,int stateId);
	int GetStateId();
	double GetMaxQ(int robotId,int stateId);
	bool GoalReached();

	//COORDINATED-RL SPECIFIC
	//int JointAction
	void BuildCG();
	void TraverseCG(int nodeId,JOINT_ACTION *pJointAction,int state);
	void UpdateQValues(int s1,JOINT_ACTION* pJointAction, int s2, double reward);
	double GetMaxQValue(int s,int agentId);
	unsigned char GetMaxQValueActionCG(int s,int agentId);
	unsigned char GetSoftMaxAction(int s,int agentId);

//	void InitExploringStartEpisode();
	bool IsGoalReached();
//	bool IsStateTerminal();
	bool IsStateFeasible();
	bool IsThereAnyCollision();

public:
	static int m_step;
	static int m_maxStepCount;


	CRLManager();
	~CRLManager();



	//log
	void SetOutputFilenames(char *pStatsFilename,char *pLogFilename);

	void Init();
	void Release();
	void RunEpisodes(int totalEpisodeCount,char *dirname= 0);
};


#endif