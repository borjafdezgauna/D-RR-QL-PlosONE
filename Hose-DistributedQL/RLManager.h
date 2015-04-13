#ifndef __RLMANAGER__
#define __RLMANAGER__


class CWorldState;
class CStatManager;



class CRLManager
{
	int m_episodeCount;
	int m_stateId;

	double ***m_pQMatrices;


	CWorldState *m_pWorldState;
	CStatManager *m_pStatManager;

	float m_alpha;
	float m_gamma;
	float m_epsilon;


	int m_robotCount;
	int m_stateCount;
	int m_actionCount;

	double TakeAction(int agentId,int actionId,bool& bTerminal);
	double TakeJointAction(int *localAction,bool& bTerminalState);
	int SelectAction(int robotId,int stateId);
	int GetStateId();
	double GetMaxQ(int robotId,int stateId);
	bool GoalReached();

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
	void RunEpisodes(int totalEpisodeCount,char* dirname);
};


#endif