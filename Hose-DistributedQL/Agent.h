#ifndef __AGENT__
#define __AGENT__


class CWorldState;
class CRLModule;

class CAgent
{
	CRLModule **m_pRLModules;
	int m_RLModuleCount;

	float m_epsilon;

	int m_id;

	int m_actionCount;
	int m_lastAction;
	bool m_bLastActionWasRandom;
	bool m_bUseVetos;

	//TerminationInfo m_terminationInfo;

	Q_VALUE_TYPE *m_pQBuffer;
	bool *m_pVetoBuffer;

	bool IsProbablyTimeTo(float probability); //probability: [0..1]
	int SelectRandomAction(int actionCount);
	int SelectAction();
	int SelectActionAmongPossibleOnes(int possibleActionCount);
	void TakeAction(int actionId,CWorldState *pWorldState);

	void UpdateQMatrices(CWorldState *pWorldState,bool bConstraints);

	void FilterState(CWorldState *pWorldState, bool currentState);
public:
	CAgent();
	~CAgent();

	bool IsStateTerminal(int &blamedModuleId,int &blamedRobotId);//TerminationInfo& termInfo);
	bool IsStateValid();
	void Init(int id);
	int TimeStep(CWorldState *pWorldState); //returns the action taken
	void SetEpsilon(float epsilon);
	void SaveQMatrices(char* pFilenameFormat);
	void LoadQMatrices(char* pFilenameFormat);
	void StartEpisode();
};

#endif