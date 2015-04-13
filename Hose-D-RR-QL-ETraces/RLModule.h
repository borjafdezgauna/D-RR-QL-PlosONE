#ifndef __RL_MODULE__
#define __RL_MODULE__

#include "GlobalDefines.h"

class CWorldState;
class CQMatrix;

class IStateView
{
public:

	int m_stateId;
	virtual int GetStateId()= 0;
	virtual void CopyFrom(IStateView *pNewState)= 0;
	virtual int GetStateSpace()= 0;
	virtual void StateFromStateId(int stateId)= 0;
	virtual void GenerateStateTransition(IStateView* pState,int action)= 0;
};


class CRLModule
{
public:
	CQMatrix *m_pQMatrix;
	Q_VALUE_TYPE *m_pDummyQEntry;
	IStateView *m_pState;
	IStateView *m_pNextState;
	
	bool m_bStateIsTerminal;
	int m_terminationCode;
	int m_lastAction;
	double m_lastReward;
	int m_lastStateId;
	int m_id;
	int m_moduleId;

	static double m_negativeReward;
	static double m_positiveReward;
	static double m_neutralReward;

	static int m_robotCount;
	static double m_hoseSegmentLength;
	static int m_gridSizeX;
	static int m_gridSizeY;
	static int m_gridMaxX;
	static int m_gridMaxY;
	static int m_maxAbsDist;
	static int m_actionCount;
	
	double m_alpha;
	double m_gamma;
	double m_epsilon;
	
	bool m_bGenericModule;
	bool m_bConstraintModule;

	CRLModule();
	~CRLModule();

	bool IsConstraint(){return m_bConstraintModule;}

	void SetId(int id);
	int GetStateSpace();
	void StateFromStateId(int stateId,bool bCurrentState);
	void GenerateStateTransition(int action);

	IStateView *GetStateHandle(bool bCurrentState) {if (bCurrentState) return m_pState;else return m_pNextState;}

	void Init();
	Q_VALUE_TYPE* GetQEntry(bool bCurrentState);
	bool IsStateValid(bool bCurrentState);
	void UpdateQEntry(CWorldState *pWorldState,bool bInmediateReward);
	static int GetMaxQAction(Q_VALUE_TYPE *pQEntry);
	void UpdateState();
	void SetAction(int actionId);
	void StartEpisode(){m_lastStateId= -1;};

	virtual REWARD_TYPE GetReward(CWorldState *pWorldState)= 0;
	virtual void FilterWorldState(CWorldState *pWorldState,bool currentState)= 0;
	bool IsStateTerminal(int &terminationCode);
	//virtual double GetFactor();

	void SetEpsilon(float epsilon);

	void SaveQMatrix(char *pFilenameFormat);
	void LoadQMatrix(char *pFilenameFormat);
	int GetModuleId(){return m_moduleId;}
};

#endif