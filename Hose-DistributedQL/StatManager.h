#ifndef __STAT_MANAGER__
#define __STAT_MANAGER__

class CWorldState;

struct TerminationInfo
{
	bool bSuccesful;
	int stepCount;
	int robotId;
	int moduleId;
};

struct LogEpisodeStart
{
	short blockSize;
	int episodeId;
	int dataSize;
	short actionCount;
	bool succesful;
	short blamedRobotId;
	short blamedModuleId;
	short worldStateSize;
};

struct LogPosition
{
	short x,y;
};


struct LogAction
{
	unsigned short step;
	unsigned short robotId;
	unsigned short actionId;
};



class CStatManager
{
	TerminationInfo *m_pData;
	int *m_pRobotBlameCount;
	int m_robotCount;
	int *m_pModuleBlameCount;
	int m_moduleCount;
	int m_dataCount;
	int m_windowSize;

	//general stats
	char m_statsFilename[256];
	void* m_pStatsFile;
	int m_totalDataCount;

	float m_lastSuccessRate;

	//complete logs
	char m_episodeLogFilename[256];
	void* m_pEpisodeLogFile;
	int m_step;
	int m_actionCount;
	int m_stepLimit;
	long m_lastEpisodeOffset;

	void AddFailedEpisode(int blamedModuleId,int blamedRobotId);
	void AddForcedToFinishEpisode();
	void AddSuccesfulEpisode();

public:
	CStatManager();
	~CStatManager();

	//general stats
	void SetOutputFilenames(char *pStatFilename,char *pLogFilename);
	void InitDataWindow(int size);
	void ResetStats();
	void CloseStats();
	void PrintStats();
	void LogAction(int robotId,int action,int step);
	void LogEpisodeEnd(bool succesful,int blamedRobot= -1,int blamedModule= -1);
	void LogEpisodeStart(int episode,CWorldState *pWorldState);
	float GetSuccessRate(){return m_lastSuccessRate;}
};


#endif