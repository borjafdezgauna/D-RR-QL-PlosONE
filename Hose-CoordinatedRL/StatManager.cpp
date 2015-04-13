#include "stdafx.h"
#include "StatManager.h"
#include "GlobalDefines.h"
#include "ParameterManager.h"
#include "WorldState.h"
#include "AuxFunctions.h"

#define FORCED_TO_FINISH -1

CStatManager::CStatManager()
{
	m_windowSize= 0;
	m_pData= 0;
	m_pRobotBlameCount= 0;
	m_robotCount= 0;
	m_pModuleBlameCount= 0;
	m_moduleCount= 0;
	m_dataCount= 0;
	m_totalDataCount= 0;
	m_step= 0;
	m_pStatsFile= 0;
	m_pEpisodeLogFile= 0;
	m_lastSuccessRate= 0.;
}

CStatManager::~CStatManager()
{
	if (m_pData)
	{
		delete [] m_pData;
		m_pData= 0;
	}
	if (m_pRobotBlameCount)
	{
		delete [] m_pRobotBlameCount;
		m_pRobotBlameCount= 0;
	}
	if (m_pModuleBlameCount)
	{
		delete [] m_pModuleBlameCount;
		m_pModuleBlameCount= 0;
	}
}

void CStatManager::ResetStats()
{
	m_dataCount= 0;
	m_totalDataCount= 0;
	m_step= 0;
	m_actionCount= 0;
	m_stepLimit= g_pParameterManager->GetIntParameter("MAX_EPISODE_STEP_COUNT");
	fopen_s((FILE**)&m_pStatsFile,m_statsFilename,"w");
	fopen_s((FILE**)&m_pEpisodeLogFile,m_episodeLogFilename,"wb+");
}

void CStatManager::CloseStats()
{
	fclose((FILE*)m_pStatsFile);
	fclose((FILE*)m_pEpisodeLogFile);
}

void CStatManager::SetOutputFilenames(char *pStatsFilename,char *pLogFilename)
{
	strcpy_s(m_statsFilename,256,pStatsFilename);
	strcpy_s(m_episodeLogFilename,256,pLogFilename);
}

void CStatManager::InitDataWindow(int size)
{
	if (m_pData)
	{
		delete [] m_pData;
		m_pData= 0;
	}
	m_pData= new TerminationInfo[size];
	
	if (m_pRobotBlameCount)
	{
		delete [] m_pRobotBlameCount;
		m_pRobotBlameCount= 0;
	}
	m_robotCount= g_pParameterManager->GetIntParameter("ROBOT_COUNT");
	m_pRobotBlameCount= new int[m_robotCount];
	if (m_pModuleBlameCount)
	{
		delete [] m_pModuleBlameCount;
		m_pModuleBlameCount= 0;
	}
	m_moduleCount= g_pParameterManager->GetIntParameter("RL_MODULE_COUNT");
	m_pModuleBlameCount= new int[m_moduleCount];

	m_windowSize= size;	
	m_dataCount= 0;
}

void CStatManager::AddFailedEpisode(int blamedModuleId,int blamedRobotId)//TerminationInfo &termInfo)
{
	int index;
	index= m_dataCount%m_windowSize;
	m_pData[index].bSuccesful= false;//termInfo;
	m_pData[index].moduleId= blamedModuleId;
	m_pData[index].stepCount= 0;
	m_pData[index].robotId= blamedRobotId;
	m_dataCount++;
	m_totalDataCount++;
}

void CStatManager::AddForcedToFinishEpisode()
{
	int index;
	index= m_dataCount%m_windowSize;
	m_pData[index].bSuccesful= false;//termInfo;
	m_pData[index].moduleId= FORCED_TO_FINISH;
	m_pData[index].stepCount= m_step;
	m_pData[index].robotId= 0;
	m_dataCount++;
	m_totalDataCount++;
}

void CStatManager::AddSuccesfulEpisode()
{
	int index;
	index= m_dataCount%m_windowSize;
	m_pData[index].bSuccesful= true;
	m_pData[index].moduleId= 0;
	m_pData[index].stepCount= m_step;
	m_pData[index].robotId= 0;
	m_dataCount++;
	m_totalDataCount++;
}

void CStatManager::PrintStats()
{
	int entryCount= 0;
	int succesfulCount= 0;
	int failedCount= 0;
	int forcedToFinishCount= 0;
	int steps= 0;

	if (m_dataCount<m_windowSize)
		return;

	//BLAMED MODULES
	for (int module= 0; module<m_moduleCount; module++)
		m_pModuleBlameCount[module]= 0;
	//BLAMED ROBOTS
	for (int robot= 0; robot<m_robotCount; robot++)
		m_pRobotBlameCount[robot]= 0;

	for (int i= 0; i<m_windowSize; i++)
	{
		if (m_pData[i].bSuccesful)
		{
			succesfulCount++;
			steps+= m_pData[i].stepCount;
		}
		else
		{
			failedCount++;
			if (m_pData[i].moduleId==FORCED_TO_FINISH) forcedToFinishCount++;
			else
			{
				m_pRobotBlameCount[m_pData[i].robotId]++;
				m_pModuleBlameCount[m_pData[i].moduleId]++;
			}
		}
	}

	FILE* pFile= (FILE*)m_pStatsFile;

	//episodeCount
	fprintf(pFile,"%d,",m_totalDataCount-1);
	//successful
	m_lastSuccessRate= ((float)succesfulCount)/(float)m_windowSize;
	fprintf(pFile,"%.2f,",m_lastSuccessRate);
	//average steps
	float avgSteps;
	if (succesfulCount) avgSteps= ((float)steps)/(float)succesfulCount;
	else avgSteps= 0.f;
	fprintf(pFile,"%.2f,",avgSteps);
	if (failedCount)
	{
		/*	//failed
		fprintf(pFile,"%.2f,",((float)failedCount)/(float)m_windowSize);*/
		//step limit reached
		float forced;
		if (failedCount) forced= ((float)forcedToFinishCount)/(float)m_windowSize;
		else forced= 0.f;
		fprintf(pFile,"%.2f",forced);


		//BLAMED MODULES
		for (int module= 0; module<m_moduleCount; module++)
			fprintf(pFile,",%.2f",(float)m_pModuleBlameCount[module]/(float)m_windowSize);
		//BLAMED ROBOTS
		for (int robot= 0; robot<m_robotCount; robot++)
			fprintf(pFile,",%.2f",(float)m_pRobotBlameCount[robot]/(float)m_windowSize);
	}

	fprintf(pFile,"\n");
}


/*
void CStatManager::LogAction(int robotId,int action,int step)
{
	struct LogAction logAction;
	/*char actionName[256];
	GetActionName(action,actionName,256);
	m_actionCount++;
	m_step= step;
	logAction.step= step;
	logAction.robotId= robotId;
	logAction.actionId= action;
	fwrite((void*)&logAction,sizeof(logAction),1,(FILE*)m_pEpisodeLogFile);
	//fprintf((FILE*)m_pEpisodeLogFile,"R#%d Action:%s\n",robotId,actionName);
}
/*
void CStatManager::LogEpisodeStart(int episodeId,CWorldState *pWorldState)
{
	struct LogEpisodeStart logStart;
	struct LogPosition logPos;

	m_lastEpisodeOffset= ftell((FILE*)m_pEpisodeLogFile);



	//fprintf((FILE*)m_pEpisodeLogFile,"EPISODE %d BEGINS\n",episodeId);

	//INITIAL WORLD STATE
	logStart.episodeId= episodeId;
	logStart.worldStateSize= (m_robotCount+1) * sizeof(logPos);
	logStart.blockSize= sizeof(logStart) + logStart.worldStateSize;
	
	fwrite(&logStart,sizeof(logStart),1,(FILE*)m_pEpisodeLogFile);
	//goal
	logPos.x= (short)pWorldState->m_goal.m_x;
	logPos.y= (short)pWorldState->m_goal.m_y;
	fwrite(&logPos,sizeof(logPos),1,(FILE*)m_pEpisodeLogFile);
	for (int i= 0; i<m_robotCount; i++)
	{
		logPos.x= (short)pWorldState->m_pRobotPos[i].m_x;
		logPos.y= (short)pWorldState->m_pRobotPos[i].m_y;
		fwrite(&logPos,sizeof(logPos),1,(FILE*)m_pEpisodeLogFile);
	}

	/*
	//goal
	fprintf((FILE*)m_pEpisodeLogFile,"Goal [%d,%d]",pWorldState->m_goal.m_x,pWorldState->m_goal.m_y);

	//robots
	int robotCount= g_pParameterManager->GetIntParameter("ROBOT_COUNT");
	for (int i= 0; i<robotCount; i++)
	{
		fprintf((FILE*)m_pEpisodeLogFile," R#%d [%d,%d]",i,pWorldState->m_pRobotPos[i].m_x
			,pWorldState->m_pRobotPos[i].m_y);
	}
	fprintf((FILE*)m_pEpisodeLogFile,"\n");
}*/
/*
void CStatManager::LogEpisodeEnd(bool succesful,int blamedRobot,int blamedModule)
{
	struct LogEpisodeStart logStart;
	//reconstruct episode header
	long currentOffset= ftell((FILE*)m_pEpisodeLogFile);
	fseek((FILE*)m_pEpisodeLogFile,m_lastEpisodeOffset,SEEK_SET);
	fread(&logStart,sizeof(logStart),1,(FILE*)m_pEpisodeLogFile);
	logStart.actionCount= m_actionCount;
	logStart.blamedModuleId= blamedModule;
	logStart.blamedRobotId= blamedRobot;
	logStart.succesful= succesful;
	logStart.dataSize= logStart.blockSize + m_actionCount*sizeof(struct LogAction);
	m_actionCount= 0;
	fseek((FILE*)m_pEpisodeLogFile,m_lastEpisodeOffset,SEEK_SET);
	fwrite(&logStart,sizeof(logStart),1,(FILE*)m_pEpisodeLogFile);
	fseek((FILE*)m_pEpisodeLogFile,currentOffset,SEEK_SET);

	if (succesful)
	{
		AddSuccesfulEpisode();
		//fprintf((FILE*)m_pEpisodeLogFile,"SUCCESFUL EPISODE. GOAL REACHED");
	}
	else if (m_step>=m_stepLimit)
	{
		AddForcedToFinishEpisode();
		/*fprintf((FILE*)m_pEpisodeLogFile,"FAILED EPISODE. %d steps. Blamed robot= %d, blamed module= %d"
			,m_step,blamedRobot,blamedModule);
	}
	else
	{
		AddFailedEpisode(blamedModule,blamedRobot);
		/*fprintf((FILE*)m_pEpisodeLogFile,"FAILED EPISODE. %d steps. Blamed robot= %d, blamed module= %d"
			,m_step,blamedRobot,blamedModule);
	}
}*/
