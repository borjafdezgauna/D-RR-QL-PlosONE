// LeaderFollowerLearner.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "RLManager.h"
#include "ParameterManager.h"


int _tmain(int argc, _TCHAR* argv[])
{
	CRLManager RLManager;
	g_pParameterManager= new CParameterManager();

	char outStatsFile[256];
	char outLogFile[256];

	
	char dirname[256];
	

	if (argc>1)
	{
		sprintf_s(dirname,256,"%S",argv[1]);
	}
	else
	{
		sprintf_s(dirname,256,"experiments");
	}
	
	char configFile[256];

	sprintf_s(configFile,256,"%s/parameters.txt",dirname);
	g_pParameterManager->LoadParameters(configFile);


	printf ("\nLearning process begins...\n");
	RLManager.Init();

	sprintf_s(outStatsFile,256,"%s/stats.txt",dirname);
	sprintf_s(outLogFile,256,"%s/log.txt",dirname);
	RLManager.SetOutputFilenames(outStatsFile,outLogFile);


	RLManager.RunEpisodes(g_pParameterManager->GetIntParameter("EPISODE_COUNT"),dirname);

	delete g_pParameterManager;

	return 0;
}

