#include "stdafx.h"
#include "RLManager.h"

#include "StatManager.h"
#include "WorldState.h"

#include "ParameterManager.h"
//HOSE MOD//////////////////
#include "GridDistances.h"
//////////////////////////

int CRLManager::m_step= 0;
int CRLManager::m_maxStepCount= 0;


CRLManager::CRLManager()
{
	srand(1);
	m_stateId= 0;
	m_epsilon= 0.1f; //exploration - explotation
}

CRLManager::~CRLManager()
{
}

void CRLManager::SetOutputFilenames(char* pStatsFilename,char* pLogFilename)
{
	//m_pStatManager->SetOutputFilenames(pStatsFilename,pLogFilename);
}

//HOSE MOD///////////////////////////////

#define NUM_STATES_PER_AGENT CountRelPosInRange(MAX_REL_DIST)
#define NUM_STATES int(pow(NUM_STATES_PER_AGENT,(float)NUM_AGENTS))
#define NUM_ACTIONS_PER_AGENT 5
#define NUM_ACTIONS int(pow(NUM_ACTIONS_PER_AGENT,(float)NUM_AGENTS))
////////////////////////////////////////

struct tipPos *pVVStates;
int g_numVVStates=0;
struct tipPos
{
	int x,y;
};
void AddState(int x,int y)
{
	int i;
	for (i= 0; i<g_numVVStates && (pVVStates[i].x!=x || pVVStates[i].y!=y); i++);
	
	if (i>=g_numVVStates)
	{
		pVVStates[g_numVVStates].x= x;
		pVVStates[g_numVVStates].y= y;
		g_numVVStates++;
	}
}

void CRLManager::Init()
{
//HOSE MOD/////////////////////////
	InitDistMatrix(MAX_REL_DIST+1,MAX_REL_DIST+1,MAX_REL_DIST,true);
///////////////////////////////////
	m_robotCount= g_pParameterManager->GetIntParameter("ROBOT_COUNT");
	m_actionCount= NUM_ACTIONS;//g_pParameterManager->GetIntParameter("ACTION_COUNT");
	m_epsilon= (float)g_pParameterManager->GetDoubleParameter("TRAINING_EPSILON");
	m_stateCount= NUM_STATES;//NUM_STATES_PER_AGENT*NUM_STATES_PER_AGENT*NUM_STATES_PER_AGENT*NUM_STATES_PER_AGENT;

	m_pWorldState= new CWorldState();

	m_pQMatrices= new double **[m_robotCount];//[NUM_STATES][NUM_ACTIONS]

	for (int i= 0; i<m_robotCount; i++)
	{	
		m_pQMatrices[i]= new double*[NUM_STATES];

		for (int j= 0; j<NUM_STATES; j++)
		{
			m_pQMatrices[i][j]= new double[NUM_ACTIONS_PER_AGENT];
			for(int k=0; k<NUM_ACTIONS_PER_AGENT; k++)
			{
				m_pQMatrices[i][j][k]= 0.;
			}
		}
			
	}

	//random function
	srand(1);


	//m_pStatManager= new CStatManager();
	//m_pStatManager->InitDataWindow(g_pParameterManager->GetIntParameter("STAT_WINDOW_SIZE"));
}

void CRLManager::Release()
{
	delete m_pWorldState;
	//delete m_pStatManager;

}

int CRLManager::GetStateId()
{
	int aux= 1;
	int stateId = 0;
//HOSE MOD///////////////////////////////////
	int prevX= 0, prevY= 0;
	int posCount= CountRelPosInRange(MAX_REL_DIST);
	for (int i= 0; i<NUM_AGENTS; i++)
	{
		
		int index= GetRelPosIndex(m_pWorldState->m_positions[i].m_x-prevX
			,m_pWorldState->m_positions[i].m_y-prevY
			,posCount);
		if (index<0) return -1;
		stateId+= index*aux;
		prevX= m_pWorldState->m_positions[i].m_x;
		prevY= m_pWorldState->m_positions[i].m_y;
/////////////////////////////////////////////////
		aux*= NUM_STATES_PER_AGENT;
	}
	return stateId;
}

int CRLManager::SelectAction(int agentId,int stateId)
{
		double max;
	int maxA;
	int randomNumber;
	double probs[NUM_ACTIONS_PER_AGENT];
	int vetoes[NUM_ACTIONS_PER_AGENT];
	double sum= 0.;
	if (m_epsilon>0.00001 && g_pParameterManager->GetIntParameter("USE_BOLTZMANN"))
	{
		double temp= 0.2;//m_epsilon;
		for (int i= 0; i<NUM_ACTIONS_PER_AGENT; i++)
		{
			sum+= exp((m_pQMatrices[agentId][stateId][i])/temp);
		}
		sum= max(sum,0.001);
		for (int i= 0; i<NUM_ACTIONS_PER_AGENT; i++)
		{
			probs[i]= 100*exp((m_pQMatrices[agentId][stateId][i])/temp)/sum;
		}
		int rn= rand()%101;
		int j= 0;
		double probsum= probs[0];
		while (j<NUM_ACTIONS_PER_AGENT && rn>=probsum)
		{
			j++;
			probsum+=probs[j];
		}
		return min(j,NUM_ACTIONS_PER_AGENT-1);
	}

	int probability=(int)(m_epsilon*1000);
	randomNumber= rand()%1001;
	if (randomNumber < (int)probability)
		return rand()%NUM_ACTIONS_PER_AGENT;

	max= m_pQMatrices[agentId][stateId][0];
	maxA= 0;
	for (int i=1; i<NUM_ACTIONS_PER_AGENT; i++)
	{
		if (m_pQMatrices[agentId][stateId][i]>max)
		{
			max= m_pQMatrices[agentId][stateId][i];
			maxA= i;
		}
	}
	return maxA;
/*	double max;
	int maxA;

	int randomNumber;
	int probability=(int)(m_epsilon*1000);
	randomNumber= rand()%1001;
	if (randomNumber < (int)probability)
		return rand()%NUM_ACTIONS_PER_AGENT;


	max= m_pQMatrices[agentId][stateId][0];
	maxA= 0;
	for (int i=1; i<NUM_ACTIONS_PER_AGENT; i++)
	{
		if (m_pQMatrices[agentId][stateId][i]>max)
		{
			max= m_pQMatrices[agentId][stateId][i];
			maxA= i;
		}
	}
	return maxA;*/
}

double CRLManager::GetMaxQ(int agentId,int stateId)
{
	double max;
	if (stateId<0) return 0;

	max= m_pQMatrices[agentId][stateId][0];
	for (int i=1; i<NUM_ACTIONS_PER_AGENT; i++)
	{
		if (m_pQMatrices[agentId][stateId][i]>max)
		{
			max= m_pQMatrices[agentId][stateId][i];
		}
	}
	return max;
}

bool CRLManager::GoalReached()
{
	return (m_pWorldState->m_positions[NUM_AGENTS-1].m_x==g_pParameterManager->GetIntParameter("GOAL_X")
		&& m_pWorldState->m_positions[NUM_AGENTS-1].m_y==g_pParameterManager->GetIntParameter("GOAL_Y"));
}

double CRLManager::TakeJointAction(int* localAction,bool& bTerminalState)
{
	int destPosX[NUM_AGENTS];
	int destPosY[NUM_AGENTS];
	double reward= 0;
	double negReward,posReward,neuReward;
	int nCollisions= 0;


////HOSE MOD/////////////////////////////////////////
	negReward= g_pParameterManager->GetDoubleParameter("REWARD_NEGATIVE");
	posReward= g_pParameterManager->GetDoubleParameter("REWARD_POSITIVE");
	neuReward= g_pParameterManager->GetDoubleParameter("REWARD_NEUTRAL");

	for (int i= 0; i<NUM_AGENTS; i++)
	{
		destPosX[i]=-1;
		destPosY[i]=-1;
		//1. destPosX & destPosY?
		if (localAction[i]==ACTION_UP)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x;
			destPosY[i]= m_pWorldState->m_positions[i].m_y+1;
		}
		else if (localAction[i]==ACTION_DOWN)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x;
			destPosY[i]= m_pWorldState->m_positions[i].m_y-1;
		}
		else if (localAction[i]==ACTION_RIGHT)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x+1;
			destPosY[i]= m_pWorldState->m_positions[i].m_y;
		}
		else if (localAction[i]==ACTION_LEFT)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x-1;
			destPosY[i]= m_pWorldState->m_positions[i].m_y;
		}
		else if (localAction[i]==ACTION_LEFT_UP)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x-1;
			destPosY[i]= m_pWorldState->m_positions[i].m_y+1;
		}
		else if (localAction[i]==ACTION_RIGHT_UP)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x+1;
			destPosY[i]= m_pWorldState->m_positions[i].m_y+1;
		}
		else if (localAction[i]==ACTION_LEFT_DOWN)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x-1;
			destPosY[i]= m_pWorldState->m_positions[i].m_y-1;
		}
		else if (localAction[i]==ACTION_RIGHT_DOWN)
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x+1;
			destPosY[i]= m_pWorldState->m_positions[i].m_y-1;
		}
		else
		{
			destPosX[i]= m_pWorldState->m_positions[i].m_x;
			destPosY[i]= m_pWorldState->m_positions[i].m_y;
		}
	}
	//2. collisions against walls?
	for (int i= 0; i<NUM_AGENTS; i++)
	{
		if ( destPosX[i]<-9 || destPosX[i]>9 || destPosY[i]<-9 || destPosY[i]>9 )
		{
			nCollisions++;
			destPosX[i]= m_pWorldState->m_positions[i].m_x;
			destPosY[i]= m_pWorldState->m_positions[i].m_y;
		}
	}
	//3. collisions against not moving robots?
	for (int i= 0; i<NUM_AGENTS; i++)
	{
		for (int j= 0; j<NUM_AGENTS; j++)
		{
			if (i!=j && destPosX[i]==destPosX[j] && destPosY[i]==destPosY[j]
			&& destPosX[j]==m_pWorldState->m_positions[j].m_x
			&& destPosY[j]==m_pWorldState->m_positions[j].m_y)
			{
				nCollisions++;
				destPosX[i]= m_pWorldState->m_positions[i].m_x;
				destPosY[i]= m_pWorldState->m_positions[i].m_y;
			}
		}
	}

	//4. crossings and common destination??
	for (int a1= 0; a1<NUM_AGENTS; a1++)
	{
		for (int a2= a1+1; a2<NUM_AGENTS; a2++)
		{
			//crossing
			if ( destPosX[a1]==m_pWorldState->m_positions[a2].m_x
				&& destPosY[a1]==m_pWorldState->m_positions[a2].m_y
				&& destPosX[a2]==m_pWorldState->m_positions[a1].m_x
				&& destPosY[a2]==m_pWorldState->m_positions[a1].m_y )
			{
				destPosX[a1]= m_pWorldState->m_positions[a1].m_x;
				destPosY[a1]= m_pWorldState->m_positions[a1].m_y;

				destPosX[a2]= m_pWorldState->m_positions[a2].m_x;
				destPosY[a2]= m_pWorldState->m_positions[a2].m_y;
				nCollisions++;
			}
			else if ( (destPosX[a1]==destPosX[a2] && destPosY[a1]==destPosY[a2]) )
			{
				//destPosX[a1]= m_pWorldState->m_positions[a1].m_x;
				//destPosY[a1]= m_pWorldState->m_positions[a1].m_y;

				destPosX[a2]= m_pWorldState->m_positions[a2].m_x;
				destPosY[a2]= m_pWorldState->m_positions[a2].m_y;
				nCollisions++;
			}
		}
	}
	//hose stretched???
	int prevX= 0,prevY= 0;
	bool bStretched= true;
//	while (bStretched)
	{
		for (int i= 0; i<NUM_AGENTS; i++)
		{

			int distPrev, distNext1= 0, distNext2= 0;
			distPrev= (prevX-destPosX[i])*(prevX-destPosX[i]) + (prevY-destPosY[i])*(prevY-destPosY[i]);

			if (i<NUM_AGENTS-1)
			{
				distNext1= (destPosX[i]-destPosX[i+1])*(destPosX[i]-destPosX[i+1])
				+ (destPosY[i]-destPosY[i+1])*(destPosY[i]-destPosY[i+1]);
				distNext2= (destPosX[i]-m_pWorldState->m_positions[i+1].m_x)
				*(destPosX[i]-m_pWorldState->m_positions[i+1].m_x)
				+ (destPosY[i]-m_pWorldState->m_positions[i+1].m_y)
				*(destPosY[i]-m_pWorldState->m_positions[i+1].m_y);
			}
			if ( distPrev>MAX_REL_DIST*MAX_REL_DIST 
				|| (distNext1>MAX_REL_DIST*MAX_REL_DIST  && distNext2>MAX_REL_DIST*MAX_REL_DIST) )
			{
				nCollisions++;
			/*	destPosX[i]= m_pWorldState->m_positions[i].m_x;
				destPosY[i]= m_pWorldState->m_positions[i].m_y;*/
			}
			prevX= destPosX[i];
			prevY= destPosY[i];
		}
		//re-check if the hose is stretched
	/*	bStretched= false;
		prevX= 0; prevY= 0;
		for (int i= 0; i<NUM_AGENTS; i++)
		{
			int dist = (destPosX[i]-prevX)*(destPosX[i]-prevX) + (destPosY[i]-prevY)*(destPosY[i]-prevY);
			if (dist>MAX_REL_DIST*MAX_REL_DIST)
				bStretched= true;
			prevX= destPosX[i];
			prevY= destPosY[i];
		}*/

	}
	

	//update world state
	for (int i= 0; i<NUM_AGENTS; i++)
	{
		m_pWorldState->m_positions[i].m_x= destPosX[i];
		m_pWorldState->m_positions[i].m_y= destPosY[i];
	}


	if (GoalReached())
	{
		bTerminalState= true;
		return posReward;
	}
	if (nCollisions)
	{
		bTerminalState= true;
		reward= nCollisions*negReward;
	}
	else reward= neuReward;
///////////////////////////////////////////////////////////

	return reward;

}

#define ALPHA 0.4
#define GAMMA 0.9



void CRLManager::RunEpisodes(int totalEpisodeCount,char* dirname)
{
	float epsilon= 1.;
	int robot;
	FILE *pOutFile, *pOutFile2;
	bool bGreedy;
	int totalMoveCount= 0;
	int firstGoal=-1;

	m_maxStepCount= g_pParameterManager->GetIntParameter("MAX_EPISODE_STEP_COUNT");

	pVVStates= new struct tipPos[NUM_STATES];

	printf("Simulation started\n");

	//m_pStatManager->ResetStats();
for (int run= 0; run<3; run++)
{
	srand(run);
	double epsilon_delta= 1./ (double)totalEpisodeCount;
		char logFile[256];
		char greedyLogFile[256];
		sprintf_s(logFile,256,"%s/Hose-D-RR-QL.Log.%d.txt",dirname,run);//epsilon_delta);
		sprintf_s(greedyLogFile,256,"%s/Hose-D-RR-QL.GreedyLog.%d.txt",dirname,run);//,epsilon_delta);
		fopen_s(&pOutFile,logFile,"w");
		fopen_s(&pOutFile2,greedyLogFile,"w");
bool bGoalReached= false;
int bestGreedyPolicy= 0;
int firstGoal=-1;

	totalMoveCount= 0;
	epsilon= (float)g_pParameterManager->GetDoubleParameter("TRAINING_EPSILON");
g_numVVStates= 0;
	for (int i= 0; i<m_robotCount; i++)
	{	
		for (int j= 0; j<NUM_STATES; j++)
		{
			for(int k=0; k<NUM_ACTIONS_PER_AGENT; k++)
			{
				m_pQMatrices[i][j][k]= 0.;
			}
		}			
	}

	for (m_episodeCount= 0; m_episodeCount<totalEpisodeCount; m_episodeCount++)
	{
		m_step= 0;


		bool bTerminalState= false;
		int blamedModuleId= -1;
		int blamedRobotId= -1;
		int actionTaken= -1;
		int lastAction[NUM_AGENTS];
		int lastState[NUM_AGENTS];
		double rewards[NUM_AGENTS];
		double newQ;
		int localActions[NUM_AGENTS];

		double lastReward= 0.;
		int s=-1;
		double maxQ= 0.;
		int totalMoves= 0;
		for (int i= 0; i<NUM_AGENTS; i++)
		{
			lastAction[i]=-1;
			lastState[i]= -1;
			rewards[i]= 0;
		}

		double totalRewards= 0.0;

		if (m_episodeCount%500==0 && m_episodeCount>0)
		{
			m_epsilon= 0.;
			bGreedy= true;
		}
		else
		{
			m_epsilon= epsilon;
			bGreedy= false;
		}

		m_pWorldState->Init();

		while ((!bGreedy || m_step<120) && (bGreedy || m_step<m_maxStepCount) && !bTerminalState)
		{
			
			{
				s= GetStateId();
				for (robot= 0; robot<m_robotCount; robot++)
					localActions[robot]= SelectAction(robot,s);
				
				//rewards[robot]= lastReward;
				//update QMatrix
				//actionTaken= SelectAction(s);
				//if (actionTaken!=ACTION_NONE)
					totalMoves++;
			//	lastAction[robot]= actionTaken;
				lastReward= TakeJointAction(localActions,bTerminalState);
				int s2= GetStateId();

				totalRewards+= lastReward*pow(GAMMA,m_step);

				if (lastReward>=0)
					AddState(m_pWorldState->m_positions[NUM_AGENTS-1].m_x
						,m_pWorldState->m_positions[NUM_AGENTS-1].m_y);


				
				//if (lastAction[robot]>=0 && lastState[robot]>=0)
				if (!bGreedy && s>=0)
				{
					for (robot= 0; robot<m_robotCount; robot++)
					{	
						maxQ= GetMaxQ(robot,s2);
						newQ= ALPHA*lastReward + GAMMA*maxQ;

						if (newQ>m_pQMatrices[robot][s][localActions[robot]])
							m_pQMatrices[robot][s][localActions[robot]]+=
								ALPHA*(lastReward + GAMMA*maxQ- m_pQMatrices[robot][s][localActions[robot]]);
					}
				}

			}
			m_step++;
		}


		if (!bGreedy)
		{
			totalMoveCount+=m_step;

			if (GoalReached() && firstGoal<0)
			{
				firstGoal= totalMoveCount;
			}
			else
			{
				fprintf(pOutFile
					,"%d %.2f %d %d %d %f\n",m_episodeCount,m_epsilon,g_numVVStates
					,m_step,totalMoveCount,totalRewards);
			}
		}
		else if (GoalReached())
		{	
			bGoalReached= bGoalReached || GoalReached();
			bestGreedyPolicy= m_step*m_robotCount;
		}
		if (bGreedy)
			fprintf(pOutFile2,"%d %.2f %d %d %d %f\n",m_episodeCount,m_epsilon,g_numVVStates
					,m_step,totalMoveCount,totalRewards);
			//printf("%d %.2f %d %d %d\n",m_episodeCount,m_epsilon,g_numVVStates,m_step,totalMoves,totalMoveCount);
		
		printf ("RUN #%d : %d/%d episodes\r",run,m_episodeCount,totalEpisodeCount);


		epsilon= max(0.f,epsilon-epsilon_delta);
		

	}
	fclose(pOutFile);
	fclose(pOutFile2);

}
	delete []pVVStates;
}