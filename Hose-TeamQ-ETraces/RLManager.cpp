#include "stdafx.h"
#include "RLManager.h"
#include <assert.h>
//#include "StatManager.h"
#include "WorldState.h"

//HOSE MOD//////////////////
#include "GridDistances.h"
//////////////////////////

#include "ParameterManager.h"


int CRLManager::m_step= 0;
int CRLManager::m_maxStepCount= 0;

//HOSE MOD///////////////////////////////

#define NUM_STATES_PER_ROBOT CountRelPosInRange(MAX_REL_DIST)
#define NUM_STATES (int)(pow(NUM_STATES_PER_ROBOT,(float)NUM_AGENTS))
#define NUM_ACTIONS_PER_ROBOT 5//9
#define NUM_ACTIONS (int)(pow(NUM_ACTIONS_PER_ROBOT,(float)NUM_AGENTS))
////////////////////////////////////////

struct Q_VALUE
{
	int s;
	int a;
	float Qvalue;
};
#define NUM_MAX_Q_VALUES 10000000
struct Q_VALUE g_QValues[NUM_MAX_Q_VALUES];
int g_numQValues;


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




void CRLManager::Init()
{
	
//HOSE MOD/////////////////////////
		InitDistMatrix(MAX_REL_DIST+1,MAX_REL_DIST+1,MAX_REL_DIST,true);

///////////////////////////////////
	m_robotCount= g_pParameterManager->GetIntParameter("ROBOT_COUNT");
	m_actionCount= NUM_ACTIONS;//g_pParameterManager->GetIntParameter("ACTION_COUNT");
	m_epsilon= (float)g_pParameterManager->GetDoubleParameter("TRAINING_EPSILON");
	m_stateCount= NUM_STATES;//NUM_STATES_PER_AGENT*NUM_STATES_PER_AGENT*NUM_STATES_PER_AGENT*NUM_STATES_PER_AGENT;

	m_pWorldState= new CWorldState(m_robotCount);



	//for (int i= 0; i<NUM_AGENTS; i++)
	{
/*		unsigned int numStates= NUM_STATES;
		unsigned int numActions= NUM_ACTIONS;
		unsigned int numEntries= numStates*numActions;
		m_pQMatrices= new float*[numStates];

		for (int i= 0; i<numStates; i++)
			m_pQMatrices[i]= new float[numActions];*/
	}


	//random function
	srand(1);


	//m_pStatManager= new CStatManager();
	//m_pStatManager->InitDataWindow(g_pParameterManager->GetIntParameter("STAT_WINDOW_SIZE"));

}

void CRLManager::Release()
{
	/*for (int i=0; i<NUM_STATES; i++) delete [] m_pQMatrices[i];
	delete [] m_pQMatrices;*/
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
		aux*= NUM_STATES_PER_ROBOT;
	}
	return stateId;
}

int CRLManager::JointActionIdFromLocalActions(int* localAction)
{
	int actionId= 0;
	for (int i= NUM_AGENTS-1; i>=0; i--)
	{
		actionId= localAction[i] + actionId*NUM_ACTIONS_PER_ROBOT;
	}
	return actionId;
}
short CRLManager::LocalActionFromJointActionId(int actionId,int agentId)
{
	for (int i= 0; i<agentId; i++) actionId= actionId/NUM_ACTIONS_PER_ROBOT;
	return (short) actionId%NUM_ACTIONS_PER_ROBOT;
}

void CRLManager::SelectAction(int* localAction,int stateId)
{
	double max;
	int maxA;
	int randomNumber;
	double probs[1000];
	float q_values[1000];
	double sum= 0.;

	for (int i=0; i<NUM_ACTIONS; i++) q_values[i]= 0.f;

	for (int i=0; i<g_numQValues; i++)
		if (g_QValues[i].s==stateId)
			q_values[g_QValues[i].a]= g_QValues[i].Qvalue;

	for (int agent= 0; agent<NUM_AGENTS; agent++)
	{

		if (m_epsilon>0.00001 && g_pParameterManager->GetIntParameter("USE_BOLTZMANN"))
		{
			double temp= g_pParameterManager->GetDoubleParameter("MIN_TEMPERATURE")
					+ m_epsilon*g_pParameterManager->GetDoubleParameter("TEMPERATURE_RANGE");
			for (int i= 0; i<NUM_ACTIONS; i++)
			{
				sum+= exp(q_values[i]/temp);//((m_pQMatrices[stateId][i])/temp);
			}
			//sum= max(sum,0.001);
			for (int i= 0; i<NUM_ACTIONS; i++)
			{
				probs[i]= 10000*exp((q_values[i]/temp))/sum;//((m_pQMatrices[stateId][i])/temp)/sum;
			}
			int rn= rand()%10001;
			int j= 0;
			double probsum= probs[0];
			while (j<NUM_ACTIONS && rn>=probsum)
			{
				j++;
				probsum+=probs[j];
			}
			localAction[agent]= LocalActionFromJointActionId(j,agent);
		}
		else
		{
			//vetoes + e-greedy
			int probability=(int)(m_epsilon*100000);
			randomNumber= rand()%100001;
			if (randomNumber < (int)probability)
			{
				//random among non-vetoed actions
				int randomActionOrder= rand()%(NUM_ACTIONS_PER_ROBOT);
				int action= 0;int actionOrder= -1;
				while (randomActionOrder!=actionOrder)
				{
					actionOrder++;
					if (randomActionOrder!=actionOrder) action++;
				}
				localAction[agent]= action;
			}
			else
			{
				max= q_values[0];//m_pQMatrices[stateId][0];
				maxA= 0;
				for (int i=1; i<NUM_ACTIONS; i++)
				{
					if (q_values[i]>max)//m_pQMatrices[stateId][i]>max)
					{
						max= q_values[i];//m_pQMatrices[stateId][i];
						maxA= i;
					}
				}
				localAction[agent]= LocalActionFromJointActionId(maxA,agent);
			}			
		}
	}
}

void CRLManager::SetQValue(int stateId,int actionId,float qvalue,int index)
{
	if (index<0)
	{
		g_QValues[g_numQValues].a=actionId;
		g_QValues[g_numQValues].s=stateId;
		g_QValues[g_numQValues].Qvalue=qvalue;
		g_numQValues++;
		assert(g_numQValues<NUM_MAX_Q_VALUES);
	}
	else
	{
		g_QValues[index].a=actionId;
		g_QValues[index].s=stateId;
		g_QValues[index].Qvalue=qvalue;
	}
}

double CRLManager::GetQValue(int stateId,int actionId,int &index)
{
	for (int i= 0; i<g_numQValues; i++)
		if (g_QValues[i].a==actionId && g_QValues[i].s==stateId)
		{
			index=i;
			return g_QValues[i].Qvalue;
		}
	index= -1;
	return 0.0;
}

double CRLManager::GetMaxQ(int stateId)
{
	float q_values[1000];
	double max;

	if (stateId<0) return 0.0;

	for (int i= 0; i<NUM_ACTIONS; i++) q_values[i]= 0.f;

	for (int i= 0; i<g_numQValues; i++)
		if (g_QValues[i].s==stateId)
			q_values[g_QValues[i].a]= g_QValues[i].Qvalue;

	max= q_values[0];//m_pQMatrices[stateId][0];
	for (int i=1; i<NUM_ACTIONS; i++)
	{
		if (q_values[i]>max)//m_pQMatrices[stateId][i]>max)
		{
			max= q_values[i];//m_pQMatrices[stateId][i];
		}
	}
	return max;
}

bool CRLManager::GoalReached()
{
	return (m_pWorldState->m_positions[NUM_AGENTS-1].m_x==GOAL_X
		&& m_pWorldState->m_positions[NUM_AGENTS-1].m_y==GOAL_Y);
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
#define ALPHA 0.1
#define GAMMA 0.9



void CRLManager::RunEpisodes(int totalEpisodeCount,char* dirname)
{
	pVVStates= new struct tipPos[10000];


	float epsilon;
	int robot;
	FILE *pOutFile,*pOutFile2;
	bool bGreedy;
	int totalMoveCount= 0;

	m_maxStepCount= g_pParameterManager->GetIntParameter("MAX_EPISODE_STEP_COUNT");

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
	

/*	for (int i= 0; i<NUM_STATES; i++)
		memset(m_pQMatrices[i],0,NUM_ACTIONS*sizeof(float));
*/
	g_numQValues= 0;

	double totalRewards= 0.0;
	g_numVVStates= 0;

	for (m_episodeCount= 0; m_episodeCount<=totalEpisodeCount; m_episodeCount++)
	{
		m_step= 0;

		bool bTerminalState= false;
		int blamedModuleId= -1;
		int blamedRobotId= -1;
		int actionTaken= -1;
		double reward;//={0,0};
		int localActions[NUM_AGENTS];

		
		totalRewards= 0.0;

		double lastReward= 0.;
		int s=-1;
		double maxQ= 0.;
		int totalMoves= 0;
		int s2;

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

		while ((!bGreedy || m_step<120) && (bGreedy || m_step<m_maxStepCount) && !bTerminalState)// && m_step<=m_maxStepCount)
		{

			CWorldState oldState= *m_pWorldState;
			s= GetStateId();

			SelectAction(localActions,s);

			totalMoves++;

			lastReward= TakeJointAction(localActions,bTerminalState);
			totalRewards+= lastReward*pow(GAMMA,m_step);

			s2= GetStateId();

			if (lastReward>=0)
				AddState(m_pWorldState->m_positions[NUM_AGENTS-1].m_x
					,m_pWorldState->m_positions[NUM_AGENTS-1].m_y);

			double delta;
			int jointAction;
			float value;
			int index;
			if (!bGreedy)
			{
				maxQ= GetMaxQ(s2);
				jointAction= JointActionIdFromLocalActions(localActions);

				value= GetQValue(s,jointAction,index);

				delta= lastReward + GAMMA*maxQ- value;//m_pQMatrices[s][jointAction];

				SetQValue(s,jointAction,value+ALPHA*delta,index);//m_pQMatrices[s][jointAction]+=ALPHA*delta;
			}
			printf ("RUN #%d // EPISODE %d/%d // STEP %d \r",run,m_episodeCount,totalEpisodeCount,m_step);
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
			bestGreedyPolicy= m_step*NUM_AGENTS;
		}
		if (bGreedy)
			fprintf(pOutFile2,"%d %.2f %d %d %d %f\n",m_episodeCount,m_epsilon,g_numVVStates
					,m_step,totalMoveCount,totalRewards);
			//printf("%d %.2f %d %d %d\n",m_episodeCount,m_epsilon,g_numVVStates,m_step,totalMoves,totalMoveCount);
		
		


		epsilon= max(0.f,epsilon-epsilon_delta);
		

	}
	fclose(pOutFile);
	fclose(pOutFile2);
}
	//printf("first goal: %d",firstGoal);

	delete [] pVVStates;

//	m_pStatManager->PrintStats();
//	m_pStatManager->CloseStats();

}
