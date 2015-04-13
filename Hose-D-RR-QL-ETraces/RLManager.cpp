#include "stdafx.h"
#include "RLManager.h"

#include "StatManager.h"
#include "WorldState.h"

//HOSE MOD//////////////////
#include "GridDistances.h"
//////////////////////////

#include "ParameterManager.h"


int CRLManager::m_step= 0;
int CRLManager::m_maxStepCount= 0;

//HOSE MOD///////////////////////////////

#define NUM_STATES_PER_AGENT CountRelPosInRange(MAX_REL_DIST)
#define NUM_STATES (int)(pow(NUM_STATES_PER_AGENT,(float)NUM_AGENTS))
#define NUM_ACTIONS_PER_AGENT 5//9
#define NUM_ACTIONS 625//5^4
//(int)(pow(NUM_ACTIONS_PER_AGENT,(float)NUM_AGENTS))
////////////////////////////////////////

struct transition
{
	int s0, s1;
};

#define NUM_MAX_TRANSITIONS_PER_ACTION 2000000
struct transition *pTransitions[NUM_ACTIONS_PER_AGENT*NUM_AGENTS];
int nTransitions[NUM_ACTIONS_PER_AGENT*NUM_AGENTS];

void AddTransition(int s0,int agentId,int a, int s1)
{
	int actionId= agentId*NUM_ACTIONS_PER_AGENT + a;
	for (int i= 0; i<nTransitions[actionId]; i++)
	{
		if (pTransitions[actionId][i].s0==s0)// && pTransitions[i].a==actionId)
			return;
	}

	if (nTransitions[actionId]>=NUM_MAX_TRANSITIONS_PER_ACTION)
	{
		printf("max transition count reached\n");
		return;
	}
	pTransitions[actionId][nTransitions[actionId]].s0= s0;
	pTransitions[actionId][nTransitions[actionId]].s1= s1;
	//pTransitions[nTransitions].a= actionId;
	nTransitions[actionId]++;
};

int GetTransition(int s0,int agentId,int a)
{
	if (s0<0 || a<0) return -1;

	int actionId= agentId*NUM_ACTIONS_PER_AGENT + a;
	for (int i= 0; i<nTransitions[actionId]; i++)
	{
		if (pTransitions[actionId][i].s0==s0)// && pTransitions[i].a==actionId)
			return pTransitions[actionId][i].s1;
	}

	return s0;
}





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

#define NUM_MAX_VETOES1_PER_ACTION 50000
struct veto1
{
	short prevX,prevY;
	short nextX,nextY;
	short actions[NUM_ACTIONS_PER_AGENT];
	short numActions;
};
struct veto2
{
	bool obsUp,obsDown,obsLeft,obsRight;
	short action;
	short robot;
};
struct veto3
{
	short posX,posY;
	short action, robot;
};
struct veto2 *pVetoes2;
struct veto1 *pVetoes1[NUM_AGENTS];
struct veto3 *pVetoes3;
int g_numVetoes1[NUM_AGENTS];
int g_numVetoes2 = 0;
int g_numVetoes3 = 0;
int g_reason= 0;
int g_outCount= 0;
void AddVeto(CWorldState *pWorldState,short action,short robot)
{
	if (g_reason==1)
	{

		//KEEP DISTANCES
		int nextX,nextY;
		int prevX,prevY;

		if (robot==NUM_AGENTS-1){nextX= 0; nextY= 0;}
		else
		{
			nextX= pWorldState->m_positions[robot+1].m_x-pWorldState->m_positions[robot].m_x;
			nextY= pWorldState->m_positions[robot+1].m_y-pWorldState->m_positions[robot].m_y;
		}

		if (robot==0)
		{
			prevX= -pWorldState->m_positions[robot].m_x;
			prevY= -pWorldState->m_positions[robot].m_y;
		}
		else
		{
			prevX= pWorldState->m_positions[robot-1].m_x - pWorldState->m_positions[robot].m_x;
			prevY= pWorldState->m_positions[robot-1].m_y - pWorldState->m_positions[robot].m_y;
		}
		int i;
		for (i= 0; i<g_numVetoes1[robot] && (pVetoes1[robot][i].prevX!=prevX || pVetoes1[robot][i].prevY!=prevY
			|| pVetoes1[robot][i].nextX!=nextX || pVetoes1[robot][i].nextY!=nextY); i++);
		
		if (i>=g_numVetoes1[robot])
		{
			if (g_numVetoes1[robot]<NUM_MAX_VETOES1_PER_ACTION)
			{
				pVetoes1[robot][g_numVetoes1[robot]].prevX= prevX;
				pVetoes1[robot][g_numVetoes1[robot]].prevY= prevY;
				pVetoes1[robot][g_numVetoes1[robot]].nextX= nextX;
				pVetoes1[robot][g_numVetoes1[robot]].nextY= nextY;
				pVetoes1[robot][g_numVetoes1[robot]].numActions= 1;
				pVetoes1[robot][g_numVetoes1[robot]].actions[0]= action;
				g_numVetoes1[robot]++;
			}
			else printf("max veto count reached\n");
		}
		else
		{
			int j;
			for (j= 0; j<pVetoes1[robot][i].numActions && pVetoes1[robot][i].actions[j]!=action; j++);

			if (j>=pVetoes1[robot][i].numActions)
			{
				pVetoes1[robot][i].actions[pVetoes1[robot][i].numActions]= action;
				pVetoes1[robot][i].numActions++;
			}
		}
	}
	else if (g_reason==2)
	{
		//COLLISIONS
		bool bUp=false,bDown=false,bRight=false,bLeft=false;
		for (int i= 0; i<NUM_AGENTS; i++)
		{
			if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x 
				&& pWorldState->m_positions[i].m_y+1==pWorldState->m_positions[robot].m_y)
				bUp= true;
			if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x 
				&& pWorldState->m_positions[i].m_y-1==pWorldState->m_positions[robot].m_y)
				bDown= true;
			if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x-1 
				&& pWorldState->m_positions[i].m_y==pWorldState->m_positions[robot].m_y)
				bLeft= true;
			if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x+1
				&& pWorldState->m_positions[i].m_y==pWorldState->m_positions[robot].m_y)
				bRight= true;
		}
		int i;
		for (i= 0; i<g_numVetoes2 && (pVetoes2[i].obsDown!=bDown || pVetoes2[i].obsLeft!=bLeft
			|| pVetoes2[i].obsRight!=bRight || pVetoes2[i].obsUp!=bUp
			|| pVetoes2[i].robot!=robot || pVetoes2[i].action!=action); i++);
		
		if (i>=g_numVetoes2)
		{
			pVetoes2[g_numVetoes2].obsDown= bDown;
			pVetoes2[g_numVetoes2].obsUp= bUp;
			pVetoes2[g_numVetoes2].obsLeft= bLeft;
			pVetoes2[g_numVetoes2].obsRight= bRight;
			pVetoes2[g_numVetoes2].action= action;
			pVetoes2[g_numVetoes2].robot= robot;
			g_numVetoes2++;
		}
	}
	else if (g_reason==3)
	{
		int i;
		for (i= 0; i<g_numVetoes3 && (pVetoes3[i].posX!=pWorldState->m_positions[robot].m_x
			|| pVetoes3[i].posY!=pWorldState->m_positions[robot].m_y
			|| pVetoes3[i].robot!=robot || pVetoes3[i].action!=action); i++);
		if (i>=g_numVetoes3)
		{
			pVetoes3[g_numVetoes3].posX=pWorldState->m_positions[robot].m_x;
			pVetoes3[g_numVetoes3].posY=pWorldState->m_positions[robot].m_y;
			pVetoes3[g_numVetoes3].robot=robot;
			pVetoes3[g_numVetoes3].action=action;
			g_numVetoes3++;
		}
	}
}

bool CRLManager::Vetoed(int stateId,short action, short robot)
{
	int i;
	int prevX,prevY,nextX,nextY;

	CWorldState state;
	CWorldState *pWorldState= &state;
	BuildStateFromStateId(stateId,pWorldState);
	
	
	//KEEP DISTANCES

	if (robot==NUM_AGENTS-1){nextX= 0; nextY= 0;}
	else
	{
		nextX= pWorldState->m_positions[robot+1].m_x-pWorldState->m_positions[robot].m_x;
		nextY= pWorldState->m_positions[robot+1].m_y-pWorldState->m_positions[robot].m_y;
	}

	if (robot==0)
	{
		prevX= -pWorldState->m_positions[robot].m_x;
		prevY= -pWorldState->m_positions[robot].m_y;
	}
	else
	{
		prevX= pWorldState->m_positions[robot-1].m_x - pWorldState->m_positions[robot].m_x;
		prevY= pWorldState->m_positions[robot-1].m_y - pWorldState->m_positions[robot].m_y;
	}

	for ( i= 0; i<g_numVetoes1[robot] && (pVetoes1[robot][i].prevX!=prevX || pVetoes1[robot][i].prevY!=prevY
		|| pVetoes1[robot][i].nextX!=nextX || pVetoes1[robot][i].nextY!=nextY); i++);
	
	if (i<g_numVetoes1[robot])
	{
		int j;
		for (j= 0; j<pVetoes1[robot][i].numActions && pVetoes1[robot][i].actions[j]!=action; j++);
		
		if (j<pVetoes1[robot][i].numActions)
			return true;
	}

			//COLLISIONS
	bool bUp=false,bDown=false,bRight=false,bLeft=false;
	for (i= 0; i<NUM_AGENTS; i++)
	{
		if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x 
			&& pWorldState->m_positions[i].m_y+1==pWorldState->m_positions[robot].m_y)
			bUp= true;
		if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x 
			&& pWorldState->m_positions[i].m_y-1==pWorldState->m_positions[robot].m_y)
			bDown= true;
		if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x-1 
			&& pWorldState->m_positions[i].m_y==pWorldState->m_positions[robot].m_y)
			bLeft= true;
		if (pWorldState->m_positions[i].m_x==pWorldState->m_positions[robot].m_x+1
			&& pWorldState->m_positions[i].m_y==pWorldState->m_positions[robot].m_y)
			bRight= true;
	}

	for (i= 0; i<g_numVetoes2 && (pVetoes2[i].obsDown!=bDown || pVetoes2[i].obsLeft!=bLeft
			|| pVetoes2[i].obsRight!=bRight || pVetoes2[i].obsUp!=bUp
		|| pVetoes2[i].robot!=robot || pVetoes2[i].action!=action); i++);
	if (i<g_numVetoes2)
		return true;

	//out of grid
	for (i= 0; i<g_numVetoes3 && (pVetoes3[i].posX!=pWorldState->m_positions[robot].m_x
	|| pVetoes3[i].posY!=pWorldState->m_positions[robot].m_y
	|| pVetoes3[i].robot!=robot || pVetoes3[i].action!=action); i++);

	if (i<g_numVetoes3)
		return true;

	return false;
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

	m_pWorldState= new CWorldState();

	m_pQMatrices= new double *[NUM_AGENTS];//[NUM_STATES][NUM_ACTIONS]


	for (int i= 0; i<NUM_AGENTS; i++)
	{	
		m_pQMatrices[i]= new double[NUM_STATES*NUM_ACTIONS_PER_AGENT];

		for (int j= 0; j<NUM_STATES*NUM_ACTIONS_PER_AGENT; j++)
			m_pQMatrices[i][j]= 0.;
	}


	//random function
	srand(1);


	//m_pStatManager= new CStatManager();
	//m_pStatManager->InitDataWindow(g_pParameterManager->GetIntParameter("STAT_WINDOW_SIZE"));

}

void CRLManager::Release()
{
	delete m_pWorldState;
	delete m_pStatManager;

}

void CRLManager::BuildStateFromStateId(int stateId, CWorldState* pWorldState)
{
	int index;

	for (int i= 0; i<NUM_AGENTS; i++)
	{
		index= stateId % NUM_STATES_PER_AGENT;
		GetIthRelPos(index,pWorldState->m_positions[i].m_x
			,pWorldState->m_positions[i].m_y);
		stateId= stateId / NUM_STATES_PER_AGENT;
	}
	for (int i= 1; i<NUM_AGENTS; i++)
	{
		pWorldState->m_positions[i].m_x+= pWorldState->m_positions[i-1].m_x;
		pWorldState->m_positions[i].m_y+= pWorldState->m_positions[i-1].m_y;
	}
}

int CRLManager::GetStateId(CWorldState* pWorldState)
{
	int aux= 1;
	int stateId = 0;
//HOSE MOD///////////////////////////////////
	int prevX= 0, prevY= 0;
	int posCount= CountRelPosInRange(MAX_REL_DIST);
	for (int i= 0; i<NUM_AGENTS; i++)
	{
		
		int index= GetRelPosIndex(pWorldState->m_positions[i].m_x-prevX
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

	if (g_pParameterManager->GetIntParameter("USE_VETOS"))
	{
		int vetoCount= 0;
		for (int i= 0; i<NUM_ACTIONS_PER_AGENT; i++)
		{
			if (Vetoed(stateId,i,agentId))
			{
				vetoCount++;
				vetoes[i]= true;
			}
			else vetoes[i]= false;
		}
		if (g_pParameterManager->GetIntParameter("USE_BOLTZMANN") && m_epsilon>0.001)
		{		
			double temp= g_pParameterManager->GetDoubleParameter("MIN_TEMPERATURE")
				+ m_epsilon*g_pParameterManager->GetDoubleParameter("TEMPERATURE_RANGE");
			for (int i= 0; i<NUM_ACTIONS_PER_AGENT; i++)
			{
				if (!vetoes[i])
					sum+= exp((m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i])/temp);
			}
			sum= max(sum,0.001);
			for (int i= 0; i<NUM_ACTIONS_PER_AGENT; i++)
			{
				if (!vetoes[i])
					probs[i]= 100*exp((m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i])/temp)/sum;
				else probs[i]= 0.;
			}
			int rn2= rand()%101;
			int j2= 0;
			double probsum2= probs[0];
			while (j2<NUM_ACTIONS_PER_AGENT && rn2>=probsum2)
			{
				j2++;
				probsum2+=probs[j2];
			}
			return min(j2,NUM_ACTIONS_PER_AGENT-1);
		}
		else
		{
			//vetoes + e-greedy
			int probability=(int)(m_epsilon*1000);
			randomNumber= rand()%1001;
			if (randomNumber < (int)probability)
			{
				//random among non-vetoed actions
				int randomActionOrder= rand()%(NUM_ACTIONS_PER_AGENT-vetoCount);
				int action= 0;int actionOrder= -1;
				while (randomActionOrder!=actionOrder)
				{
					if (!vetoes[action]) actionOrder++;
					if (randomActionOrder!=actionOrder) action++;
				}
				return action;
			}

			//greedy among non-vetoed actions
			max= -99999999999.;//m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+0];
			maxA= -1;//0;
			for (int i=0; i<NUM_ACTIONS_PER_AGENT; i++)
			{
				if (!vetoes[i] && m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i]>max)
				{
					max= m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i];
					maxA= i;
				}
			}
			if (maxA>=0)
				return maxA;
			return 0;
		}
	}



	if (m_epsilon>0.00001 && g_pParameterManager->GetIntParameter("USE_BOLTZMANN"))
	{			
		double temp= g_pParameterManager->GetDoubleParameter("MIN_TEMPERATURE")
				+ m_epsilon*g_pParameterManager->GetDoubleParameter("TEMPERATURE_RANGE");
		for (int i= 0; i<NUM_ACTIONS_PER_AGENT; i++)
		{
			sum+= exp((m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i])/temp);
		}
		sum= max(sum,0.001);
		for (int i= 0; i<NUM_ACTIONS_PER_AGENT; i++)
		{
			probs[i]= 100*exp((m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i])/temp)/sum;
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
/*
	int probability=(int)(m_epsilon*1000);
	randomNumber= rand()%1001;
	if (randomNumber < (int)probability)
		return rand()%NUM_ACTIONS_PER_AGENT;*/

	max= m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+0];
	maxA= 0;
	for (int i=1; i<NUM_ACTIONS_PER_AGENT; i++)
	{
		if (m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i]>max)
		{
			max= m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i];
			maxA= i;
		}
	}
	return maxA;
}

double CRLManager::GetMaxQ(int agentId,int stateId)
{
	double max;
	if (stateId<0) return 0.0;

	max= m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+0];
	for (int i=1; i<NUM_ACTIONS_PER_AGENT; i++)
	{
		if (m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i]>max)
		{
			max= m_pQMatrices[agentId][stateId*NUM_ACTIONS_PER_AGENT+i];
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
	//CWorldState oldState= *m_pWorldState;
	negReward= g_pParameterManager->GetDoubleParameter("REWARD_NEGATIVE");
	posReward= g_pParameterManager->GetDoubleParameter("REWARD_POSITIVE");
	neuReward= g_pParameterManager->GetDoubleParameter("REWARD_NEUTRAL");

	g_reason= 0;
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
			g_reason=3;
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
				g_reason=2;
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
				g_reason= 2;
			}
			else if ( (destPosX[a1]==destPosX[a2] && destPosY[a1]==destPosY[a2]) )
			{
				//destPosX[a1]= m_pWorldState->m_positions[a1].m_x;
				//destPosY[a1]= m_pWorldState->m_positions[a1].m_y;

				destPosX[a2]= m_pWorldState->m_positions[a2].m_x;
				destPosY[a2]= m_pWorldState->m_positions[a2].m_y;
				nCollisions++;
				g_reason= 2;
			}
		}
	}
	//hose stretched???
	int prevX= 0,prevY= 0;
	bool bStretched= true;

	//while (bStretched)
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
				g_reason=1;
				/*
				destPosX[i]= m_pWorldState->m_positions[i].m_x;
				destPosY[i]= m_pWorldState->m_positions[i].m_y;*/
			}
			prevX= destPosX[i];
			prevY= destPosY[i];
		}
		//re-check if the hose is stretched
		/*bStretched= false;
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


	if (m_pWorldState->m_positions[NUM_AGENTS-1].m_x==g_pParameterManager->GetIntParameter("GOAL_X")
		&& m_pWorldState->m_positions[NUM_AGENTS-1].m_y==g_pParameterManager->GetIntParameter("GOAL_Y"))
	{
		bTerminalState= true;
//		printf("GOAL\n");
		return posReward;
	}


	if (nCollisions)
	{
		//*m_pWorldState= oldState;
		bTerminalState= true;
		reward= nCollisions*negReward;
	}
	else reward= neuReward;
///////////////////////////////////////////////////////////
	return reward;

}
#define ALPHA 0.1
#define GAMMA 0.9


#define NUM_MAX_ETRACES 50
#define LAMBDA 0.9

struct ETrace
{
	int state,action;
	double e;
};
struct ETrace g_eTraces[NUM_AGENTS][NUM_MAX_ETRACES];
int g_numETraces[NUM_AGENTS];

void UpdateETraces(int agent)
{
	for (int i= 0; i<g_numETraces[agent]; i++)
	{
		g_eTraces[agent][i].e*= GAMMA*LAMBDA;
	}
}

void AddETrace(int agent, int state, int action, bool bReplace=true)
{
	int index= -1;
	//1st: does it exist already?
	for (int i= 0; i<g_numETraces[agent]; i++)
	{
		if (g_eTraces[agent][i].state==state && g_eTraces[agent][i].action==action)
		{	//replace
			if (bReplace)
				g_eTraces[agent][i].e=LAMBDA;
			else g_eTraces[agent][i].e+= LAMBDA;
			return;
		}
	}
	if (g_numETraces[agent]<NUM_MAX_ETRACES-1)
	{
		//there's free space
		index= g_numETraces[agent];
		g_numETraces[agent]++;
	}
	else
	{
		index= 0;
		for (int i= 1; i<g_numETraces[agent]; i++)
		{
			if (g_eTraces[agent][i].e<g_eTraces[agent][index].e)
				index= i;
		}
	}
	g_eTraces[agent][index].state= state;
	g_eTraces[agent][index].action= action;
	g_eTraces[agent][index].e= LAMBDA;
}

void CRLManager::RunEpisodes(int totalEpisodeCount, char* dirname)
{
	pVVStates= new struct tipPos[NUM_STATES];
	for (int i= 0; i<NUM_ACTIONS_PER_AGENT*NUM_AGENTS; i++)
		pTransitions[i]= new transition[NUM_MAX_TRANSITIONS_PER_ACTION];
	for (int i= 0; i<NUM_AGENTS; i++)
		pVetoes1[i]= new struct veto1[NUM_MAX_VETOES1_PER_ACTION];
	pVetoes2= new struct veto2[10000];
	pVetoes3= new struct veto3[10000];

	float epsilon= 1.;
	int robot;
	FILE *pOutFile,*pOutFile2;
	bool bGreedy;
	int totalMoveCount= 0;
	//int statWindowSize= g_pParameterManager->GetIntParameter("STAT_WINDOW_SIZE");
	m_maxStepCount= g_pParameterManager->GetIntParameter("MAX_EPISODE_STEP_COUNT");

	printf("Simulation started\n");

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
		fclose(pOutFile);
		fclose(pOutFile2);
bool bGoalReached= false;
int bestGreedyPolicy= 0;
int firstGoal=-1;

totalMoveCount= 0;
epsilon= 1;
for (int i= 0; i<NUM_AGENTS; i++) g_numVetoes1[i] = 0;
g_numVetoes2 = 0;
g_numVetoes3 = 0;
for (int i= 0; i<NUM_ACTIONS; i++) nTransitions[i]= 0;

for (int i= 0; i<NUM_AGENTS; i++)
{	
	for (int j= 0; j<NUM_STATES*NUM_ACTIONS_PER_AGENT; j++)
		m_pQMatrices[i][j]= 0.;
}
	epsilon= (float)g_pParameterManager->GetDoubleParameter("TRAINING_EPSILON");

//	for (int maxNumSteps= 1000000; totalMoveCount<=maxNumSteps; )//maxNumSteps+= 1000000)
	for (m_episodeCount= 0; m_episodeCount<totalEpisodeCount; m_episodeCount++)
	{
		m_step= 0;

	for (int i= 0; i<m_robotCount; i++) g_numETraces[i]= 0;

		bool bTerminalState= false;
		int blamedModuleId= -1;
		int blamedRobotId= -1;
		int actionTaken= -1;
		int lastAction[NUM_AGENTS];//={-1,-1};
		int lastState[NUM_AGENTS];//={-1,-1};
		double rewards[NUM_AGENTS];//={0,0};
		int localActions[NUM_AGENTS];

		for (int i= 0; i<NUM_AGENTS; i++)
		{
			lastAction[i]= -1;
			lastState[i]= -1;
			rewards[i]= 0.;
		}

		double lastReward= 0.;
		int s=-1;
		double maxQ= 0.;
		int totalMoves= 0;
		int s2[NUM_AGENTS];
		int s_prime;

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


		double totalRewards= 0.;


		m_pWorldState->Init();


		while ((!bGreedy || m_step<120) && (bGreedy || m_step<m_maxStepCount) && !bTerminalState)
		{
			if (!bGreedy)//(1)
			{
				for (robot= 0; !bTerminalState && robot<NUM_AGENTS; robot++)
				{
					CWorldState oldState= *m_pWorldState;
					s= GetStateId(m_pWorldState);

					if (lastAction[robot]>=0)
					{
						maxQ= GetMaxQ(robot,s);//s2[robot]);
						for (int i= 0; i<g_numETraces[robot]; i++)
						{
							double delta= rewards[robot] + GAMMA*maxQ
								- m_pQMatrices[robot][g_eTraces[robot][i].state
							*NUM_ACTIONS_PER_AGENT+g_eTraces[robot][i].action];
							if (delta>0)
							m_pQMatrices[robot][g_eTraces[robot][i].state*NUM_ACTIONS_PER_AGENT+g_eTraces[robot][i].action]+=
								ALPHA*delta*g_eTraces[robot][i].e;
						}
					}

						rewards[robot]=0.;
						lastState[robot]= s;


						//rewards[robot]= lastReward;
						//update QMatrix
						actionTaken= SelectAction(robot,s);
						totalMoves++;
						lastAction[robot]= actionTaken;

						int localActions[NUM_AGENTS];
						for (int i= 0; i<NUM_AGENTS; i++)
						{
							if (i==robot) localActions[i]= actionTaken;
							else localActions[i]= ACTION_NONE;
						}
						lastReward= TakeJointAction(localActions,bTerminalState);
						s2[robot]= GetStateId(m_pWorldState);

						totalRewards+= lastReward*pow(GAMMA,m_step);
						
						if (lastReward<0)
							AddVeto(&oldState,actionTaken,robot);
						else
						{
							AddState(m_pWorldState->m_positions[NUM_AGENTS-1].m_x
								,m_pWorldState->m_positions[NUM_AGENTS-1].m_y);

							UpdateETraces(robot);
						
							AddETrace(robot,s,actionTaken);

							//REMEMBER TRANSITION
							AddTransition(s,robot,actionTaken,s2[robot]);

							//spread lastReward
							for (int i= 0; i<NUM_AGENTS; i++)
							{
								if (lastAction[i]>=0)
								{
									int exp= (robot-i)% NUM_AGENTS;//i-robot;
									while (exp<0) exp+= NUM_AGENTS;//
									rewards[i]+= lastReward*pow(GAMMA,exp);
								}
							}
						}
					}
					if (bTerminalState && lastReward>=0)
					for (robot= 0; robot<NUM_AGENTS; robot++)
					{
						maxQ= GetMaxQ(robot,s2[robot]);
						for (int i= 0; i<g_numETraces[robot]; i++)
						{
							double delta= rewards[robot] + GAMMA*maxQ
								- m_pQMatrices[robot][g_eTraces[robot][i].state
							*NUM_ACTIONS_PER_AGENT+g_eTraces[robot][i].action];
							if (delta>0)
							m_pQMatrices[robot][g_eTraces[robot][i].state*NUM_ACTIONS_PER_AGENT+g_eTraces[robot][i].action]+=
								ALPHA*delta*g_eTraces[robot][i].e;
						}
					}
				}
			else
			{
				s= GetStateId(m_pWorldState);
				
				int states[NUM_AGENTS];
				states[0]= s;
				for (robot= 0; s>=0 && robot<NUM_AGENTS; robot++)
				{
					localActions[robot]= SelectAction(robot,s);
					s= GetTransition(s,robot,localActions[robot]);
					if (robot<NUM_AGENTS-1)
						states[robot+1]= s;
				}
				for (;robot<NUM_AGENTS; robot++) localActions[robot]= ACTION_NONE;
				
				//for (;robot>=0; robot--) localActions[robot]= ACTION_NONE;
				//if (s>=0)
				{
					double r= TakeJointAction(localActions,bTerminalState);
					totalRewards+= r*pow(GAMMA,m_step);
					s_prime= GetStateId(m_pWorldState);
					//if (s!=s_prime)
					//	printf("harl");
				}
			}
			m_step++;
		}


		fopen_s(&pOutFile,logFile,"a");
		fopen_s(&pOutFile2,greedyLogFile,"a");

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
			fprintf(pOutFile2,"%d %.2f %d %d %d %f\n",m_episodeCount,epsilon,g_numVVStates
					,m_step,totalMoveCount,totalRewards);
			//printf("%d %.2f %d %d %d\n",m_episodeCount,m_epsilon,g_numVVStates,m_step,totalMoves,totalMoveCount);
		
		printf ("RUN #%d : %d/%d episodes\r",run,m_episodeCount,totalEpisodeCount);
		fclose(pOutFile);
		fclose(pOutFile2);


		epsilon= max(0.f,epsilon-epsilon_delta);
		//epsilon= 1.0-(double)totalMoveCount/(double)maxNumSteps;

	}
}
	//printf("first goal: %d",firstGoal);

	for (int i= 0; i<NUM_ACTIONS; i++)
		delete [] pTransitions[i];
	delete [] pVVStates;
	for (int i= 0; i<NUM_AGENTS; i++)
		delete [] pVetoes1[i];
	delete [] pVetoes2;
	delete [] pVetoes3;
//	m_pStatManager->PrintStats();
//	m_pStatManager->CloseStats();

}
	