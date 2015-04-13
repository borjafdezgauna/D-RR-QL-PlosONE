#ifndef __WORLD_STATE__
#define __WORLD_STATE__

struct Position
{
int m_x;
int m_y;
};


//HOSE MOD//////////////////////////////
#define MAX_NUM_ROBOTS 10
#define NUM_AGENTS 4
#define GOAL_X -2
#define GOAL_Y -2
#define MAX_REL_DIST 4

class CWorldState
{
public:
	Position m_positions[MAX_NUM_ROBOTS];

	CWorldState(int numRobots){}
	~CWorldState(){}
	void Init()
	{
		m_positions[0].m_x= 1;
		m_positions[0].m_y= 2;
		m_positions[1].m_x= 3;
		m_positions[1].m_y= 1;
		m_positions[2].m_x= 2;
		m_positions[2].m_y= 3;
		m_positions[3].m_x= 2;
		m_positions[3].m_y= 5;
		/*m_positions[4].m_x= 2;
		m_positions[4].m_y= 3;*/
	}
};
/////////////////////////////////////////

#endif
