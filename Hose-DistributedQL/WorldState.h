#ifndef __WORLD_STATE__
#define __WORLD_STATE__

struct Position
{
int m_x;
int m_y;
};

#define NUM_AGENTS 4

#define MAX_REL_DIST 4.

class CWorldState
{
public:
	Position m_positions[NUM_AGENTS];

	CWorldState();
	~CWorldState();
	void Init()
	{
		m_positions[0].m_x= 1;
		m_positions[0].m_y= 2;
		m_positions[1].m_x= 3;
		m_positions[1].m_y= 1;
		m_positions[2].m_x= 2;
		m_positions[2].m_y= 3;
		m_positions[3].m_x= 2;
		m_positions[3].m_y= 5;/**/

	}
};
#endif