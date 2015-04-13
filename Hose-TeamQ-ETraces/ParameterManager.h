#ifndef __PARAMETERMANAGER__
#define __PARAMETERMANAGER__

#define MAX_NAME_SIZE 64
#define MAX_FILE_LINE_SIZE 128
#define MAX_NUM_PARAMETERS 100

struct SParameter
{
	char name[MAX_NAME_SIZE];
	double value;
};

class CParameterManager
{
	SParameter m_parameters[MAX_NUM_PARAMETERS];
	int m_numParameters;

	int GetParameterIndex(char *pName);
public:
	CParameterManager();
	~CParameterManager();

	int LoadParameters(char* fileName);
	int SaveParameters(char* fileName);

	int GetIntParameter(char* name);
	int SetIntParameter(char* name, int value);

	double GetDoubleParameter(char* name);
	int SetDoubleParameter(char* name, double value);
//	int IncIntParameter(char* name,int maxValue,int incValue= 1);
//	int DecIntParameter(char* name,int minValue,int decValue= 1);
};

#endif