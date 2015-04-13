#include "stdafx.h"
#include "ParameterManager.h"

CParameterManager *g_pParameterManager= 0;

CParameterManager::CParameterManager()
{
	m_numParameters= 0;
}

CParameterManager::~CParameterManager()
{
}

int CParameterManager::GetParameterIndex(char* pName)
{
	int i= 0;
	while (i<m_numParameters && strcmp(m_parameters[i].name,pName))
		i++;

	if (i>=m_numParameters)
		return -1;
	return i;
}

int CParameterManager::LoadParameters(char* fileName)
{
	FILE* pFile;
	char line[MAX_FILE_LINE_SIZE];
	char parameterName[MAX_NAME_SIZE];
	double parameterValue;
	int error;
	int index;
	error= fopen_s(&pFile,fileName,"r");

	if(!error)
	{
		while (fgets(line,MAX_FILE_LINE_SIZE,pFile))//!feof(pFile))
		{
			if ( 2==sscanf_s(line,"NAME: %s VALUE: %lf\r\n"
				,parameterName,MAX_NAME_SIZE
				,&parameterValue) )
			{
				index = GetParameterIndex(parameterName);
				if (index<0)
				{
					//parametro berria
					strcpy_s(m_parameters[m_numParameters].name,MAX_NAME_SIZE,parameterName);
					m_parameters[m_numParameters].value= parameterValue;
					m_numParameters++;
				}
				else
				{
					//parametro zaharra eguneratu
					m_parameters[index].value= parameterValue;
				}
			}
		}
		fclose(pFile);
		return m_numParameters;
	}
	return -1;
}

int CParameterManager::SaveParameters(char *fileName)
{
	FILE* pFile;
	int error;
	int i;
	error= fopen_s(&pFile,fileName,"w");

	if(!error)
	{
		for (i=0; i<m_numParameters; i++)
		{
			fprintf(pFile,"NAME: %s VALUE: %lf\n"
				,m_parameters[i].name
				,m_parameters[i].value);
		}
		fclose(pFile);
		return m_numParameters;
	}
	return -1;
}


int CParameterManager::GetIntParameter(char *name)
{
	int index;
	index= GetParameterIndex(name);
	if (index>=0)
		return (int)m_parameters[index].value;

	printf("ERROR: parameter %s not defined\n",name);
	return -1;
}

int CParameterManager::SetIntParameter(char *name, int value)
{
	int index;
	index= GetParameterIndex(name);
	if (index>=0)
	{
#ifdef _DEBUG
		printf("%s parameter's value changed to %d\n",name, value);
#endif
		m_parameters[index].value= (double)value;
		return 1;
	}
	return -1;
}

double CParameterManager::GetDoubleParameter(char* name)
{
	int index;
	index= GetParameterIndex(name);
	if (index>=0)
		return m_parameters[index].value;

	printf("ERROR: parameter %s not defined\n",name);
	return -1;
}
int CParameterManager::SetDoubleParameter(char* name, double value)
{
	int index;
	index= GetParameterIndex(name);
	if (index>=0)
	{
#ifdef _DEBUG
		printf("%s parameter's value changed to %.2f\n",name, value);
#endif
		m_parameters[index].value= value;
		return 1;
	}
	return -1;
}