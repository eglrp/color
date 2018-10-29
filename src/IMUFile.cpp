#include "IMUFile.h"
#include <string.h>
#include <stdio.h>
#include <boost/concept_check.hpp>

IMUFile::IMUFile(void):_fp(NULL)
{
}


IMUFile::~IMUFile(void)
{
	if(_fp)
	{
		fclose(_fp);
		_fp = NULL;
	}
}

int IMUFile::Open(const char *fileName)
{
	_fp = fopen(fileName, "rb");
	if(_fp)
	{
		return 0;
	}
	return -1;
}

int IMUFile::GetData(IMUData& data)
{
	if(!_fp)
		return -1;
	if(fread(&data, sizeof(IMUData), 1, _fp) == 0)
	  return -2;
	return 0;
}

int IMUFile::GetData(unsigned int &week , double &sec, float gyro[3], float acc[3])
{
	if(!_fp)
		return -1;

	IMUData line;
	fread(&line, sizeof(IMUData), 1, _fp);
	week = line.week;
	sec = line.sec;
	memcpy(gyro, line.gyro, 3*4);
	memcpy(acc, line.acc, 3*4);
	return 0;
}

void IMUFile::Close()
{
	if(_fp)
	{
		fclose(_fp);
		_fp = NULL;
	}
}
