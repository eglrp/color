#pragma once
#include <stdio.h>
#include "IMUStruct.h"

class IMUFile
{
public:
	IMUFile(void);

	virtual ~IMUFile(void);

	int Open(const char *fileName);
	
	virtual int GetData(unsigned int &week , double &sec, float gyro[3], float acc[3]);
	
	virtual int GetData(IMUData &data);
	
	void Close();

protected:
	FILE *_fp;
};

