#pragma once
#include <stdio.h>
#include <vector>
#include <map>
struct LoopInfo
{
	unsigned int posidx;
	unsigned int loopidx;
	float acc[3];
};
class LoopInfoFile
{
public:
	LoopInfoFile(void);

	virtual ~LoopInfoFile(void);

	int Open(const char *fileName);
	
	int GetData(std::map<unsigned int, std::vector<LoopInfo> > &loopInfoMap);
	
	void Close();

protected:
	FILE *_fp;
};