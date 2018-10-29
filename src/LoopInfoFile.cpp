#include "LoopInfoFile.h"

LoopInfoFile::LoopInfoFile(void):_fp(NULL)
{
}


LoopInfoFile::~LoopInfoFile(void)
{
	if(_fp)
	{
		fclose(_fp);
		_fp = NULL;
	}
}

int LoopInfoFile::Open(const char *fileName)
{
	_fp = fopen(fileName, "r");
	if(_fp)
	{
		return 0;
	}
	return -1;
}

int LoopInfoFile::GetData(std::map<unsigned int, std::vector<LoopInfo> > &loopInfoMap)
{
	if(!_fp)
		return -1;

	LoopInfo info;
	while(!feof(_fp))
	{
		unsigned int loopidx, size;
		fscanf(_fp, "%d %d\n", &loopidx, &size);
		for(unsigned int i = 0;i < size;i++)
		{
			fscanf(_fp, "%d %f %f %f\n", &info.posidx, info.acc, info.acc+1, info.acc+2);
			loopInfoMap[loopidx].push_back(info);
		}
	}
	Close();
	return 0;
}

void LoopInfoFile::Close()
{
	if(_fp)
	{
		fclose(_fp);
		_fp = NULL;
	}
}