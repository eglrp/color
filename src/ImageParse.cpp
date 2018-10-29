#include "ImageParse.h"
#include <stdio.h>



ImageParse::ImageParse():capture(0)
	, fp_(0)
{
}

ImageParse::~ImageParse()
{
	close();
	if (capture)
	{
		delete capture;
	}
}

int ImageParse::Open(const char* video_filename, const char * timestamp_filename)
{
	if (!capture)
		capture = new cv::VideoCapture;

	if (capture)
	{
		if(!capture->open(video_filename))
		{
			return -1;
		}
	}

	if (fp_)
	{
		fclose(fp_);
	}
	
	fp_ = fopen(timestamp_filename, "r");
	if (!fp_)
		return -2;
	return 0;
}

int ImageParse::GetNextFrame()
{
	if (capture && capture->isOpened())
	{
		*capture >> curimage_;
		if (curimage_.empty())
			return -1;
		if (fp_)
		{
			if (!feof(fp_))
			{
				unsigned long long sec, usec;
				fscanf(fp_, "%lld %lld\n", &sec, &usec);
				timestamp_ = sec * 1000000ULL + usec;
				return 0;
			}
			return -2;
		}
	}
	return -3;
}

void ImageParse::GetCurrentFrame(cv::Mat &image, unsigned long long &timestamp)
{
	image = curimage_;
	timestamp = timestamp_;
}

void ImageParse::close()
{
	if (fp_)
	{
		fclose(fp_);
		fp_ = NULL;
	}
		
	if (capture && capture->isOpened())
		capture->release();
}