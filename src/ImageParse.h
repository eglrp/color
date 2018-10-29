#pragma once
#include <opencv2\opencv.hpp>
struct _iobuf;

namespace cv
{
	class VideoCapture;
}

class ImageParse
{
public:
	ImageParse();

	~ImageParse();

	int Open(const char* video_filename, const char * timestamp_filename);

	int GetNextFrame();

	void GetCurrentFrame(cv::Mat &image, unsigned long long &timestamp);

	void close();

private:
	_iobuf *fp_;
	cv::Mat curimage_;
	unsigned long long timestamp_;
	cv::VideoCapture *capture;
};

