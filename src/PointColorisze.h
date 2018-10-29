#pragma once
#include <pcl/common/transforms.h>
#include <opencv2\opencv.hpp>

struct Pidxrgb
{
	unsigned int indices;
	unsigned int rgb;
};

void PointColorize(pcl::PointCloud<pcl::PointXYZI> &cloud, cv::Mat &intrinsic_matrix, cv::Mat &distortion_coeffs, cv::Mat &image,
	std::vector<Pidxrgb> &rgbs);

void OmniPointColorize(pcl::PointCloud<pcl::PointXYZI> &cloud, cv::Mat &K, cv::Mat &xi, cv::Mat &D, cv::Mat &image,
	std::vector<Pidxrgb> &rgbs);

Eigen::Isometry3d TransformationSlerp(Eigen::Isometry3d begin_mat, Eigen::Isometry3d end_mat, float rate);