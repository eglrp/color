#include "PointColorisze.h"

using namespace cv;
void PointColorize(pcl::PointCloud<pcl::PointXYZI> &cloud, cv::Mat &intrinsic_matrix, cv::Mat &distortion_coeffs, cv::Mat &image,
	std::vector<Pidxrgb> &rgbs)
{
	double x, y, x1, y1;
	int u, v;
	Pidxrgb pidx;
	double r2, r4, r6;
	rgbs.clear();
	double fx = intrinsic_matrix.at<double>(0, 0);
	double fy = intrinsic_matrix.at<double>(1, 1);
	double cx = intrinsic_matrix.at<double>(0, 2);
	double cy = intrinsic_matrix.at<double>(1, 2);
	double k1 = distortion_coeffs.at<double>(0, 0);
	double k2 = distortion_coeffs.at<double>(1, 0);
	double p1 = distortion_coeffs.at<double>(2, 0);
	double p2 = distortion_coeffs.at<double>(3, 0);
	double k3 = distortion_coeffs.at<double>(4, 0);
	//k3 = 0;

	/*cv::Size imageSize;
	imageSize = image.size();
	cv::Mat view, rview, map1, map2, imgcal1;
	cv::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, cv::Mat(),
		cv::getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, imageSize, 1, imageSize, 0),
		imageSize, CV_16SC2, map1, map2);
	remap(image, imgcal1, map1, map2, cv::INTER_LINEAR);
	cv::imshow("show", imgcal1);
	cv::waitKey();*/

	for (size_t i = 0; i < cloud.size(); i++)
	{
		pcl::PointXYZI &p = cloud[i];
		if (p.z < 0) continue;
		/*if (p.z < 0)
		{
			pidx.indices = i;
			pidx.rgb = 0;
			rgbs.push_back(pidx);
			continue;
		}*/

		//归一化平面
		x = p.x / p.z;
		y = p.y / p.z;

		r2 = x*x + y*y;
		r4 = r2*r2;
		r6 = r4*r2;

		//畸变模型-----径向畸变+切向畸变
		x1 = x*(1 + k1*r2 + k2*r4 + k3*r6) + 2 * p1*x*y + p2*(r2 + 2*x*x);
		y1 = y*(1 + k1*r2 + k2*r4 + k3*r6) + 2 * p2*x*y + p1*(r2 + 2*y*y);

		u = fx*x1 + cx+1;
		v = fy*y1 + cy+1;

		if (u < 0 || u >= image.cols) continue;
		if (v < 0 || v >= image.rows) continue;
		/*if (u < 0 || u >= image.cols)
		{
			pidx.indices = i;
			pidx.rgb = 0;
			rgbs.push_back(pidx);
			continue;
		}
		if (v < 0 || v >= image.rows)
		{
			pidx.indices = i;
			pidx.rgb = 0;
			rgbs.push_back(pidx);
			continue;
		}*/
		cv::Vec3b rgb = image.at<cv::Vec3b>(v, u);
		pidx.indices = i;
		pidx.rgb = (rgb[2] << 16 | rgb[1] << 8 | rgb[0]);
		rgbs.push_back(pidx);
	}
}

void OmniPointColorize(pcl::PointCloud<pcl::PointXYZI> &cloud, cv::Mat &K, cv::Mat &xi, cv::Mat &D, cv::Mat &image,
	std::vector<Pidxrgb> &rgbs)
{
	int u, v;
	Pidxrgb pidx;
	rgbs.clear();

	Vec2d f, c;
	double s, xi_d;
	if (K.depth() == CV_32F)
	{
		Matx33f Kc = K;
		f = Vec2f(Kc(0, 0), Kc(1, 1));
		c = Vec2f(Kc(0, 2), Kc(1, 2));
		s = (double)Kc(0, 1);
	}
	else
	{
		Matx33d Kc = K;
		f = Vec2d(Kc(0, 0), Kc(1, 1));
		c = Vec2d(Kc(0, 2), Kc(1, 2));
		s = Kc(0, 1);
	}

	if (xi.depth() == CV_32F)
	{
		xi_d = xi.at<float>(0);
	}
	else
	{
		xi_d = xi.at<double>(0);
	}
	Vec4d kp = D.depth() == CV_32F ? (Vec4d)*D.ptr<Vec4f>() : *D.ptr<Vec4d>();
	double k1 = kp[0], k2 = kp[1];
	double p1 = kp[2], p2 = kp[3];
	for (size_t i = 0; i < cloud.size(); i++)
	{
		pcl::PointXYZI p = cloud[i];
		if (p.z < 0) continue;
		double norm = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
		p.x /= norm;
		p.y /= norm;
		p.z /= norm;
		// convert to normalized image plane
		Vec2d xu = Vec2d(p.x / (p.z + xi_d), p.y / (p.z + xi_d));

		// add distortion
		Vec2d xd;
		double r2 = xu[0] * xu[0] + xu[1] * xu[1];
		double r4 = r2*r2;

		xd[0] = xu[0] * (1 + k1*r2 + k2*r4) + 2 * p1*xu[0] * xu[1] + p2*(r2 + 2 * xu[0] * xu[0]);
		xd[1] = xu[1] * (1 + k1*r2 + k2*r4) + p1*(r2 + 2 * xu[1] * xu[1]) + 2 * p2*xu[0] * xu[1];

		u = f[0] * xd[0] + s*xd[1] + c[0];
		v = f[1] * xd[1] + c[1];

		if (u < 0 || u >= image.cols) continue;
		if (v < 0 || v >= image.rows) continue;
		
		cv::Vec3b rgb = image.at<cv::Vec3b>(v, u);
		pidx.indices = i;

		// pack r/g/b into rgb
		pidx.rgb = (rgb[2] << 16 | rgb[1] << 8 | rgb[0]);
		rgbs.push_back(pidx);
	}
}

Eigen::Isometry3d TransformationSlerp(Eigen::Isometry3d begin_mat, Eigen::Isometry3d end_mat, float rate)
{
	Eigen::Isometry3d mat;
	Eigen::Quaterniond q1, q2, q;
	q1 = begin_mat.rotation();
	q2 = end_mat.rotation();
	q = q2.slerp(rate, q1);
	mat = q;
	mat(0, 3) = rate*(end_mat(0, 3) - begin_mat(0, 3)) + begin_mat(0, 3);
	mat(1, 3) = rate*(end_mat(1, 3) - begin_mat(1, 3)) + begin_mat(1, 3);
	mat(2, 3) = rate*(end_mat(2, 3) - begin_mat(2, 3)) + begin_mat(2, 3);
	return mat;
}