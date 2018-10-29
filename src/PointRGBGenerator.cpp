#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "TrjFile.h"
#include "PcapParse.h"
#include "ImageParse.h"
#include "PointColorisze.h"
#include <iostream>

using namespace std;

#if 0
int main()
{
	pcl::PointCloud<pcl::PointXYZI> submap, cloudblock, cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
	PcapParse pcap;
	ImageParse imgparse;
	TrjFileParse trj;
	cv::Mat img;
	cv::Mat intrinsicMat(3, 3, CV_64FC1), distortion_coeffs(5, 1, CV_64FC1);
	/*intrinsicMat.at<double>(0, 0) = 1.214183e+03;
	intrinsicMat.at<double>(0, 1) = 0;
	intrinsicMat.at<double>(0, 2) = 6.288512e+02;
	intrinsicMat.at<double>(1, 0) = 0;
	intrinsicMat.at<double>(1, 1) = 1.212959e+03;
	intrinsicMat.at<double>(1, 2) = 4.933558e+02;
	intrinsicMat.at<double>(2, 0) = 0;
	intrinsicMat.at<double>(2, 1) = 0;
	intrinsicMat.at<double>(2, 2) = 1;
	distortion_coeffs.at<double>(0, 0) = -1.675524e-01;
	distortion_coeffs.at<double>(1, 0) = 3.314823e-01;
	distortion_coeffs.at<double>(2, 0) = -1.348386e-03;
	distortion_coeffs.at<double>(3, 0) = -3.222424e-04;
	distortion_coeffs.at<double>(4, 0) = -4.336718e-01;*/

	intrinsicMat.at<double>(0, 0) = 1.209584e+03;
	intrinsicMat.at<double>(0, 1) = 0;
	intrinsicMat.at<double>(0, 2) = 6.245486e+02;
	intrinsicMat.at<double>(1, 0) = 0;
	intrinsicMat.at<double>(1, 1) = 1.209021e+03;
	intrinsicMat.at<double>(1, 2) = 4.976582e+02;
	intrinsicMat.at<double>(2, 0) = 0;
	intrinsicMat.at<double>(2, 1) = 0;
	intrinsicMat.at<double>(2, 2) = 1;
	distortion_coeffs.at<double>(0, 0) = -1.531023e-01;
	distortion_coeffs.at<double>(1, 0) = 1.673448e-01;
	distortion_coeffs.at<double>(2, 0) = -1.261768e-03;
	distortion_coeffs.at<double>(3, 0) = -1.963575e-04;
	distortion_coeffs.at<double>(4, 0) = 0;
	std::vector<Pidxrgb> rgbs;
	uint64_t veltime, imgtime, trjtime, pre_trjtime;
	Eigen::Isometry3d tran, prev_tran, img_tran;
	Eigen::Matrix4d extrinsic_tran;
	extrinsic_tran << -1.5479844970302451e-02, 9.9952788576146701e-01, -2.6540157966882631e-02, -1.1527147925306959e-01,
		-3.6745934037061841e-02, -2.7094096870933275e-02,	-9.9895727949021551e-01, -6.8916384004789985e-02,
		-9.9920473914580654e-01, -1.4488460924479361e-02,	3.7147998177567157e-02, -5.6053084341192945e-02,
		0., 0., 0., 1.;
	pcap.OpenPcapFile("E:\\slam3d\\PointCloudGenerator\\build\\Release\\1542\\1_1.pcap");
	trj.open("E:\\slam3d\\PointCloudGenerator\\build\\Release\\1542\\trj.txt");
	imgparse.Open("E:\\slam3d\\PointCloudGenerator\\build\\Release\\1542\\1.avi", "E:\\slam3d\\PointCloudGenerator\\build\\Release\\1542\\1.time");
	imgparse.GetNextFrame();
	HeaderTrj header;
	BlockTrj trjline;
	trj.readHeader(header);
	trj.readBlock(trjline);
	trjtime = trjline.time;
	pre_trjtime = trjtime;
	Eigen::Quaterniond q(trjline.q[0], trjline.q[1], trjline.q[2], trjline.q[3]);
	tran = q;
	tran(0, 3) = trjline.pos[0];
	tran(1, 3) = trjline.pos[1];
	tran(2, 3) = trjline.pos[2];

	pcap.ReadPcapData(cloudblock, veltime);
	imgparse.GetCurrentFrame(img, imgtime);
	int count = 0;
	int id = 0;
	const int submapnum = 37 * 40 * 10;
	char filename[255];
	Eigen::Isometry3d mat;
	bool isAccPoints = false;
	while (1)
	{
		if (trjtime < veltime)
		{
			int ret = trj.readBlock(trjline);
			if (ret != 0)
				break;

			prev_tran = tran;
			Eigen::Quaterniond q(trjline.q[0], trjline.q[1], trjline.q[2], trjline.q[3]);
			tran = q;
			tran(0, 3) = trjline.pos[0];
			tran(1, 3) = trjline.pos[1];
			tran(2, 3) = trjline.pos[2];

			pre_trjtime = trjtime;
			trjtime = trjline.time;
		}
		else if ((trjtime > veltime))
		{
			int ret = pcap.ReadPcapData(cloudblock, veltime);
			if (ret != 0)
				break;
		}

		if (trjtime == veltime)
		{
			if (isAccPoints)
			{
				pcl::transformPointCloud(cloudblock, cloudblock, tran.matrix());
				submap += cloudblock;
			}
			int ret = pcap.ReadPcapData(cloudblock, veltime);
			if (ret != 0)
				break;
		}

		if (trjtime > imgtime)
		{
			if (submap.size() > 0)
			{
				unsigned long long delta = trjtime - pre_trjtime;
				unsigned long long delta1 = imgtime - pre_trjtime;
				//计算插值比例
				float rate = (delta1*1e-6) / (delta*1e-6);

				//插值计算相机位置
				//img_tran = TransformationSlerp(prev_tran, tran, rate);
				img_tran = tran;
				//cv::imwrite("test.jpg", img);
				pcl::transformPointCloud(submap, cloud, (extrinsic_tran*img_tran.inverse().matrix()).cast<float>());
				PointColorize(cloud, intrinsicMat, distortion_coeffs, img, rgbs);
				//pcl::io::savePCDFileBinary("test.pcd", cloud);
				//转换为RGB点云数据
				pcl::PointXYZRGB po;

				for (size_t i = 0; i < rgbs.size(); i++)
				{
					unsigned int idx = rgbs[i].indices;
					unsigned int rgb = rgbs[i].rgb;
					pcl::PointXYZI &p = submap.points[idx];
					po.x = p.x;
					po.y = p.y;
					po.z = p.z;
					po.r = (rgb & 0x0ff0000) >> 16;
					po.g = (rgb & 0x0ff00) >> 8;
					po.b = rgb & 0x0ff;
					cloudrgb.push_back(po);
				}
				submap.clear();
				if (cloudrgb.size() == 0)
				{
					continue;
				}
				id++;
				//if (id % 1000 == 0)
				{
					sprintf(filename, "E:\\slam3d\\PointCloudGenerator\\build\\Release\\building/%d.pcd", id);
					pcl::io::savePCDFileBinary(filename, cloudrgb);
					cloudrgb.clear();
				}
			}

			//for (int i = 0; i < 10; i++)
			{
				if (imgparse.GetNextFrame()!= 0)
					return 0;
			}
			imgparse.GetCurrentFrame(img, imgtime);
		}
		else //if (pre_trjtime > imgtime)
		{
			

			isAccPoints = true;
		}
	}
	return 0;
}
#else
void getFiles(std::string path, std::string exd, std::vector<std::string>& files)
{
	//文件句柄  
	long long hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	std::string pathName, exdName;
	files.clear();
	if (0 != strcmp(exd.c_str(), ""))
	{
		exdName = "\\*." + exd;
	}
	else
	{
		exdName = "\\*";
	}

	if ((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(), &fileinfo)) != -1)
	{
		//do
		{
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					files.push_back(pathName.assign(path).append("\\").append(fileinfo.name));
			}
		}// while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

int main(int  argc, char*   argv[])
{
	
	std::string pcapfilename = ".\\0.pcap";
	std::string timefilename = ".\\0.pcap";
	std::string avifilename = ".\\0.pcap";
	//轨迹文件
	std::string trjfilename = ".\\trj.txt";


	if (argc == 3)
	{
		std::vector<std::string> files;

		//pcap文件
		getFiles(argv[1], "pcap", files);
		if (files.size() < 1)
		{
			printf("没有找到pcap文件\n");
			system("pause");
			return -1;
		}
		pcapfilename = files[0];

		//时间戳
		getFiles(argv[1], "time", files);
		if (files.size() < 1)
		{
			printf("没有找到time文件\n");
			system("pause");
			return -1;
		}
		timefilename = files[0];

		//采集视频
		getFiles(argv[1], "avi", files);
		if (files.size() < 1)
		{
			printf("没有找到avi文件\n");
			system("pause");
			return -1;
		}
		avifilename = files[0];

		//轨迹文件
		trjfilename = argv[2];
	}
	else
	{
		printf("请输入路径： PointCloudGenerator 工程路径 轨迹文件全路径\n");
		system("pause");
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZI> submap, cloudblock, cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloudrgb;
	PcapParse pcap;
	ImageParse imgparse;
	TrjFileParse trj;
	cv::Mat img;
	cv::Mat extrinsictran, K, xi, D;
	char exeFullPath[MAX_PATH]; // Full path   
	std::string strPath = "";
	GetModuleFileName(NULL, exeFullPath, MAX_PATH);
	strPath = (std::string)exeFullPath;    // Get full path of the file   
	int pos = strPath.find_last_of('\\', strPath.length());
	strPath = strPath.substr(0, pos);  // Return the directory without the file name 
	strPath = argv[1];
	printf("%s\n", strPath.c_str());
	cv::FileStorage fs(strPath+"\\LaserPara.yaml", cv::FileStorage::READ);
	if (!fs.isOpened()) {
		printf("无法找到LaserPara.yaml\n");
		system("pause");
		return -1;
	}

	fs["TransMatFromLaserToPano"] >> extrinsictran;
	fs.release();

	//外参
	cv::Matx44d ext = extrinsictran;
	cv::FileStorage fs1(strPath + "\\CameraPara.yaml", cv::FileStorage::READ);
	if (!fs1.isOpened()) {
		printf("无法找到CameraPara.yaml\n");
		system("pause");
		return -1;
	}

	fs1["K"] >> K;
	fs1["D"] >> D;
	fs1["Xi"] >> xi;
	fs1.release();

	//外参
	Eigen::Matrix4d ext_tran;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			ext_tran(i, j) = ext(i, j);
		}
	}
	
	std::vector<Pidxrgb> rgbs;
	uint64_t veltime, imgtime, trjtime, pre_trjtime;
	Eigen::Isometry3d tran, prev_tran, img_tran;

	pcap.OpenPcapFile(pcapfilename.c_str());
	trj.open(trjfilename.c_str());
	imgparse.Open(avifilename.c_str(), timefilename.c_str());
	imgparse.GetNextFrame();
	HeaderTrj header;
	BlockTrj trjline;
	trj.readHeader(header);
	trj.readBlock(trjline);
	trjtime = trjline.time;
	pre_trjtime = trjtime;
	//四元数
	Eigen::Quaterniond q(trjline.q[0], trjline.q[1], trjline.q[2], trjline.q[3]);
	tran = q;
	tran(0, 3) = trjline.pos[0];
	tran(1, 3) = trjline.pos[1];
	tran(2, 3) = trjline.pos[2];

	pcap.ReadPcapData(cloudblock, veltime);
	imgparse.GetCurrentFrame(img, imgtime);
	int count = 0;
	int id = 0;
	const int submapnum = 37 * 40 * 10;
	char filename[255];
	Eigen::Isometry3d mat;
	bool isAccPoints = true;

	while (1)
	{
		if (trjtime < veltime)
		{
			int ret = trj.readBlock(trjline);
			if (ret != 0)
				break;

			prev_tran = tran;
			Eigen::Quaterniond q(trjline.q[0], trjline.q[1], trjline.q[2], trjline.q[3]);
			tran = q;
			tran(0, 3) = trjline.pos[0];
			tran(1, 3) = trjline.pos[1];
			tran(2, 3) = trjline.pos[2];

			pre_trjtime = trjtime;
			trjtime = trjline.time;
		}
		else if ((trjtime > veltime))
		{
			int ret = pcap.ReadPcapData(cloudblock, veltime);
			if (ret != 0)
				break;
		}

		if (trjtime == veltime)
		{
			if (isAccPoints)
			{
				pcl::transformPointCloud(cloudblock, cloudblock, tran.matrix());
				submap += cloudblock;
			}
			int ret = pcap.ReadPcapData(cloudblock, veltime);
			if (ret != 0)
				break;
		}

		if (trjtime > imgtime)
		{
			if (submap.size() > 0)
			{
				unsigned long long delta = trjtime - pre_trjtime;
				unsigned long long delta1 = imgtime - pre_trjtime;
				//计算插值比例
				float rate = (delta1*1e-6) / (delta*1e-6);

				//插值计算相机位置
				//img_tran = TransformationSlerp(prev_tran, tran, rate);
				img_tran = tran;
				//cv::imwrite("test.jpg", img);
				pcl::transformPointCloud(submap, cloud, (ext_tran*img_tran.inverse().matrix()).cast<float>());
				//pcl::io::savePCDFileBinary("test.pcd", cloud);
				OmniPointColorize(cloud, K, xi, D, img, rgbs);
				
				//转换为RGB点云数据
				pcl::PointXYZRGB po;

				for (size_t i = 0; i < rgbs.size(); i++)
				{
					unsigned int idx = rgbs[i].indices;
					unsigned int rgb = rgbs[i].rgb;
					pcl::PointXYZI &p = submap.points[idx];
					po.x = p.x;
					po.y = p.y;
					po.z = p.z;
					po.r = (rgb & 0x0ff0000) >> 16;
					po.g = (rgb & 0x0ff00) >> 8;
					po.b = rgb & 0x0ff;
					cloudrgb.push_back(po);
				}
				submap.clear();
				if (cloudrgb.size() == 0)
				{
					continue;
				}
				id++;
				if (id % 200 == 0)
				{
					printf("%d\n", id);
					sprintf(filename, "/%d.pcd", id);
					std::string outfilename = argv[1];
					outfilename += filename;
					pcl::io::savePCDFileBinary(outfilename.c_str(), cloudrgb);
					cloudrgb.clear();
				}
			}

			for (int i = 0; i < 1; i++)
			{
				if (imgparse.GetNextFrame() != 0)
					return 0;
			}
			imgparse.GetCurrentFrame(img, imgtime);
		}
		else if (pre_trjtime > imgtime)
		{
			isAccPoints = true;
		}
	}
	if (cloudrgb.size() > 0)
	{

		sprintf(filename, "/%d.pcd", id);
		printf("%d\n", id);
		std::string outfilename = argv[1];
		outfilename += filename;
		pcl::io::savePCDFileBinary(outfilename.c_str(), cloudrgb);
		cloudrgb.clear();
	}
	
	return 0;
}
#endif