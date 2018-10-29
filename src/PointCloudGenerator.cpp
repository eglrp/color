#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "TrjFile.h"
#include "PcapParse.h"

#if 0
#if 1
int main()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	PcapParse pcap;
	TrjFileParse trj;
	uint64_t veltime, trjtime;
	Eigen::Isometry3d tran;
	pcap.OpenPcapFile("H:\\handle_data\\20180516161958\\0.pcap");
	trj.open("H:\\handle_data\\20180516161958\\trj.txt");
	HeaderTrj header;
	BlockTrj trjline;
	trj.readHeader(header);
	trj.readBlock(trjline);
	trjtime = trjline.time;
	pcap.ReadPcapData(*cloud, veltime);
	int count = 0;
	int id = 0;
	const int submapnum = 37 * 20;
	char filename[255];
	Eigen::Isometry3d mat;
	while (1)
	{
		if (trjtime < veltime)
		{
			int ret = trj.readBlock(trjline);
			if (ret != 0)
				break;
			trjtime = trjline.time;
		}
		else if((trjtime > veltime))
		{
			int ret = pcap.ReadPcapData(*cloud, veltime);
			if (ret != 0)
				break;
		}

		if(trjtime == veltime)
		{
			Eigen::Quaterniond q(trjline.q[0], trjline.q[1], trjline.q[2], trjline.q[3]);
			tran = q;
			tran(0, 3) = trjline.pos[0];
			tran(1, 3) = trjline.pos[1];
			tran(2, 3) = trjline.pos[2];
			pcl::transformPointCloud(*cloud, *cloud, tran.matrix());
			*out += *cloud;
			count++;
			if (count == submapnum)
			{
				sprintf(filename, "H:\\handle_data\\20180516161958\\%d.pcd", id++);
				pcl::io::savePCDFileBinary(filename, *out);
				out->clear();
				count = 0;
			}
			int ret = pcap.ReadPcapData(*cloud, veltime);
			if (ret != 0)
				break;
		}
	}
	return 0;
}
#else

int main()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	PcapParse pcap;
	TrjFileParse trj;
	uint64_t veltime, trjtime;
	Eigen::Isometry3d tran;
	pcap.OpenPcapFile("F:\\LidarData\\Œ‰¥ÛºÏ–£≥°1216\\2016-12-16_03-51-12\\16_2016-12-16_11-49-22-553.pcap", true);
	trj.open("D:\\OpenSourceLib\\cartographer\\build\\Carotographer\\x64\\Release\\building.txt");
	HeaderTrj header;
	BlockTrj trjline;
	trj.readHeader(header);
	trj.readBlock(trjline);
	trjtime = trjline.time;
	pcap.ReadPcapData(*cloud, veltime);
	int count = 0;
	int id = 0;
	const int submapnum = 37 * 40 * 10;
	char filename[255];

	while (1)
	{
		if (trjtime < veltime)
		{
			int ret = trj.readBlock(trjline);
			if (ret != 0)
				break;
			trjtime = trjline.time;
		}
		else if (trjtime > veltime)
		{
			int ret = pcap.ReadPcapData(*cloud, veltime);
			if (ret != 0)
				break;
		}

		if (trjtime == veltime)
		{
			Eigen::Quaterniond q(trjline.q[0], trjline.q[1], trjline.q[2], trjline.q[3]);
			tran = q;
			tran(0, 3) = trjline.pos[0];
			tran(1, 3) = trjline.pos[1];
			tran(2, 3) = trjline.pos[2];
			pcl::transformPointCloud(*cloud, *cloud, tran.matrix());
			*out += *cloud;
			count++;
			if (count == submapnum)
			{
				sprintf(filename, "E:\\slam3d\\PointCloudGenerator\\build\\Release\\building/%d.pcd", id++);
				pcl::io::savePCDFileBinary(filename, *out);
				out->clear();
				count = 0;
			}
			int ret = pcap.ReadPcapData(*cloud, veltime);
			if (ret != 0)
				break;
		}

		
	}
	return 0;
}
#endif

#endif