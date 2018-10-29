#ifndef _PCAP_PARSE_H
#define _PCAP_PARSE_H

#include <pcap.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define HDL_DATA_PORT			2368
#define HDL_NUM_ROT_ANGLES		36001
#define HDL_LASER_PER_FIRING	32
#define HDL_MAX_NUM_LASERS		64
#define HDL_FIRING_PER_PKT		12


class PcapParse
{
	enum HDLBlock
	{
		BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
	};

	enum RETURNTYPE
	{
		STRONGEST = 0x37, LAST = 0x38, DUAL = 0x39
	};

	enum DEVICETYPE
	{
		HDL32 = 0x21, VLP16 = 0x22
	};

#pragma pack(push, 1)
	typedef struct HDLLaserReturn
	{
		unsigned short distance;
		unsigned char intensity;
	} HDLLaserReturn;
#pragma pack(pop)

	struct HDLFiringData
	{
		unsigned short blockIdentifier;
		unsigned short rotationalPosition;
		HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
	};

	struct HDLDataPacket
	{
		HDLFiringData firingData[HDL_FIRING_PER_PKT];
		unsigned int gpsTimestamp;
		unsigned char returnType;
		unsigned char deviceType;
	};

	struct HDLLaserCorrection
	{
		double azimuthCorrection;
		double verticalCorrection;
		double distanceCorrection;
		double verticalOffsetCorrection;
		double horizontalOffsetCorrection;
		double sinVertCorrection;
		double cosVertCorrection;
		double sinVertOffsetCorrection;
		double cosVertOffsetCorrection;
	};
#pragma pack(push, 1)
	struct GPSDataPacket
	{
		unsigned char unused0[198];
		unsigned int timeStamp;
		unsigned char unused1[4];
		unsigned char GPRMC[90];
		unsigned char unused2[216];
	};
#pragma pack(pop)

	HDLLaserCorrection laser_corrections_[HDL_LASER_PER_FIRING];

public:
	PcapParse();

	~PcapParse();

	int OpenPcapFile(const char *fileName, bool isUseGPSTime = false);

	int ReadPcapData(pcl::PointCloud<pcl::PointXYZI> &current_scan_xyzi_, unsigned long long &timestamp);

	int ReadGPSData(double &latitude, double &Longitude);

	void setMaxDistanceThreshlod(float max_dis_thd);

	void setMinDistanceThreshlod(float min_dis_thd);

	void ClosePcapFile();
  
private:
	pcap_t *pcap;

	float min_distance_threshold_;
	float max_distance_threshold_;

	double cos_lookup_table_[HDL_NUM_ROT_ANGLES];
	double sin_lookup_table_[HDL_NUM_ROT_ANGLES];
	unsigned int startTime;
	unsigned int prevtimeStamp_;

	bool isUseGPSTime_;
	
	DEVICETYPE deviceType_;

private:
	bool computeXYZI (pcl::PointXYZI& point, int azimuth, HDLLaserReturn laserReturn,
						HDLLaserCorrection correction);
	int toPointClouds (HDLDataPacket *dataPacket, pcl::PointCloud<pcl::PointXYZI> &current_scan_xyzi_);
};

#endif