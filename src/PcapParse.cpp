#define _USE_MATH_DEFINES
#include <math.h>
#include <sstream>
#include <string.h>

#include "PcapParse.h"
#include "GPRMCInfo.h"


#define HDL_Grabber_toRadians(x) ((x) * M_PI / 180.0)
 
bool PcapParse::computeXYZI (pcl::PointXYZI &point,
                              int azimuth,
                              HDLLaserReturn laserReturn,
                              HDLLaserCorrection correction)
{
  double cos_azimuth, sin_azimuth;

  double distanceM = laserReturn.distance * 0.002;

  point.intensity = static_cast<float> (laserReturn.intensity);
  if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_)
  {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
    return false;
  }

  if (correction.azimuthCorrection == 0)
  {
    cos_azimuth = cos_lookup_table_[azimuth];
    sin_azimuth = sin_lookup_table_[azimuth];
  }
  else
  {
    double azimuthInRadians = HDL_Grabber_toRadians( (static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
    cos_azimuth = std::cos (azimuthInRadians);
    sin_azimuth = std::sin (azimuthInRadians);
  }

  distanceM += correction.distanceCorrection;

  double xyDistance = distanceM * correction.cosVertCorrection;

  point.x = static_cast<float> (xyDistance * sin_azimuth - correction.horizontalOffsetCorrection * cos_azimuth);
  point.y = static_cast<float> (xyDistance * cos_azimuth + correction.horizontalOffsetCorrection * sin_azimuth);
  point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.verticalOffsetCorrection);
  return true;
}

int PcapParse::toPointClouds (HDLDataPacket *dataPacket, pcl::PointCloud<pcl::PointXYZI> &current_scan_xyzi_)
{
	static uint32_t scan_counter = 0;
	static uint32_t sweep_counter = 0;
	current_scan_xyzi_.clear();
	if (sizeof(HDLLaserReturn) != 3)
	return -1;
  
	scan_counter++;

	if(deviceType_ == VLP16)
	{
		unsigned short rotation1 = 0, rotation2 = 0, delta = 40;
		pcl::PointXYZI xyzi;
		for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
		{
			HDLFiringData firing_data = dataPacket->firingData[i];
			int offset = (firing_data.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

			rotation1 = firing_data.rotationalPosition;
			if (i < HDL_FIRING_PER_PKT - 1)
			{
				unsigned short nextRotationPostion = dataPacket->firingData[i + 1].rotationalPosition;
				if (nextRotationPostion < firing_data.rotationalPosition)
				{
					nextRotationPostion += 36000;
				}
				delta = (nextRotationPostion - rotation1) / 2;
			}
			rotation2 = rotation1 + delta;
			if (rotation2 > 36000)
			{
				rotation2 -= 36000;
			}

			for (int j = 0; j < HDL_LASER_PER_FIRING; j++)
			{
				if(j > 15)
					computeXYZI (xyzi, rotation2, firing_data.laserReturns[j],  laser_corrections_[j]);
				else
					computeXYZI (xyzi, rotation1, firing_data.laserReturns[j],  laser_corrections_[j]);
				current_scan_xyzi_.push_back (xyzi);
			}
		}
	}
	else
	{
		pcl::PointXYZI xyzi;
		for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
		{
			HDLFiringData firing_data = dataPacket->firingData[i];
			int offset = (firing_data.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;
			for (int j = 0; j < HDL_LASER_PER_FIRING; j++)
			{
				computeXYZI (xyzi, firing_data.rotationalPosition, firing_data.laserReturns[j],  laser_corrections_[j]);
				current_scan_xyzi_.push_back (xyzi);
			}
		}
	}
	return 0;
}

PcapParse::PcapParse():pcap(NULL)
    ,isUseGPSTime_(false)
	,min_distance_threshold_(0.0f)
	,max_distance_threshold_(1000.0f)
{
	for (int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
	{
		double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
		cos_lookup_table_[i] = std::cos (rad);
		sin_lookup_table_[i] = std::sin (rad);
	}
}

PcapParse::~PcapParse()
{
  if(pcap)
  {
    pcap_close(pcap);
    pcap = NULL;
  }
}

int  PcapParse::OpenPcapFile (const char *fileName, bool isUseGPSTime)
{
	char errbuff[PCAP_ERRBUF_SIZE];
	pcap = pcap_open_offline (fileName, errbuff);
	struct bpf_program filter;
	std::ostringstream string_stream;

	if(!pcap)
		return -1;
    
	string_stream << "udp";
	// PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
	if (pcap_compile (pcap, &filter, string_stream.str ().c_str(), 0, 0xffffffff) == -1)
	{
		//PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
	}
	else if (pcap_setfilter(pcap, &filter) == -1)
	{
		//PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
	}
  
	int isReadLineData = 0;
	deviceType_ = VLP16;
	prevtimeStamp_ = 3600*1e6+1;
	isUseGPSTime_ = isUseGPSTime;
	while (1)
	{
		struct pcap_pkthdr *header;
		const unsigned char *data;
		int returnValue = pcap_next_ex(pcap, &header, &data);
		if (returnValue >= 0)
		{
			size_t bytesReceived = header->len - 42;
			if (bytesReceived == 1206 && isReadLineData == 0)
			{
				isReadLineData = 1;
				deviceType_ = (DEVICETYPE)data[1205 + 42];
				pcap_close(pcap);
				pcap = pcap_open_offline(fileName, errbuff);
				if (pcap_compile(pcap, &filter, string_stream.str().c_str(), 0, 0xffffffff) == -1)
				{
					//PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr (pcap));
				}
				else if (pcap_setfilter(pcap, &filter) == -1)
				{
					//PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr (pcap));
				}
			}
			else if (bytesReceived == 512 && isReadLineData == 1)
			{
				GPRMCInfo info;
				unsigned char dup[512];
				memcpy(dup, data + 42, bytesReceived * sizeof(unsigned char));
				GPSDataPacket *packet = reinterpret_cast<GPSDataPacket*>(dup);
				if (ParseGPSInfo(packet->GPRMC, &info) == 0)
				{
					tm t;
					t.tm_year = 80;
					t.tm_mon = 0;
					t.tm_mday = 6;
					t.tm_hour = 0;
					t.tm_min = 0;
					t.tm_sec = 0;
					uint64_t gpsbegin = mktime(&t);
					char chrnum[3] = { 0, 0, 0 };
					chrnum[0] = info.timeStamp[10];
					chrnum[1] = info.timeStamp[11];
					t.tm_year = 100 + atoi(chrnum);
					chrnum[0] = info.timeStamp[8];
					chrnum[1] = info.timeStamp[9];
					t.tm_mon = atoi(chrnum) - 1;
					chrnum[0] = info.timeStamp[6];
					chrnum[1] = info.timeStamp[7];
					t.tm_mday = atoi(chrnum);
					chrnum[0] = info.timeStamp[0];
					chrnum[1] = info.timeStamp[1];
					t.tm_hour = atoi(chrnum);
					t.tm_min = 0;
					t.tm_sec = 0;
					startTime = mktime(&t) - gpsbegin;
					isReadLineData = 2;
				}
				else
				{
					startTime = 0;
				}
				
			}

			if (isUseGPSTime)
			{
				if (isReadLineData == 2)
					break;
			}
			else if (isReadLineData == 1)
			{
				isReadLineData = 2;
				break;
			}
				
			
		}
		else
			break;
	}

	if(isReadLineData == 2)
	{
		if(deviceType_ == VLP16)
		{
			const double hdl16_vertical_corrections[] = { -0.2617993877991494, 0.017453292519943295,    -0.22689280275926285, 0.05235987755982989,
				-0.19198621771937624, 0.08726646259971647, -0.15707963267948966, 0.12217304763960307, 
				-0.12217304763960307, 0.15707963267948966, -0.08726646259971647, 0.19198621771937624,
				-0.05235987755982989,  0.22689280275926285, -0.017453292519943295, 0.2617993877991494};
			for (int i = 0; i < 16; i++)
			{
				laser_corrections_[i].azimuthCorrection = 0.0;
				laser_corrections_[i].distanceCorrection = 0.0;
				laser_corrections_[i].horizontalOffsetCorrection = 0.0;
				laser_corrections_[i].verticalOffsetCorrection = 0.0;
				laser_corrections_[i].verticalCorrection = hdl16_vertical_corrections[i];
				laser_corrections_[i].sinVertCorrection = std::sin (hdl16_vertical_corrections[i]);
				laser_corrections_[i].cosVertCorrection = std::cos (hdl16_vertical_corrections[i]);
			}

			for (int i = 16; i < HDL_LASER_PER_FIRING; i++)
			{
				laser_corrections_[i].azimuthCorrection = 0.0;
				laser_corrections_[i].distanceCorrection = 0.0;
				laser_corrections_[i].horizontalOffsetCorrection = 0.0;
				laser_corrections_[i].verticalOffsetCorrection = 0.0;
				laser_corrections_[i].verticalCorrection = hdl16_vertical_corrections[i-16];
				laser_corrections_[i].sinVertCorrection = std::sin (hdl16_vertical_corrections[i-16]);;
				laser_corrections_[i].cosVertCorrection = std::cos (hdl16_vertical_corrections[i-16]);
			}
		}
		else if(deviceType_ == HDL32)
		{
			const double hdl32VerticalCorrections[] = { 
				-30.67, -9.3299999, -29.33, -8, -28,
				-6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
				-1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
				-14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };
			for (int i = 0; i < 32; i++)
			{
				laser_corrections_[i].azimuthCorrection = 0.0;
				laser_corrections_[i].distanceCorrection = 0.0;
				laser_corrections_[i].horizontalOffsetCorrection = 0.0;
				laser_corrections_[i].verticalOffsetCorrection = 0.0;
				laser_corrections_[i].verticalCorrection = hdl32VerticalCorrections[i];
				double rad = HDL_Grabber_toRadians(hdl32VerticalCorrections[i]);
				laser_corrections_[i].sinVertCorrection = std::sin (rad);
				laser_corrections_[i].cosVertCorrection = std::cos (rad);
			}
		}
		return 0;
		
	}
	else
	{
		return -1;
	}
}

int PcapParse::ReadPcapData(pcl::PointCloud<pcl::PointXYZI> &current_scan_xyzi_, unsigned long long &timestamp)
{
  current_scan_xyzi_.clear();
  if(!pcap)
    return -1;
  
  struct pcap_pkthdr *header;
  const unsigned char *data;
  int ret = 0;
  while(1)
  {
    int returnValue = pcap_next_ex(pcap, &header, &data);
    if(returnValue >= 0)
    {    
      size_t bytesReceived = header->len - 42;
      if(bytesReceived == 1206 && (data))
      {
		unsigned char dup[1206];
		memcpy (dup, data + 42, bytesReceived * sizeof(unsigned char));
		HDLDataPacket *packet = reinterpret_cast<HDLDataPacket*>(dup);	
		toPointClouds(packet, current_scan_xyzi_);
		if (isUseGPSTime_)
		{
			unsigned int curtimestamp = packet->gpsTimestamp;
			if (prevtimeStamp_ < 3600 * 1e6  && prevtimeStamp_ >  curtimestamp)
				startTime += 3600;
			prevtimeStamp_ = curtimestamp;
			timestamp = curtimestamp + startTime * 1000000ll;
		}
		else
		{
			timestamp = header->ts.tv_sec * 1000000ULL + header->ts.tv_usec;
		}
		current_scan_xyzi_.header.stamp = timestamp;
		ret = 0;
		break;
      }
    }
    else
    {
      ret = -1;
      break;
    }
  }
  return ret;
}

int PcapParse::ReadGPSData(double &latitude, double &longitude)
{
	if(!pcap)
		return -1;

	struct pcap_pkthdr *header;
	const unsigned char *data;
	int returnValue = pcap_next_ex(pcap, &header, &data);
	if(returnValue >= 0)
	{    
		size_t bytesReceived = header->len - 42;
		if(bytesReceived == 512 && (data))
		{
			GPRMCInfo info;
			unsigned char dup[512];
			memcpy (dup, data + 42, bytesReceived * sizeof(unsigned char));
			GPSDataPacket *packet = reinterpret_cast<GPSDataPacket*>(dup);
			if(ParseGPSInfo(packet->GPRMC, &info)==0)
			{
				latitude = info.latitude;
				longitude = info.longitude;
				return 0;
			}
			return 2;
			
		}
		else
			return 1;
	}
	else
		return -1;
}

void PcapParse::ClosePcapFile()
{
  if(pcap)
  {
    pcap_close(pcap);
    pcap = NULL;
  }
}

void PcapParse::setMaxDistanceThreshlod(float max_dis_thd)
{
	max_distance_threshold_ = max_dis_thd;
}

void PcapParse::setMinDistanceThreshlod(float min_dis_thd)
{
	min_distance_threshold_ = min_dis_thd;
}