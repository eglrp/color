#pragma once
//#pragma pack(4)
//旧格式
/*struct IMUData
{
	unsigned int week;
	double sec;
	float gyro[3];
	float acc[3];
};*/

/*struct IMUData
{
	unsigned int week;//系统时间
	double sec;
	float gyro[3];
	float acc[3];
	unsigned int utcweek;//传感器时间
	double utcsec;
};*/

struct IMUData
{
	unsigned int week;//系统时间
	double sec;
	float gyro[3];
	float acc[3];
	float mag[3];
	unsigned int utcweek;//传感器时间
	double utcsec;
};
//#pragma pack(pop)