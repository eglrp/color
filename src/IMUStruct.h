#pragma once
//#pragma pack(4)
//�ɸ�ʽ
/*struct IMUData
{
	unsigned int week;
	double sec;
	float gyro[3];
	float acc[3];
};*/

/*struct IMUData
{
	unsigned int week;//ϵͳʱ��
	double sec;
	float gyro[3];
	float acc[3];
	unsigned int utcweek;//������ʱ��
	double utcsec;
};*/

struct IMUData
{
	unsigned int week;//ϵͳʱ��
	double sec;
	float gyro[3];
	float acc[3];
	float mag[3];
	unsigned int utcweek;//������ʱ��
	double utcsec;
};
//#pragma pack(pop)