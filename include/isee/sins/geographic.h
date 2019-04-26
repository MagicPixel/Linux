#pragma once
#ifdef WIN32
#pragma warning  (disable:4819)
#endif
#include "export.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace aoi{
namespace utillity{
namespace geographic{

	/******************�������WGS84******************/
	static const double  Earth_a = 6378137.0;                   //�����׼�����峤�뾶(m)
	static const double  Earth_f = 0.00335281066474748;  //��׼�����弫����   1.0 / 298.257223563
	static const double  Earth_we_ie = 7.2921151467e-5;        //�����Դ����ٶ� ��rad/s��
	static const double  Earth_g = 9.7803267714;              //����������ٶ�
	static const double  Earth_e2 = 0.00669437999014132; // e*e = f*(2-f)

	/******************ʱ��ת��******************/
	static const double   LEAP_SECOND = 18;  //������ obs-UTC=18s (Year:2018)
	double SINS_FUNCTION gps2UnixTime(const int& gps_week, const double& gps_second, const double& leap_second = LEAP_SECOND); //GpsתUnixʱ��
	double SINS_FUNCTION utc2UnixTime(const int& utc_week, const double& utc_second); //UtcתUnixʱ��
	void SINS_FUNCTION unix2GpsTime(const double& unix_time, int& gps_week, double& gps_second, const double& leap_second = LEAP_SECOND); //UnixתGpsʱ��
	void SINS_FUNCTION unix2UtcTime(const double& unix_time, int& utc_week, double& utc_second); //UnixתUtcʱ��
	void SINS_FUNCTION gps2UtcTime(const int& gps_week, const double& gps_second, int& utc_week, double& utc_second, const double& leap_second = LEAP_SECOND); //GpsתUtcʱ��
	void SINS_FUNCTION utc2GpsTime(const int& utc_week, const double& utc_second, int& gps_week, double& gps_second, const double& leap_second = LEAP_SECOND); //UtcתGpsʱ��

	/******************ŷ���ǡ���̬������Ԫ��ת������******************/
	//att:  pitch roll  yaw
	Eigen::Vector3d SINS_FUNCTION cnb2Att(Eigen::Matrix3d& Cnb); //  ��̬��ת��ŷ����
	Eigen::Vector3d SINS_FUNCTION quat2Att(Eigen::Quaterniond& qnb); //��Ԫ��תŷ����
	Eigen::Matrix3d SINS_FUNCTION att2Cnb(const Eigen::Vector3d& att); //����̬�ǵõ��������Ҿ���
	Eigen::Quaterniond SINS_FUNCTION att2Quat(const Eigen::Vector3d& att); //��ŷ���ǵõ���Ԫ��
	Eigen::Matrix3d SINS_FUNCTION skewSymMat(const Eigen::Vector3d& vec); // �������б�Գ���
	Eigen::Quaterniond SINS_FUNCTION rotVect2Quat(const Eigen::Vector3d& vec); //��Ч��תʸ��ת��Ԫ������Ԫ��������

}	// namespace geographic
}	// namespace utillity 
}	// namespace aoi
