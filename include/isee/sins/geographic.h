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

	/******************地球参数WGS84******************/
	static const double  Earth_a = 6378137.0;                   //地球基准椭球体长半径(m)
	static const double  Earth_f = 0.00335281066474748;  //基准椭球体极扁率   1.0 / 298.257223563
	static const double  Earth_we_ie = 7.2921151467e-5;        //地球自传角速度 （rad/s）
	static const double  Earth_g = 9.7803267714;              //赤道重力加速度
	static const double  Earth_e2 = 0.00669437999014132; // e*e = f*(2-f)

	/******************时间转换******************/
	static const double   LEAP_SECOND = 18;  //闰秒数 obs-UTC=18s (Year:2018)
	double SINS_FUNCTION gps2UnixTime(const int& gps_week, const double& gps_second, const double& leap_second = LEAP_SECOND); //Gps转Unix时间
	double SINS_FUNCTION utc2UnixTime(const int& utc_week, const double& utc_second); //Utc转Unix时间
	void SINS_FUNCTION unix2GpsTime(const double& unix_time, int& gps_week, double& gps_second, const double& leap_second = LEAP_SECOND); //Unix转Gps时间
	void SINS_FUNCTION unix2UtcTime(const double& unix_time, int& utc_week, double& utc_second); //Unix转Utc时间
	void SINS_FUNCTION gps2UtcTime(const int& gps_week, const double& gps_second, int& utc_week, double& utc_second, const double& leap_second = LEAP_SECOND); //Gps转Utc时间
	void SINS_FUNCTION utc2GpsTime(const int& utc_week, const double& utc_second, int& gps_week, double& gps_second, const double& leap_second = LEAP_SECOND); //Utc转Gps时间

	/******************欧拉角、姿态矩阵、四元数转换函数******************/
	//att:  pitch roll  yaw
	Eigen::Vector3d SINS_FUNCTION cnb2Att(Eigen::Matrix3d& Cnb); //  姿态阵转换欧拉角
	Eigen::Vector3d SINS_FUNCTION quat2Att(Eigen::Quaterniond& qnb); //四元数转欧拉角
	Eigen::Matrix3d SINS_FUNCTION att2Cnb(const Eigen::Vector3d& att); //由姿态角得到方向余弦矩阵
	Eigen::Quaterniond SINS_FUNCTION att2Quat(const Eigen::Vector3d& att); //由欧拉角得到四元数
	Eigen::Matrix3d SINS_FUNCTION skewSymMat(const Eigen::Vector3d& vec); // 向量叉乘斜对称阵
	Eigen::Quaterniond SINS_FUNCTION rotVect2Quat(const Eigen::Vector3d& vec); //等效旋转矢量转四元数（四元数迭代）

}	// namespace geographic
}	// namespace utillity 
}	// namespace aoi
