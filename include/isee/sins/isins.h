#ifndef ISINS_H_682A67FB_98EE_4E30_9913_5F18C3497CDE_INCLUDE
#define ISINS_H_682A67FB_98EE_4E30_9913_5F18C3497CDE_INCLUDE
/*****************************************************************************
*  @file     isins.h                                                         *
*  @brief    algorithm library                                               *
*  Details.                                                                  *
*                                                                            *
*  @author   Yan                                                          *
*  @version  1.0.0.1(version number)                                         *
*  @date     2017.09.06                                                      *
*  @license  none                                                            *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2017/09/06 | 1.0.0.0   | Yan      | create file                        *
*  2017/12/26 | 1.0.0.1   | Yan      | modify file                        *
*----------------------------------------------------------------------------*
*                                                                            *
*/
#ifdef WIN32
#pragma warning  (disable:4819)
#endif
#include<Eigen/Core>
#include<Eigen/Dense>

namespace aoi{
namespace utillity{

	class ISins {
	public:
		//virtual ~ISins() = 0;
		// 设置IMU采样率
		virtual void setImuFrequency(const int & imu_frequency = 100) = 0;
		//设置初始陀螺和加计零偏
		virtual void setInitialBias(Eigen::Vector3d& initial_gyro_bias, Eigen::Vector3d& initial_acc_bias) = 0;
		//设置闰秒数（默认为18）
		virtual void setLeapSecond(const double &leap_second_ = 18) = 0;
		// 设置初始时间（默认为gps时间）
		virtual void setInitialTime(const double & gps_time_ = 0.0, const int & gps_week_ = 0) = 0;
		//利用外部信息初始化位置速度姿态，无姿态信息则为动态初始化，杆臂值默认为(0,0,0)
		virtual void setInitialState(Eigen::Vector3d& pos_, Eigen::Vector3d& vel_,
			Eigen::Vector3d& lever_arm_, Eigen::Vector3d att_ = Eigen::Vector3d(0, 0, 0)) = 0;

		/******************捷联解算相关函数******************/
		//由imu数据进行捷联解算更新(注意时间也会一并更新)
		virtual void strapdownUpdate(Eigen::Vector3d& gyro, Eigen::Vector3d& acc) = 0;
		//根据速度计算LLH增量
		virtual Eigen::Vector3d vel2DletaLLH(const  Eigen::Vector3d& vel, const double ts = 1.0) const = 0;
		//速度杆臂值补偿
		virtual Eigen::Vector3d leverArmCompensationOfVel(Eigen::Vector3d& lever_arm_b) const = 0;
		//位置杆臂值补偿
		virtual Eigen::Vector3d leverArmCompensationOfPos(Eigen::Vector3d& lever_arm_b) const = 0;
		//速度杆臂值补偿系数矩阵
		virtual Eigen::Matrix3d velLeverArmCompensationCoef() const = 0;
		//位置杆臂值补偿系数矩阵
		virtual Eigen::Matrix3d posLeverArmCompensationCoef() const = 0;

		/******************卡尔曼状态更新/校正相关函数******************/
		//根据sins状态计算惯导相关状态更新矩阵Ft中的Sins部分
		//对应的15维状态向量Xk顺序：俯仰 横滚 偏航  东速 北速 天速 纬度 经度 高程 陀螺X 陀螺Y 陀螺Z 加计X 加计Y 加计Z
		virtual Eigen::Matrix<double, 15, 15> calcKalmanFt() = 0;
		//根据Xk反馈值进行系统状态校正
		virtual void kalmanCorrection(Eigen::Matrix<double, 15, 1>& Xk) = 0;

		// 拷贝函数
		virtual void copy(const ISins* rhs) = 0;

		/*@brief getter*/
		virtual int getWeek() const = 0;
		virtual double getTime() const = 0;
		virtual double getUnixTime() const = 0;
		virtual const Eigen::Vector3d& getPos() const = 0;
		virtual const Eigen::Vector3d& getVel() const = 0;
		virtual const Eigen::Vector3d& getAtt() const = 0;
		virtual const Eigen::Quaterniond& getQnb() const = 0;
		virtual const Eigen::Vector3d& getGyroBias() const = 0;
		virtual const Eigen::Vector3d& getAccBias() const = 0;
		virtual const Eigen::Matrix3d& getCnb() const = 0;


		/*@brief setter*/
		virtual void setWeek(int i) = 0;
		virtual void setTime(double d) = 0;
		virtual void setUnixTime(double d) = 0;
		virtual void setPos(const Eigen::Vector3d& vec) = 0;
		virtual void setVel(const Eigen::Vector3d& vec) = 0;
		virtual void setAtt(const Eigen::Vector3d& vec) = 0;
		virtual void setQnb(const Eigen::Quaterniond& quater) = 0;
		virtual void setGyroBias(const Eigen::Vector3d& vec) = 0;
		virtual void setAccBias(const Eigen::Vector3d& vec) = 0;
		virtual void setCnb(const Eigen::Matrix3d& mtr) = 0;
	};


}	// namespace utillity 
}	// namespace aoi











#endif	// ISINS_H_682A67FB_98EE_4E30_9913_5F18C3497CDE_INCLUDE
