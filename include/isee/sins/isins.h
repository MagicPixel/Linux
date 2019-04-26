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
		// ����IMU������
		virtual void setImuFrequency(const int & imu_frequency = 100) = 0;
		//���ó�ʼ���ݺͼӼ���ƫ
		virtual void setInitialBias(Eigen::Vector3d& initial_gyro_bias, Eigen::Vector3d& initial_acc_bias) = 0;
		//������������Ĭ��Ϊ18��
		virtual void setLeapSecond(const double &leap_second_ = 18) = 0;
		// ���ó�ʼʱ�䣨Ĭ��Ϊgpsʱ�䣩
		virtual void setInitialTime(const double & gps_time_ = 0.0, const int & gps_week_ = 0) = 0;
		//�����ⲿ��Ϣ��ʼ��λ���ٶ���̬������̬��Ϣ��Ϊ��̬��ʼ�����˱�ֵĬ��Ϊ(0,0,0)
		virtual void setInitialState(Eigen::Vector3d& pos_, Eigen::Vector3d& vel_,
			Eigen::Vector3d& lever_arm_, Eigen::Vector3d att_ = Eigen::Vector3d(0, 0, 0)) = 0;

		/******************����������غ���******************/
		//��imu���ݽ��н����������(ע��ʱ��Ҳ��һ������)
		virtual void strapdownUpdate(Eigen::Vector3d& gyro, Eigen::Vector3d& acc) = 0;
		//�����ٶȼ���LLH����
		virtual Eigen::Vector3d vel2DletaLLH(const  Eigen::Vector3d& vel, const double ts = 1.0) const = 0;
		//�ٶȸ˱�ֵ����
		virtual Eigen::Vector3d leverArmCompensationOfVel(Eigen::Vector3d& lever_arm_b) const = 0;
		//λ�ø˱�ֵ����
		virtual Eigen::Vector3d leverArmCompensationOfPos(Eigen::Vector3d& lever_arm_b) const = 0;
		//�ٶȸ˱�ֵ����ϵ������
		virtual Eigen::Matrix3d velLeverArmCompensationCoef() const = 0;
		//λ�ø˱�ֵ����ϵ������
		virtual Eigen::Matrix3d posLeverArmCompensationCoef() const = 0;

		/******************������״̬����/У����غ���******************/
		//����sins״̬����ߵ����״̬���¾���Ft�е�Sins����
		//��Ӧ��15ά״̬����Xk˳�򣺸��� ��� ƫ��  ���� ���� ���� γ�� ���� �߳� ����X ����Y ����Z �Ӽ�X �Ӽ�Y �Ӽ�Z
		virtual Eigen::Matrix<double, 15, 15> calcKalmanFt() = 0;
		//����Xk����ֵ����ϵͳ״̬У��
		virtual void kalmanCorrection(Eigen::Matrix<double, 15, 1>& Xk) = 0;

		// ��������
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
