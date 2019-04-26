#ifndef IADAPTROBUSTKF_H_4EADCF71_D892_4DD3_BBA6_750B3902A1BD__INCLUDE
#define IADAPTROBUSTKF_H_4EADCF71_D892_4DD3_BBA6_750B3902A1BD__INCLUDE
/*****************************************************************************
*  @file     iadaptrobustkf.h                                                         *
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

	class IAdaptRobustKf {
	public:
		//virtual ~IAdaptRobustKf() = 0;
		/*@brief set iae weight*/
		virtual void setIAEWeight(const double &forget_factor = 0.95) = 0;
		/*@brief calulate iae rk*/
		virtual double calIAERk(double *innova_res_cache, const double& innova_res, const double &Rmin, const double &Rmax) = 0;
		/*@brief calulate robust rk*/
		virtual double calRobustRk(const double innova, const double HPHR) const = 0;
		/*@brief calulate adapt weight*/
		virtual void  calAdaptWeight(const Eigen::VectorXd &Xk, const Eigen::VectorXd &Xk_new, const  Eigen::MatrixXd &Pk) = 0;

		/*@brief getter*/
		virtual int getFilterCount() const = 0;
		virtual double getAdaptWeight() const = 0;

		/*@brief setter*/
		virtual void setFilterCount(int count) = 0;
		virtual void setAdaptWeight(double weight) = 0;
		
	};


}	// namespace utillity 
}	// namespace aoi








#endif	// IADAPTROBUSTKF_H_4EADCF71_D892_4DD3_BBA6_750B3902A1BD__INCLUDE
