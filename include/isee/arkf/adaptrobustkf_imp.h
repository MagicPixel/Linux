#ifndef ADAPTROBUSTKF_IMP_H_0125B0AE_DB66_40A3_8845_6C4308E41CCA__INCLUDE
#define ADAPTROBUSTKF_IMP_H_0125B0AE_DB66_40A3_8845_6C4308E41CCA__INCLUDE
/*****************************************************************************
*  @file     adaptrobustkf_imp.h                                                      *
*  @brief    algorithm library implement                                     *
*  Details.                                                                  *
*                                                                            *
*  @author   Jocker                                                          *
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
*  2017/09/06 | 1.0.0.0   | Jocker      | create file                        *
*  2017/12/26 | 1.0.0.1   | Jocker      | modify file                        *
*----------------------------------------------------------------------------*
*                                                                            *
*/
#include "export.h"
#include "iadaptrobustkf.h"


namespace aoi{
	namespace utillity{


		class ARKF_CLASS AdaptRobustKfImp{
		public:
			AdaptRobustKfImp() = delete;
			static IAdaptRobustKf* create();
			static void destory(IAdaptRobustKf* p);
		};



	}	// namespace utillity 
}	// namespace aoi








#endif	// ADAPTROBUSTKF_IMP_H_0125B0AE_DB66_40A3_8845_6C4308E41CCA__INCLUDE