#ifndef SINS_IMP_E7AC1909_E096_4016_BBB6_AB7F06AF7B41__INCLUDE
#define SINS_IMP_E7AC1909_E096_4016_BBB6_AB7F06AF7B41__INCLUDE
/*****************************************************************************
*  @file     sins_imp.h                                                      *
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
#include "isins.h"


namespace aoi{
namespace utillity{


class SINS_CLASS SinsImp{
public:
	SinsImp() = delete ;
	static ISins* create();
	static void destory(ISins* p);
};



}	// namespace utillity 
}	// namespace aoi




#endif	// SINS_IMP_E7AC1909_E096_4016_BBB6_AB7F06AF7B41__INCLUDE