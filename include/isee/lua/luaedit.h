#ifndef LUA_FILE_EDIT_H_
#define LUA_FILE_EDIT_H_

#include "exportdll.h"
#include "lua.hpp"

class LUA_CLASS LuaConfigFileIO
{
public:
	LuaConfigFileIO();
	~LuaConfigFileIO();

	//初始化Lua对象指针参数
	void Init();

	//关闭Lua对象指针
	void Close();

	//加载指定的Lua文件
	bool LoadLuaFile(const char* pFileName);

	//执行指定Lua文件中的函数
	bool CallFileFn(const char* pFunctionName, int nParam1, int nParam2);

	//////////////////////////////////////////////////////////////////////////////////////////////
	//提取一阶元素
	bool GetTableFromKey_int(const char* lpszTableName, int &nValue);
	bool GetTableFromKey_float(const char* lpszTableName, float &nValue);
	bool GetTableFromKey_double(const char* lpszTableName, double &nValue);
	bool GetTableFromKey_String(const char* lpszTableName, char *char_Value);

	//读取提取结构体中带名字的元素
	bool GetTableFromItem_int(const char* lpszTableName, const char* lpszTableItem, int &nValue);
	bool GetTableFromItem_float(const char* lpszTableName, const char* lpszTableItem, float &nValue);
	bool GetTableFromItem_double(const char* lpszTableName, const char* lpszTableItem, double &nValue);
	bool GetTableFromItem_String(const char* lpszTableName, const char* lpszTableItem, char *char_Value);

	//读取数组中的元素
	bool GetTableFromIndex_int(const char* lpszTableName, int index, int &nValue);
	bool GetTableFromIndex_float(const char* lpszTableName, int index, float &nValue);
	bool GetTableFromIndex_double(const char* lpszTableName, int index, double &nValue);
	bool GetTableFromIndex_String(const char* lpszTableName, int index, char *char_Value);

	bool Get_Index_Elems_int(const char* lpszTableName, int EleNum, int *ElemsData);
	bool Get_Index_Elems_float(const char* lpszTableName, int EleNum, float *ElemsData);
	bool Get_Index_Elems_double(const char* lpszTableName, int EleNum, double *ElemsData);


	//数组/结构体中元素的个数
	int Get_Ele_num(const char* lpszTableName);


	//////////////////////////////////////////////////////////////////////////////////////////////
	//生成带名字的元素的结构体
	bool SetTableFromItem_int(const char* lpszTableName, const char* lpszTableItem, int nValue);
	bool SetTableFromItem_float(const char* lpszTableName, const char* lpszTableItem, float nValue);
	bool SetTableFromItem_double(const char* lpszTableName, const char* lpszTableItem, double nValue);
	bool SetTableFromItem_String(const char* lpszTableName, const char* lpszTableItem, char *char_Value);

	//生成数组
	bool SetTableFromIndex_int(const char* lpszTableName, int index, int nValue);
	bool SetTableFromIndex_float(const char* lpszTableName, int index, float nValue);
	bool SetTableFromIndex_double(const char* lpszTableName, int index, double nValue);
	bool SetTableFromIndex_String(const char* lpszTableName, int index, char *char_Value);


private:
	lua_State* m_pState;   //Lua的State对象指针，一个lua文件对应一个
};
#endif  // LUA_FILE_EDIT_H_
