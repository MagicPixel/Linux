#ifndef LUA_FILE_EDIT_H_
#define LUA_FILE_EDIT_H_

#include "exportdll.h"
#include "lua.hpp"

class LUA_CLASS LuaConfigFileIO
{
public:
	LuaConfigFileIO();
	~LuaConfigFileIO();

	//��ʼ��Lua����ָ�����
	void Init();

	//�ر�Lua����ָ��
	void Close();

	//����ָ����Lua�ļ�
	bool LoadLuaFile(const char* pFileName);

	//ִ��ָ��Lua�ļ��еĺ���
	bool CallFileFn(const char* pFunctionName, int nParam1, int nParam2);

	//////////////////////////////////////////////////////////////////////////////////////////////
	//��ȡһ��Ԫ��
	bool GetTableFromKey_int(const char* lpszTableName, int &nValue);
	bool GetTableFromKey_float(const char* lpszTableName, float &nValue);
	bool GetTableFromKey_double(const char* lpszTableName, double &nValue);
	bool GetTableFromKey_String(const char* lpszTableName, char *char_Value);

	//��ȡ��ȡ�ṹ���д����ֵ�Ԫ��
	bool GetTableFromItem_int(const char* lpszTableName, const char* lpszTableItem, int &nValue);
	bool GetTableFromItem_float(const char* lpszTableName, const char* lpszTableItem, float &nValue);
	bool GetTableFromItem_double(const char* lpszTableName, const char* lpszTableItem, double &nValue);
	bool GetTableFromItem_String(const char* lpszTableName, const char* lpszTableItem, char *char_Value);

	//��ȡ�����е�Ԫ��
	bool GetTableFromIndex_int(const char* lpszTableName, int index, int &nValue);
	bool GetTableFromIndex_float(const char* lpszTableName, int index, float &nValue);
	bool GetTableFromIndex_double(const char* lpszTableName, int index, double &nValue);
	bool GetTableFromIndex_String(const char* lpszTableName, int index, char *char_Value);

	bool Get_Index_Elems_int(const char* lpszTableName, int EleNum, int *ElemsData);
	bool Get_Index_Elems_float(const char* lpszTableName, int EleNum, float *ElemsData);
	bool Get_Index_Elems_double(const char* lpszTableName, int EleNum, double *ElemsData);


	//����/�ṹ����Ԫ�صĸ���
	int Get_Ele_num(const char* lpszTableName);


	//////////////////////////////////////////////////////////////////////////////////////////////
	//���ɴ����ֵ�Ԫ�صĽṹ��
	bool SetTableFromItem_int(const char* lpszTableName, const char* lpszTableItem, int nValue);
	bool SetTableFromItem_float(const char* lpszTableName, const char* lpszTableItem, float nValue);
	bool SetTableFromItem_double(const char* lpszTableName, const char* lpszTableItem, double nValue);
	bool SetTableFromItem_String(const char* lpszTableName, const char* lpszTableItem, char *char_Value);

	//��������
	bool SetTableFromIndex_int(const char* lpszTableName, int index, int nValue);
	bool SetTableFromIndex_float(const char* lpszTableName, int index, float nValue);
	bool SetTableFromIndex_double(const char* lpszTableName, int index, double nValue);
	bool SetTableFromIndex_String(const char* lpszTableName, int index, char *char_Value);


private:
	lua_State* m_pState;   //Lua��State����ָ�룬һ��lua�ļ���Ӧһ��
};
#endif  // LUA_FILE_EDIT_H_
