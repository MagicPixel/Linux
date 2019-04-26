#ifndef EXPORT_H_E345D868_B444_4963_8CFB_272E8362EE92__INCLUDE
#define EXPORT_H_E345D868_B444_4963_8CFB_272E8362EE92__INCLUDE

#if ( !defined STATIC_LIBRARY ) && defined WIN32
#ifdef LUA_EXPORT
    #define LUA_CLASS _declspec(dllexport)
#else
    #define LUA_CLASS _declspec(dllimport)
#endif

#else
#define LUA_CLASS

#endif

//#ifdef _MSC_VER
//#define _WIN32_WINNT 0x0701
//#define D_WIN32_WINNT 0x0701
//#endif

#endif // EXPORT_H_E345D868_B444_4963_8CFB_272E8362EE92__INCLUDE
