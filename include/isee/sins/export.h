#ifndef EXPORT_H_53411322_9746_40E3_BBE4_F2F10AF70BAE__INCLUDE
#define EXPORT_H_53411322_9746_40E3_BBE4_F2F10AF70BAE__INCLUDE

#if ( !defined STATIC_LIBRARY ) && defined WIN32
#ifdef SINS_EXPORT
    #define SINS_CLASS _declspec(dllexport)
#else
    #define SINS_CLASS _declspec(dllimport)
#endif

#ifdef SINS_EXPORT
#define SINS_FUNCTION _declspec(dllexport)
#else
#define SINS_FUNCTION _declspec(dllimport)
#endif

#else
#define SINS_CLASS
#define SINS_FUNCTION 

#endif

#endif // EXPORT_H_53411322_9746_40E3_BBE4_F2F10AF70BAE__INCLUDE
