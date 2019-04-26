#ifndef EXPORT_H_EDE67FAB_B425_4D72_99CD_E1A41418EE02__INCLUDE
#define EXPORT_H_EDE67FAB_B425_4D72_99CD_E1A41418EE02__INCLUDE

#if ( !defined STATIC_LIBRARY ) && defined WIN32
#ifdef ARKF_EXPORT
    #define ARKF_CLASS _declspec(dllexport)
#else
	#define ARKF_CLASS _declspec(dllimport)
#endif

#ifdef ARKF_EXPORT
#define ARKF_FUNCTION _declspec(dllexport)
#else
#define ARKF_FUNCTION _declspec(dllimport)
#endif
#else
#define ARKF_CLASS
#define ARKF_FUNCTION 

#endif

#endif // EXPORT_H_EDE67FAB_B425_4D72_99CD_E1A41418EE02__INCLUDE
