// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."


#ifdef _begin_code_h
#error "Nested inclusion of begin_code.h"
#endif
#define _begin_code_h

#ifndef WVR_EXPORT
# if defined(__GNUC__) && __GNUC__ >=4
#  define WVR_EXPORT __attribute__ ((visibility("default")))
#  define WVR_DEPRECATED(message) __attribute__((deprecated(message)))
#  define WVR_EXP_DEPRECATED(message) __attribute__((visibility("default"), deprecated(message)))
# else
#  define WVR_EXPORT
#  define WVR_DEPRECATED(message)
#  define WVR_EXP_DEPRECATED(message)
# endif
#endif
