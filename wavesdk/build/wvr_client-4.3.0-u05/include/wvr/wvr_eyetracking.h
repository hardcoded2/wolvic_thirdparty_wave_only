// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."


#ifndef wvr_eyetracking_h_
#define wvr_eyetracking_h_

#include "wvr_types.h"
#include "wvr_stdinc.h"

#include "begin_code.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure containing flags indicating data validity of an eye pose
 */
typedef enum {
    WVR_GazeOriginValid                 = 1<<0,
    WVR_GazeDirectionNormalizedValid    = 1<<1,
    WVR_PupilDiameterValid              = 1<<2,
    WVR_EyeOpennessValid                = 1<<3,
    WVR_PupilPositionInSensorAreaValid  = 1<<4
} WVR_EyeTrackingStatus;

/**
 * @brief the single eye tracking data
 *
 * the single eye tracking data
 */
typedef struct WVR_SingleEyeTracking {
    uint64_t       eyeTrackingValidBitMask;    /**< The bits containing all validity for this data. */
    WVR_Vector3f_t gazeOrigin;                 /**< The point in the eye from which the gaze ray originates in meters. */
    WVR_Vector3f_t gazeDirectionNormalized;    /**< The normalized gaze direction of the eye in [0,1]. */
    float          eyeOpenness;                /**< A value representing how open the eye is. */
    float          pupilDiameter;              /**< The diameter of pupil in millimeters. */
    WVR_Vector2f_t pupilPositionInSensorArea;  /**< The normalized position of a pupil in [0,1]. */
} WVR_SingleEyeTracking_t;

/**
 * @brief the combined eye tracking data
 *
 * the combined eye tracking data
 */
typedef struct WVR_CombinedEyeTracking {
    uint64_t       eyeTrackingValidBitMask;    /**< The bits containing all validity for this data. */
    WVR_Vector3f_t gazeOrigin;                 /**< The point of two eyes combined from which the gaze ray originates in meters. */
    WVR_Vector3f_t gazeDirectionNormalized;    /**< The normalized gaze direction of two eyes in [0,1]. */
} WVR_CombinedEyeTracking_t;

/**
 * @brief the eye tracking data
 *
 * the eye tracking data
 */
typedef struct WVR_EyeTracking {
    WVR_SingleEyeTracking   left;
    WVR_SingleEyeTracking   right;
    WVR_CombinedEyeTracking combined;
    int64_t                 timestamp;
} WVR_EyeTracking_t;

/**
 * @brief Function to start eye tracking.
 *
 * This API will start eye tracking module
 * This API must be called by main thread.
 *
 * @retval WVR_Success start eye tracking feature successfully.
 * @retval others @ref WVR_Result mean failure.
 * @version API Level 6
*/
extern WVR_EXPORT WVR_Result WVR_StartEyeTracking();

/**
 * @brief Function to stop eye tracking that you already started..
 *
 * This API will stop eye tracking that you already started,
 * and release related source.
 * This API must be called by main thread.
 * @version API Level 6
*/
extern WVR_EXPORT void WVR_StopEyeTracking();

/**
 * @brief Function to eye tracking data.
 *
 * This API is used to get eye tracking data from the eye tracker module
 * This API must be called by main thread.
 *
 * The tracking data from the eye tracker module itself is on **WVR_CoordinateSystem_Local** space.
 * If you access API by **WVR_CoordinateSystem_Global** parameter,
 * you will get tracking data which is transformed to HMD world coordinate space.
 *
 * @param data the eye tracking data @ref WVR_EyeTracking
 * @param space the coordinate system of tracking data @ref WVR_CoordinateSystem
 * @retval WVR_Success get data successfully.
 * @retval others @ref WVR_Result mean failure.
 * @version API Level 6
*/
extern WVR_EXPORT WVR_Result WVR_GetEyeTracking(WVR_EyeTracking_t *data, WVR_CoordinateSystem space=WVR_CoordinateSystem_Global);

#ifdef __cplusplus
} /* extern "C" */
#endif
#include "close_code.h"

#endif /* wvr_eyetracking_h_ */

