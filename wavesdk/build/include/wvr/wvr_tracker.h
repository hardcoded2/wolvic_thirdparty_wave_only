// "WaveVR SDK 
// © 2021 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."


#ifndef wvr_tracker_h_
#define wvr_tracker_h_

#include "wvr_types.h"
#include "wvr_stdinc.h"

#include "begin_code.h"
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief The tracker role
 * Describes the role of the tracker device
 * @version API Level 8
 */
typedef enum {
    WVR_TrackerRole_Undefined        = 0,
    WVR_TrackerRole_Standalone       = 1,
    WVR_TrackerRole_Pair1_Right      = 2,
    WVR_TrackerRole_Pair1_Left       = 3
} WVR_TrackerRole;

/**
 * @brief The capabilities of the tracker device.
 * @version API Level 8
 */
typedef struct WVR_TrackerCapabilities {
    bool supportsOrientationTracking;
    bool supportsPositionTracking;
    bool supportsInputDevice;
    bool supportsHapticVibration;
    bool supportsBatteryLevel;
    bool supportsExtendedData;
} WVR_TrackerCapabilities_t;

/**
 * @brief Function to start the feature of the tracker.
 *
 * This API will start the feature of the tracker.
 * This API must be called by main thread.
 *
 * @retval WVR_Success start the tracker feature successfully.
 * @retval others means failure. Refer to @ref WVR_Result
 * @version API Level 8
*/
extern WVR_EXPORT WVR_Result WVR_StartTracker();

/**
 * @brief Function to stop the feature of the tracker.
 *
 * This API will stop the feature of the tracker that already started, and release related sources.
 * This API must be called by main thread.
 * @version API Level 8
*/
extern WVR_EXPORT void WVR_StopTracker();

/**
 * @brief Function to know whether the tracker device is connected or not.
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @return True means connected; false means disconnected.
 * @version API Level 8
 */
extern WVR_EXPORT bool WVR_IsTrackerConnected(WVR_TrackerId trackerId);

/**
 * @brief Function to get the tracker role of the tracker device.
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @return The tracker role, refer to @ref WVR_TrackerRole.
 * @version API Level 8
*/
extern WVR_EXPORT WVR_TrackerRole WVR_GetTrackerRole(WVR_TrackerId trackerId);

/**
 * @brief Function to get the capability of the tracker device.
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param info The capabilities of the tracker. Refer to @ref WVR_TrackerCapabilities.
 * @retval WVR_Success Get the capabilities of the tracker successfully.
 * @retval Others @ref WVR_Result mean failure.
 * @version API Level 8
*/
extern WVR_EXPORT WVR_Result WVR_GetTrackerCapabilities(WVR_TrackerId trackerId, WVR_TrackerCapabilities_t* capabilities);

/**
 * @brief Function to get the pose of the tracker device with or without prediction
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param originModel Specify the tracking universe of the origin tracking model. Refer to @ref WVR_PoseOriginModel.
 * @param predictedMilliSec Number of milliseconds from now to predict poses for. Positive numbers are in the future.
 * Pass 0 to get the state at the instant the function is called.
 * @param poseState Obtain the pose data. Refer to @ref WVR_PoseState.
 * @retval WVR_Success Get the pose of the tracker successfully.
 * @retval Others @ref WVR_Result mean failure.
 * @version API Level 8
*/
extern WVR_EXPORT WVR_Result WVR_GetTrackerPoseState(WVR_TrackerId trackerId, WVR_PoseOriginModel originModel, uint32_t predictedMilliSec, WVR_PoseState_t *poseState);

/**
 * @brief Function to get the input device capability of the tracker device for WVR_InputType
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param inputType One of @ref WVR_InputType.
 * @return int32_t, bitmask for @ref WVR_InputId. A value of -1 means an error has occurred.
 * @version API Level 8
*/
extern WVR_EXPORT int32_t WVR_GetTrackerInputDeviceCapability(WVR_TrackerId trackerId, WVR_InputType inputType);

/**
 * @brief Function to get the analog type of the input device of the tracker device
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param inputId One of @ref WVR_InputId.
 * @return Analog type for @ref WVR_InputId. Refer to @ref WVR_AnalogType.
 * @version API Level 8
*/
extern WVR_EXPORT WVR_AnalogType WVR_GetTrackerInputDeviceAnalogType(WVR_TrackerId trackerId, WVR_InputId inputId);

/**
 * @brief Function to get the button state of a specific input id of the tracker device
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param inputId One of @ref WVR_InputId. Indicates which button is pressed or unpressed.
 * @return bool, true for pressed and false means unpressed.
 * @version API Level 8
*/
extern WVR_EXPORT bool WVR_GetTrackerInputButtonState(WVR_TrackerId trackerId, WVR_InputId inputId);

/**
 * @brief Function to get the touch state of a specific input id of the tracker device
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param inputId One of @ref WVR_InputId. Indicates which input id is touched or untouched.
 * @return bool, true for touched and false means untouched.
 * @version API Level 8
*/
extern WVR_EXPORT bool WVR_GetTrackerInputTouchState(WVR_TrackerId trackerId, WVR_InputId inputId);

/**
 * @brief Function to get the analog data of a specific input id of the tracker device
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param inputId One of @ref WVR_InputId. This parameter specifies the physical touch on the tracker that retrieves the input data.
 * @return Analog data for @ref WVR_InputId. Refer to @ref WVR_Axis.
 * @version API Level 8
*/
extern WVR_EXPORT WVR_Axis_t WVR_GetTrackerInputAnalogAxis(WVR_TrackerId trackerId, WVR_InputId inputId);

/**
 * @brief Function to get the current battery level of the tracker device.
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @return The current battery level of the tracker device. Valid interval: [0.0, 1.0].
 * Negative value means the tracker device does not support this feature or is not connected.
 * @version API Level 8
 */
extern WVR_EXPORT float WVR_GetTrackerBatteryLevel(WVR_TrackerId trackerId);

/**
 * @brief Function to trigger vibration with float amplitude of the tracker device
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param durationMicroSec The duration of the vibration in microsecond.
 * @param frequency The frequency of the vibration in Hz.
 * @param amplitude float intensity of the vibration. Valid interval: [0.0, 1.0].
 * @retval WVR_Success Trigger the vibration of the tracker device successfully.
 * @retval Others @ref WVR_Result mean failure.
 * @version API Level 8
*/
extern WVR_EXPORT WVR_Result WVR_TriggerTrackerVibration(WVR_TrackerId trackerId, uint32_t durationMicroSec = 0, float frequency = 0.0f, float amplitude = 0.0f);

/**
 * @brief Function to get extended data of tracker device.
 *
 * @param trackerId Indicates what id of the tracker device. (Refer to @ref WVR_TrackerId)
 * @param validSize This size determines the valid length of data content. If validSize is less than or equal to 0, the data content from the returned address are meaningless.
 * @retval int32_t* The first address of data content.
 * @remark This API works only when supportsExtendedData (obtained by WVR_GetTrackerCapabilities) is true .
 * @version API Level 8
*/
extern WVR_EXPORT int32_t* WVR_GetTrackerExtendedData(WVR_TrackerId trackerId, int32_t* validSize);


#ifdef __cplusplus
} /* extern "C" */
#endif
#include "close_code.h"

#endif /* wvr_tracker_h_ */
