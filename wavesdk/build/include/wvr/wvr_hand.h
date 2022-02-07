// "WaveVR SDK 
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."

#ifndef wvr_hand_h_
#define wvr_hand_h_

#include "wvr_stdinc.h"
#include "wvr_types.h"

#include "begin_code.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The gesture type
 */
typedef enum {
    WVR_HandGestureType_Invalid     = 0,    /**< The gesture is invalid. */
    WVR_HandGestureType_Unknown     = 1,    /**< Unknow gesture type. */
    WVR_HandGestureType_Fist        = 2,    /**< Represent fist gesture. */
    WVR_HandGestureType_Five        = 3,    /**< Represent five gesture. */
    WVR_HandGestureType_OK          = 4,    /**< Represent OK gesture. */
    WVR_HandGestureType_ThumbUp     = 5,    /**< Represent thumb up gesture. */
    WVR_HandGestureType_IndexUp     = 6,    /**< Represent index up gesture. */
    WVR_HandGestureType_Palm_Pinch  = 7,    /**< Represent palm pinch gesture. */
    WVR_HandGestureType_Yeah        = 8,    /**< Represent yeah gesture. */
    WVR_HandGestureType_Reserved1   = 32,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved2   = 33,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved3   = 34,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved4   = 35,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved5   = 36,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved6   = 37,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved7   = 38,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved8   = 39,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved9   = 40,   /**< Reserved gesture. */
    WVR_HandGestureType_Reserved10  = 41,   /**< Reserved gesture. */
} WVR_HandGestureType;

/**
 * @brief The type of hand tracker device.
 * @version API Level 7
 */
typedef enum {
    WVR_HandTrackerType_Natural     = 1,
    WVR_HandTrackerType_Electronic  = 2,
} WVR_HandTrackerType;

/**
 * @brief The type of the hand model.
 * @version API Level 7
 */
typedef enum {
    WVR_HandModelType_WithoutController = 1<<0,
    WVR_HandModelType_WithController    = 1<<1
} WVR_HandModelType;

/**
 * @brief The flags that indicate data validity of one hand joint.
 * @version API Level 7
 */
typedef enum {
    WVR_HandJointValidFlag_PositionValid = 1<<0,
    WVR_HandJointValidFlag_RotationValid = 1<<1
} WVR_HandJointValidFlag;

/**
 * @brief The conventions of hand joints
 * @version API Level 7
 */
typedef enum {
    WVR_HandJoint_Palm          = 0,
    WVR_HandJoint_Wrist         = 1,
    WVR_HandJoint_Thumb_Joint0  = 2,
    WVR_HandJoint_Thumb_Joint1  = 3,
    WVR_HandJoint_Thumb_Joint2  = 4,
    WVR_HandJoint_Thumb_Tip     = 5,
    WVR_HandJoint_Index_Joint0  = 6,
    WVR_HandJoint_Index_Joint1  = 7,
    WVR_HandJoint_Index_Joint2  = 8,
    WVR_HandJoint_Index_Joint3  = 9,
    WVR_HandJoint_Index_Tip     = 10,
    WVR_HandJoint_Middle_Joint0 = 11,
    WVR_HandJoint_Middle_Joint1 = 12,
    WVR_HandJoint_Middle_Joint2 = 13,
    WVR_HandJoint_Middle_Joint3 = 14,
    WVR_HandJoint_Middle_Tip    = 15,
    WVR_HandJoint_Ring_Joint0   = 16,
    WVR_HandJoint_Ring_Joint1   = 17,
    WVR_HandJoint_Ring_Joint2   = 18,
    WVR_HandJoint_Ring_Joint3   = 19,
    WVR_HandJoint_Ring_Tip      = 20,
    WVR_HandJoint_Pinky_Joint0  = 21,
    WVR_HandJoint_Pinky_Joint1  = 22,
    WVR_HandJoint_Pinky_Joint2  = 23,
    WVR_HandJoint_Pinky_Joint3  = 24,
    WVR_HandJoint_Pinky_Tip     = 25,
} WVR_HandJoint;

/**
 * @brief The object type of hand hold
 */
typedef enum {
    WVR_HandHoldObjectType_None        = 0,  /**< The object type of hand hold is none. */
    WVR_HandHoldObjectType_Gun         = 1,  /**< The object type of hand hold is a gun. */
    WVR_HandHoldObjectType_OCSpray     = 2,  /**< The object type of hand hold is a OC spray. */
    WVR_HandHoldObjectType_LongGun     = 3,  /**< The object type of hand hold is a long gun. */
    WVR_HandHoldObjectType_Baton       = 4,  /**< The object type of hand hold is a baton. */
    WVR_HandHoldObjectType_FlashLight  = 5   /**< The object type of hand hold is a flashlight. */
} WVR_HandHoldObjectType;

/**
 * @brief The role type of hand hold
 */
typedef enum {
    WVR_HandHoldRoleType_None     = 0,  /**< The role type of hand hold is none. */
    WVR_HandHoldRoleType_MainHold = 1,  /**< The role type of hand hold is main hold. */
    WVR_HandHoldRoleType_SideHold = 2   /**< The role type of hand hold is side hold. */
} WVR_HandHoldRoleType;

/**
 * @brief The gesture information
 */
typedef struct WVR_HandGestureInfo {
    uint64_t supportedMask;
} WVR_HandGestureInfo_t;

/**
 * @brief The gesture information
 */
typedef struct WVR_HandGestureData {
    int64_t             timestamp;  /**< The current time in milliseconds. */
    WVR_HandGestureType right;      /**< Gesture type of right hand. */
    WVR_HandGestureType left;       /**< Gesture type of left hand. */
}  WVR_HandGestureData_t;

/**
 * @brief The finger name
 */
typedef enum {
    WVR_FingerType_Thumb    = 1,    /**< Represent thumb finger. */
    WVR_FingerType_Index    = 2,    /**< Represent index finger. */
    WVR_FingerType_Middle   = 3,    /**< Represent middle finger. */
    WVR_FingerType_Ring     = 4,    /**< Represent ring finger. */
    WVR_FingerType_Pinky    = 5     /**< Represent pinky finger. */
} WVR_FingerType;

/**
 * @brief The hand pose type definition
 */
typedef enum {
    WVR_HandPoseType_Invalid  = 0,  /**< The hand pose type is invalid. */
    WVR_HandPoseType_Pinch    = 1,  /**< The hand pose type is pinch. */
    WVR_HandPoseType_Hold     = 2,  /**< The hand pose type is hold. */
} WVR_HandPoseType;

/**
 * @brief The common pose state information
 */
typedef struct WVR_HandPoseStateBase {
    WVR_HandPoseType  type;         /**< The current hand pose type. */
} WVR_HandPoseStateBase_t;

/**
 * @brief The pinch state
 */
typedef struct WVR_HandPosePinchState {
    WVR_HandPoseStateBase_t base;       /**< Refer to @ref WVR_HandPoseStateBase */
    WVR_FingerType          finger;     /**< Move someone finger and thumb apart or bring them together.*/
    float                   strength;   /**< The value of ranges between 0 to 1 for each finger pich, 1 means pinch with the finger touching the thumb fully. */
    WVR_Vector3f_t          origin;     /**< The pinch origin. */
    WVR_Vector3f_t          direction;  /**< The pinch direction. */
} WVR_HandPosePinchState_t;

/**
 * @brief The hand hold state.
 */
typedef struct WVR_HandPoseHoldState {
    WVR_HandPoseStateBase_t base;    /**< Refer to @ref WVR_HandPoseStateBase. */
    WVR_HandHoldRoleType    role;    /**< The role type of hand hold, refer to @ref WVR_HandHoldRoleType. */
    WVR_HandHoldObjectType  object;  /**< The object type of hand hold, refer to @ref WVR_HandHoldObjectType. */
} WVR_HandPoseHoldState_t;

/**
 * @brief The hand pose state
 */
typedef union WVR_HandPoseState {
    WVR_HandPoseStateBase_t     base;   /**< Refer to @ref WVR_HandPoseStateBase */
    WVR_HandPosePinchState_t    pinch;  /**< Refer to @ref WVR_HandPosePinchState */
    WVR_HandPoseHoldState_t     hold;   /**< Refer to @ref WVR_HandPoseHoldState */
} WVR_HandPoseState_t;

/**
 * @brief The hands pose information
 */
typedef struct WVR_HandPoseData {
    int64_t                 timestamp;  /**< The current time in milliseconds. */
    WVR_HandPoseState_t     right;      /**< The pose state of right hand, refer to @ref WVR_HandPoseState. */
    WVR_HandPoseState_t     left;       /**< The pose state of left hand, refer to @ref WVR_HandPoseState. */
} WVR_HandPoseData_t;

/**
 * @brief The information of hand tracker device.
 * @version API Level 7
 */
typedef struct WVR_HandTrackerInfo {
    uint32_t         jointCount;              /**< The count of hand joints. */
    uint64_t         handModelTypeBitMask;    /**< Support how many types of @ref WVR_HandModelType. */
    WVR_HandJoint    *jointMappingArray;      /**< The array corresponds to the conventions of hand joints. Refer to @ref WVR_HandJoint. */
    uint64_t         *jointValidFlagArray;    /**< The array that indicates which data of each hand joint is valid, refer to @ref WVR_HandJointValidFlag. */
    float            pinchTHR;                /**< The value of ranges between 0 to 1. */
} WVR_HandTrackerInfo_t;

/**
 * @brief The data structure of one hand.
 * @version API Level 7
 */
typedef struct WVR_HandJointData {
    bool        isValidPose;    /**< The label of valid(true)/invalid(false) pose. */
    float       confidence;     /**< The hand confidence value. */
    uint32_t    jointCount;     /**< Specify the size of the @ref WVR_Pose_t array. */
    WVR_Pose_t  *joints;        /**< The array of the @ref WVR_Pose_t. */
    WVR_Vector3f_t scale;       /**< The hand scale value to @ref WVR_Vector3f_t. The default is 1. */
} WVR_HandJointData_t;

/**
 * @brief The data structure of the hand tracker data that contains both hands.
 * @version API Level 7
 */
typedef struct WVR_HandTrackingData {
    int64_t             timestamp;
    WVR_HandJointData_t right;    /**< The hand tracker data of right hand, refer to @ref WVR_HandJointData_t. */
    WVR_HandJointData_t left;     /**< The hand tracker data of left hand, refer to @ref WVR_HandJointData_t. */
} WVR_HandTrackingData_t;

/**
 * @brief Use this function to get hand gesture configuration.
 *
 * Use this API to get hand gesture configuration.
 * This API must be called by main thread.
 * Note: Right hand WVR_HandGestureType_Palm_Pinch is for System using. 
 *       Developer can not get right hand WVR_HandGestureType_Palm_Pinch.
 *       Developer can get left hand WVR_HandGestureType_Palm_Pinch.
 *
 * @param info of @ref WVR_HandGestureType.
 * @retval WVR_Success Get information successfully.
 * @retval Others @ref WVR_Result mean failure.
 * @version API Level 7
*/
extern WVR_EXPORT WVR_Result WVR_GetHandGestureInfo(WVR_HandGestureInfo_t* info);

/**
 * @brief Use this function to start hand gesture feature.
 *
 * Use this API to start hand gesture module
 * This API must be called by main thread.
 *
 * @param demands Bitmask of @ref WVR_HandGestureType.
 * @retval WVR_Success start hand gesture feature successfully.
 * @retval others @ref WVR_Result mean failure.
 * @version API Level 7
*/
extern WVR_EXPORT WVR_Result WVR_StartHandGesture(uint64_t demands = 0x3);

/**
 * @brief Use this function to stop hand gesture that you already started.
 *
 * Use this API to stop hand gesture that you already started,
 * and release related hand gesture source.
 * This API must be called by main thread.
 * @version API Level 7
*/
extern WVR_EXPORT void WVR_StopHandGesture();

/**
 * @brief Use this function to get gesture data.
 *
 * Use this API to get hand gesture state from the hand gesture module.
 * Use this API must be called by main thread.
 *
 * @param data The hand gesture data refer to @ref WVR_HandGestureData.
 * @retval WVR_Success Successfully retrieved data.
 * @retval others @ref WVR_Result the mean failure.
 * @version API Level 7
*/
extern WVR_EXPORT WVR_Result WVR_GetHandGestureData(WVR_HandGestureData_t *data);

/**
 * @brief Use this function to start hand tracking feature.
 *
 * Use this API to start hand tracking module.
 * This API must be called by main thread.
 * @param type The type of hand tracker. Refer to @ref WVR_HandTrackerType.
 * If type is WVR_HandTrackerType_Natural, please check WVR_DeviceType_NaturalHand_Right or WVR_DeviceType_NaturalHand_Left is connected before starting.
 * Otherwise, please check WVR_DeviceType_ElectronicHand_Right or WVR_DeviceType_ElectronicHand_Left is connected before starting.
 * @retval WVR_Success start hand tracking feature successfully.
 * @retval others @ref WVR_Result mean failure.
 * @version API Level 7
*/
extern WVR_EXPORT WVR_Result WVR_StartHandTracking(WVR_HandTrackerType type);

/**
 * @brief Use this function to stop the hand tracking that was started.
 *
 * Use this API to stop hand tracking that you already started,
 * and release related hand tracking resources.
 * This API must be called by the main thread.
 * @param type The type of hand tracker. Refer to @ref WVR_HandTrackerType.
 * @version API Level 7
*/
extern WVR_EXPORT void WVR_StopHandTracking(WVR_HandTrackerType type);

/**
 * @brief Use this function to get the hand tracking device.
 *
 * @param trackerType Specify the type of hand tracker. Refer to @ref WVR_HandTrackerType.
 * @param modelType Specify the type of hand model. Refer to @ref WVR_HandModelType.
 * @param originModel Specify the tracking universe of the origin tracking model. Refer to @ref WVR_PoseOriginModel.
 * @param skeleton The hand skeleton data @ref WVR_HandTrackingData_t
 * @param pose The hand pose state refer to @ref WVR_HandPoseData_t
 * @retval WVR_Success Successfully retrieved data.
 * @retval Others @ref WVR_Result mean failure.
 * @version API Level 7
*/
extern WVR_EXPORT WVR_Result WVR_GetHandTrackingData(WVR_HandTrackerType trackerType,
                             WVR_HandModelType modelType,
                             WVR_PoseOriginModel originModel,
                             WVR_HandTrackingData* skeleton,
                             WVR_HandPoseData_t* pose=nullptr);

/**
 * @brief Use this function to get the number of hand joints.
 *
 * @param type The type of hand tracker. Refer to @ref WVR_HandTrackerType.
 * @param jointCount Specify the number of the hand joints that the hand tracker provides.
 * @retval WVR_Success Get information successfully.
 * @retval Others @ref WVR_Result mean failure.
 * @version API Level 7
*/
extern WVR_EXPORT WVR_Result WVR_GetHandJointCount(WVR_HandTrackerType type, uint32_t* jointCount);

/**
 * @brief Use this function to get the capability of hand tracker device.
 * @param type The type of hand tracker. Refer to @ref WVR_HandTrackerType.
 * @param info The information of hand tracker device. Refer to @ref WVR_HandTrackerInfo.
 * @retval WVR_Success Get information successfully.
 * @retval Others @ref WVR_Result mean failure.
 * @version API Level 7
*/
extern WVR_EXPORT WVR_Result WVR_GetHandTrackerInfo(WVR_HandTrackerType type, WVR_HandTrackerInfo_t* info);

/**
 * @brief Use this function to enhance Hand stability with WAVE Wrist Tracker(s).
 * @param wear True if user is wearing WAVE Wrist Trackers
 * @version API Level 8
*/
extern WVR_EXPORT void WVR_EnhanceHandStable(bool wear = false);

/**
 * @brief Function to examine Hand stability is guaranteed in engine level.
 * @version API Level 8
*/
extern WVR_EXPORT bool WVR_IsEnhanceHandStable();

/**
 * @brief Function to examine that does the current controller support electronic hand feature.
 * @version API Level 7
 */
extern WVR_EXPORT bool WVR_ControllerSupportElectronicHand();

#ifdef __cplusplus
} /* extern "C" */
#endif
#include "close_code.h"

#endif /* wvr_hand_h_ */

