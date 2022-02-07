// "WaveVR SDK
// © 2017 HTC Corporation. All Rights Reserved.
//
// Unless otherwise required by copyright law and practice,
// upon the execution of HTC SDK license agreement,
// HTC grants you access to and use of the WaveVR SDK(s).
// You shall fully comply with all of HTC’s SDK license agreement terms and
// conditions signed by you and all SDK and API requirements,
// specifications, and documentation provided by HTC to You."


#ifndef wvr_hand_render_model_h_
#define wvr_hand_render_model_h_

#include "wvr_ctrller_render_model.h"

#include "begin_code.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Vertex buffer of component in hand model.
 */
typedef struct WVR_BoneIDBuffer {
    uint32_t *buffer;   /**< Vertex buffer. */
    uint32_t size;      /**< Vertex buffer size. */
    uint32_t dimension; /**< boneID */
} WVR_BoneIDBuffer_t;

/**
 * @brief Structure about one natural hand.
 */
typedef struct WVR_HandModel {
    WVR_VertexBuffer_t vertices; /**< Positions.(Size should be same with normal and tex coord buffers) */
    WVR_VertexBuffer_t normals; /**< Normals. (Size should be same with vertices and tex coord buffers) */
    WVR_VertexBuffer_t texCoords; /**< Texture coordinates for alpha contouring. (Size should be same with vertex and normal buffers) */
    WVR_VertexBuffer_t texCoord2s; /**< Texture coordinates 2 for alpha texture. (Size should be same with vertex and normal buffers) */
    WVR_BoneIDBuffer_t boneIDs; /**< boneID for each effect bone. (Size should be same with vertex and normal buffers) */
    WVR_VertexBuffer_t boneWeights; /**< bone weight for each effect bone. (Size should be same with vertex and normal buffers) */
    WVR_IndexBuffer_t indices; /**< Face indices. */
    float jointInvTransMats[768]; /**< The matrix for covert object space to bone space. */
    float jointTransMats[768]; /**< The matrix for covert bone space to object space. */
    float jointLocalTransMats[768]; /**< The matrix for covert bone space to object space. */
    uint32_t jointParentTable[48]; /**< The parent bone ID for target bone. If the ID is equal to 47. It means null.*/
    int32_t jointUsageTable[48]; /**< The bone is valid or not. */
} WVR_HandModel_t;

/**
 * @brief structure for render model for two natural hand.
 */
typedef struct WVR_HandRenderModel {
    WVR_HandModel_t left; /**< The left hand model.*/
    WVR_HandModel_t right; /**< The right hand model.*/
    WVR_CtrlerTexBitmap_t handAlphaTex; /**< The hand texture for alpha blending.*/
} WVR_HandRenderModel_t;

/**
 * @brief Use this function to get the natural hand devices. 
 *
 * Note: It is not recommended to call this function after we get controller model.
 *
 * @param handModel (Output) A double pointer for receiving the natural hand model.
 *                           The SDK will allocate the data structure for the natural hand model and write the address into handModel.
 * @retval WVR_SUCCESS The natural hand model was successfully retrieved from the currently connected device.
 * @retval WVR_Error_InvalidArgument If handModel is nullptr or pointer in *handModel isn’t nullptr.
 * @retval WVR_Error_SystemInvalid The SDK can’t get the hand model since the system has not yet been initialized.
 * @retval WVR_Error_InvalidRenderModel The SDK can’t get the hand model since the connect between hand component and app has not yet been initialized.
 * @version API Level 7
 */
extern WVR_EXPORT WVR_Result WVR_GetCurrentNaturalHandModel(WVR_HandRenderModel_t **handModel);

/**
 * @brief Use this function to deallocate the memory used for the natural hand devices.
 *
 * Delete the natural hand model received from WVR_GetCurrentNaturalHandModel.
 *
 * @param handModel An allocated object about natural hand model. The object will be destroyed and the structure cleared(handModel will be set nullptr).
 * @version API Level 7
 */
extern WVR_EXPORT void WVR_ReleaseNatureHandModel(WVR_HandRenderModel_t **handModel);


#ifdef __cplusplus
} /* extern "C" */
#endif
#include "close_code.h"

#endif