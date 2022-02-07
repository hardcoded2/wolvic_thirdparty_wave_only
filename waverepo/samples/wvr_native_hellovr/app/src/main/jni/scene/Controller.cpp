#define LOG_TAG "APCtrler"

#define E_TO_UINT(enum) static_cast<uint32_t>(enum)

#include <functional>
#include <sstream>

//#include <sys/prctl.h> //for print thread name.

#include <pthread.h>

#include <log.h>

#include "../Context.h"
#include "../shared/quat.h"
#include "Controller.h"

void dumpMatrix(const char * name, const Matrix4& mat) {
    const float * ptr = mat.get();
    LOGD("%s =\n"
        " ⎡%+6f  %+6f  %+6f  %+6f⎤\n"
        " ⎢%+6f  %+6f  %+6f  %+6f⎥\n"
        " ⎢%+6f  %+6f  %+6f  %+6f⎥\n"
        " ⎣%+6f  %+6f  %+6f  %+6f⎦\n",
        name,
        ptr[0], ptr[4], ptr[8],  ptr[12],
        ptr[1], ptr[5], ptr[9],  ptr[13],
        ptr[2], ptr[6], ptr[10], ptr[14],
        ptr[3], ptr[7], ptr[11], ptr[15]);
}

AnimationPose::AnimationPose(const WVR_CtrlerModelAnimPoseData_t &iPose)
{
    mPosition.x = iPose.position.v[0];
    mPosition.y = iPose.position.v[1];
    mPosition.z = iPose.position.v[2];
    mRotation.x = iPose.rotation.v[0];
    mRotation.y = iPose.rotation.v[1];
    mRotation.z = iPose.rotation.v[2];
    mScale.x = iPose.scale.v[0];
    mScale.y = iPose.scale.v[1];
    mScale.z = iPose.scale.v[2];
}

AnimationPose& AnimationPose::operator=(const WVR_CtrlerModelAnimPoseData_t &iPose)
{
    mPosition.x = iPose.position.v[0];
    mPosition.y = iPose.position.v[1];
    mPosition.z = iPose.position.v[2];
    mRotation.x = iPose.rotation.v[0];
    mRotation.y = iPose.rotation.v[1];
    mRotation.z = iPose.rotation.v[2];
    mScale.x = iPose.scale.v[0];
    mScale.y = iPose.scale.v[1];
    mScale.z = iPose.scale.v[2];

    return *this;
}

Matrix4 AnimationPose::toMat4() const
{
    q_vec_type rotQuat;
    rotQuat[Q_X] = (mRotation.x / 180.0f) * Q_PI;
    rotQuat[Q_Y] = (mRotation.y / 180.0f) * Q_PI;
    rotQuat[Q_Z] = (mRotation.z / 180.0f) * Q_PI;
    Matrix4 result = Matrix4(rotQuat);
    result[0] = mScale.x; result[5] = mScale.y; result[10] = mScale.z;
    result[12] = mPosition.x;
    result[13] = mPosition.y;
    result[14] = mPosition.z;
    return result;
}

std::string AnimationPose::toString() const
{
    std::stringstream out;
    out << "P(" << mPosition.x << "," << mPosition.y << "," << mPosition.z << 
           "),R(" << mRotation.x << "," << mRotation.y << "," << mRotation.z << 
           "),S(" << mScale.x << "," << mScale.y << "," << mScale.z << ")";
    return out.str();
}

AnimationPose AnimationPose::lerp(const AnimationPose &iDst, float iRatio) const
{
    AnimationPose result;
    result.mPosition = Vector3::Lerp(mPosition, iDst.mPosition, iRatio);
    result.mRotation = Vector3::Lerp(mRotation, iDst.mRotation, iRatio);
    result.mScale = Vector3::Lerp(mScale, iDst.mScale, iRatio);
    return result;
}


void AnimationNode::setAnimation(const WVR_CtrlerModelAnimNodeData_t &iNodeData)
{
    mType = iNodeData.type;
    mUseBlueEffect = iNodeData.blueEffect;
    mPoses[AnimationPoses_Origin]  = iNodeData.origin;
    mPoses[AnimationPoses_Pressed] = iNodeData.pressed;
    mPoses[AnimationPoses_MaxX]    = iNodeData.maxX;
    mPoses[AnimationPoses_MinX]    = iNodeData.minX;
    mPoses[AnimationPoses_MaxY]    = iNodeData.maxY;
    mPoses[AnimationPoses_MinY]    = iNodeData.minY;
}

AnimationPose AnimationNode::getMinMaxAnimationPose(float iAnalogX, float iAnalogY) const
{
    AnimationPose result;
    result.mPosition = mPoses[AnimationPoses_Origin].mPosition +
                    Vector3::Lerp(mPoses[AnimationPoses_MinX].mPosition, mPoses[AnimationPoses_MaxX].mPosition, (iAnalogX + 1.0f) / 2.0f) - mPoses[AnimationPoses_Origin].mPosition + 
                    Vector3::Lerp(mPoses[AnimationPoses_MinY].mPosition, mPoses[AnimationPoses_MaxY].mPosition, (iAnalogY + 1.0f) / 2.0f) - mPoses[AnimationPoses_Origin].mPosition;

    result.mRotation = Vector3::Lerp(mPoses[AnimationPoses_MinX].mRotation, mPoses[AnimationPoses_MaxX].mRotation, (iAnalogX + 1.0f) / 2.0f) +
                       Vector3::Lerp(mPoses[AnimationPoses_MinY].mRotation, mPoses[AnimationPoses_MaxY].mRotation, (iAnalogY + 1.0f) / 2.0f);
    
    result.mScale = Vector3(1.0f, 1.0f, 1.0f);
    return result;
}

Matrix4 AnimationNode::getAnimationMatrix(float iAnalogX, float iAnalogY, bool iIsBtnPressed) const
{
    Matrix4 result;
    if (mType == 1 || mType == 2) {
        float finalRatio = iAnalogX;
        if (mType == 1 && iIsBtnPressed == true) {
            finalRatio = 1.0f;
        }
        AnimationPose finalPose = mPoses[AnimationPoses_Origin].lerp(mPoses[AnimationPoses_Pressed], finalRatio);
        result = finalPose.toMat4();
    } else if (mType == 3) {
        result = getMinMaxAnimationPose(iAnalogX, iAnalogY).toMat4();
    } else {
        result = mPoses[AnimationPoses_Origin].toMat4();
    }
    return result;
}



const std::string Controller::sControllerCompNames[CtrlerComp_MaxCompNumber] = 
{
   std::string("__CM__Body"),
   std::string("__CM__TouchPad"),
   std::string("__CM__AppButton"),
   std::string("__CM__HomeButton"),
   std::string("__CM__DigitalTriggerKey"),
   std::string("__CM__TriggerKey"),
   std::string("__CM__VolumeKey"),
   std::string("__CM__VolumeUp"),
   std::string("__CM__VolumeDown"),
   std::string("__CM__Grip"),
   std::string("__CM__DPad_Left"),
   std::string("__CM__DPad_Right"),
   std::string("__CM__DPad_Up"),
   std::string("__CM__DPad_Down"),
   std::string("__CM__TouchPad_Touch"),
   std::string("__CM__BeamOrigin"),
   std::string("__CM__Emitter"),
   std::string("__CM__Battery"),
   std::string("__CM__BumperKey"),
   std::string("__CM__Thumbstick"),
   std::string("__CM__ButtonA"),
   std::string("__CM__ButtonB"),
   std::string("__CM__ButtonX"),
   std::string("__CM__ButtonY")
};

const WVR_InputId Controller::sControllerCompIDTable[CtrlerComp_MaxCompNumber] = {
    WVR_InputId_Max,
    WVR_InputId_Alias1_Touchpad,
    WVR_InputId_Alias1_Menu,
    WVR_InputId_Alias1_System,
    WVR_InputId_Max,
    WVR_InputId_Alias1_Trigger,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Alias1_Grip,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Max,
    WVR_InputId_Alias1_Bumper,
    WVR_InputId_Alias1_Thumbstick,
    WVR_InputId_Alias1_A,
    WVR_InputId_Alias1_B,
    WVR_InputId_Alias1_X,
    WVR_InputId_Alias1_Y
};

Controller::Controller(WVR_DeviceType iCtrlerType)
: mCachedData(nullptr)
, mIsDataReady(false)
, mInitialized(false)
, mCtrlerType(iCtrlerType)
, mCompExistFlags{false}
, mCompDefaultDraw{false}
, mDiffTexLocations{-1, -1}
, mMatrixLocations{-1, -1}
, mUseEffectLocations{-1, -1}
, mEffectColorLocations{-1, -1}
, mTargetShader(nullptr)
, mBtnEffect{1.0f,0.5f,0.5f,1.0f}
, mIsShowBattery(true)
, mBatteryLevel(-1)
, mCalmDownTime(1.0f)
, mIsOneVolumeKey(false)
, mIsUseAnimations(false)
, mTouchpadSacleFactor(0.15f)
, mFloatingDistance(0.0f)
, mRadius(1.0f)
{
    LOGI("(%d[%p]): ctor!!", mCtrlerType, this);
    mShift.translate(1,1.5,2);
    for (uint32_t compID = 0; compID < CtrlerComp_MaxCompNumber; ++compID) {
        mCompMeshes[compID].setName(sControllerCompNames[compID]);
        mCompTexID[compID] = -1;
        mCompExistFlags[compID] = false;
        mCompStates[compID] = CtrlerBtnState_None;
    }
    mLastUpdateTime = std::chrono::system_clock::now();
    initializeGLComp();
}

Controller::~Controller()
{
    LOGI("(%d[%p]): dtor!!", mCtrlerType, this);
    //clear cache
    if (mLoadModelFuncThread.joinable() == true) {
        mLoadModelFuncThread.join();
    }
    //Don't protected because it's critial with mLoadModelFuncThread, but mLoadModelFuncThread is joined.
    if (mCachedData != nullptr) {
        WVR_ReleaseControllerModel(&mCachedData); //we will clear cached data ptr to nullptr.
    }

    mIsDataReady = false;
    mInitialized = false;
    
    releaseCtrlerModelGLComp();
    releaseGLComp();
}

void Controller::loadControllerModelAsync()
{
    std::function<void()> loadModelFunc = [this](){
        std::string threadName("AppLCM");
        pthread_t selfThread = pthread_self();
        pthread_setname_np(selfThread, threadName.c_str());

        //char tSetName[256] = {'\0'};
        //prctl(PR_GET_NAME, tSetName);
        //LOGI("STDTHREAD(%s)", tSetName);

        LOGI("(%d[%p]): In Loading Thread", mCtrlerType, this);
        mLoadingThreadMutex.lock();
        //1. Clear status and cached data(if it exist).
        {//Critical Section: Clear flag and cached parsed data.
            std::lock_guard<std::mutex> lockGuard(mCachedDataMutex);
            if (mCachedData != nullptr) {
                WVR_ReleaseControllerModel(&mCachedData); //we will clear cached data ptr to nullptr.
            }

            mIsDataReady = false;
            mInitialized = false;
        }//Critical Section: Clear flag and cached parsed data.(End)
        //2. Load ctrler model data.
        WVR_Result result = WVR_GetCurrentControllerModel(mCtrlerType, &mCachedData, false);
        if (result == WVR_Success) {
            //3. Test another API together.
            //*** if you write sample, please get emitter in returned structure instead of use this API.
            mEmitterPose = Matrix4();


            //4. Get Animation Data.
            WVR_CtrlerModelAnimData_t *animData = nullptr;
            WVR_Result result = WVR_GetCtrlerModelAnimNodeData(mCtrlerType, &animData);
            if (result == WVR_Success) {
                initializeCtrlerModelAnimData(animData);
                mIsUseAnimations = true;
            } else {
                LOGI("(%d[%p]): Load animation data fail. Reason(%d).", mCtrlerType, this, result);
                mIsUseAnimations = false;
            }
            WVR_ReleaseCtrlerModelAnimNodeData(&animData);

            {//Critical Section: Set data ready flag.
                std::lock_guard<std::mutex> lockGuard(mCachedDataMutex);
                mIsDataReady = true;
            }//Critical Section: Set data ready flag.(End)
        } else {
            LOGI("(%d[%p]): Load fail. Reason(%d)", mCtrlerType, this, result);
        }

        mLoadingThreadMutex.unlock();
    };
    //Check controller render model name.
    std::string newRenderModelName;
    uint32_t paramLength = WVR_GetParameters(mCtrlerType, "GetRenderModelName", nullptr, 0);
    newRenderModelName.resize(paramLength);
    WVR_GetParameters(mCtrlerType, "GetRenderModelName", &newRenderModelName[0], newRenderModelName.size());
    LOGI("(%d[%p]): new rm %zu",  mCtrlerType, this, std::hash<std::string>{}(newRenderModelName));
    if (newRenderModelName.compare(mCurrentRenderModelName) == 0) {
        LOGI("(%d[%p]): model name is still %zu. So don't trigger asynchornous loading.", mCtrlerType, this, std::hash<std::string>{}(mCurrentRenderModelName));
        return;
    }

    LOGI("(%d[%p]): change model name from %zu to %zu", mCtrlerType, this, std::hash<std::string>{}(mCurrentRenderModelName), std::hash<std::string>{}(newRenderModelName));
    mCurrentRenderModelName = newRenderModelName;

    //Trigger LoadModelFuncThread.
    if (mLoadModelFuncThread.joinable() == true) {
        mLoadModelFuncThread.detach();
        LOGI("(%d[%p]): Detach", mCtrlerType, this);
    }
    LOGI("(%d[%p]): Trigger Loading Thread", mCtrlerType, this);
    
    mLoadModelFuncThread = std::thread(loadModelFunc);
}

void Controller::render(DrawModeEnum iMode, const Matrix4 iProjs[DrawMode_MaxModeMumber], const Matrix4 iEyes[DrawMode_MaxModeMumber], const Matrix4 &iView, const Matrix4 &iCtrlerPose)
{
    //1. Initialize controller model if necessary.
    {//Critical Session: Initialize data block.
        std::lock_guard<std::mutex> lockGuard(mCachedDataMutex);
        if (mInitialized == false && mIsDataReady == true) {
            if (mCachedData != nullptr) {
                //Clear old data.
                releaseCtrlerModelGLComp();
                //Initialize mCachedData to gpu.
                initializeCtrlerModelGLComp();
                //Clear cached data but don't set data.
                WVR_ReleaseControllerModel(&mCachedData); //we will clear cached data ptr to nullptr.
                
                mIsDataReady = false;
                mInitialized = true;
            } else {
                LOGW("(%d[%p]): Initialize data is nullptr but data ready flag is true!!!", mCtrlerType, this);
            }
        }
    }//Critical Session: Initialize data block.(End)
    //2. draw controller model if ok.
    if (mInitialized == false || WVR_IsDeviceConnected(mCtrlerType) == false) {
        return;
    }

    refreshBatteryStatus();

    //LOGI("Ctrller(%d):draw", mCtrlerType);
    //1. cache depth and alpha setting.
    GLboolean oldDepth, oldAlpha;
    GLint oldDepthFunc;
    GLboolean lastPolygonOffsetFill;
    GLfloat lastFactor, lastUnits;
    oldDepth = glIsEnabled(GL_DEPTH_TEST);
    glGetIntegerv(GL_DEPTH_FUNC, &oldDepthFunc);
    oldAlpha = glIsEnabled(GL_BLEND);
    lastPolygonOffsetFill = glIsEnabled(GL_POLYGON_OFFSET_FILL);
    glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &lastFactor);
    glGetFloatv(GL_POLYGON_OFFSET_UNITS, &lastUnits);
    //2. draw
    Matrix4 mvps[DrawMode_MaxModeMumber];
    if (iMode == DrawMode_General) {
        mvps[0] = iProjs[0] * iEyes[0] * iView * mShift * iCtrlerPose;
    } else {
        mvps[0] = iProjs[0] * iEyes[0] * iView * mShift * iCtrlerPose;
        mvps[1] = iProjs[1] * iEyes[1] * iView * mShift * iCtrlerPose;
    }

    drawCtrlerBody(iMode, mvps);
    drawCtrlerBattery(iMode, mvps);
    drawCtrlerButtonEffect(iMode, mvps);
    drawCtrlerTouchPad(iMode, mvps);
    drawCtrlerRay(iMode, mvps);
    drawCtrlerExtraMeshes(iMode, mvps);
    //draw end.
    //3. status recovering.
    if (lastPolygonOffsetFill == GL_TRUE) {
        glEnable(GL_POLYGON_OFFSET_FILL);
    } else {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
    glPolygonOffset(lastFactor, lastUnits);

    if (oldDepth == GL_TRUE) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    glDepthFunc(oldDepthFunc);

    if (oldAlpha == GL_TRUE) {
        glEnable(GL_BLEND);
    } else {
        glDisable(GL_BLEND);
    }
}

uint32_t Controller::getCompIdxByName(const std::string &iName) const
{
    uint32_t result = E_TO_UINT(CtrlerComp_MaxCompNumber);
    for (uint32_t compID = 0; compID < CtrlerComp_MaxCompNumber; ++compID) {
        if (sControllerCompNames[compID].compare(iName) == 0) {
            return compID;
        }
    }
    return result;
}

void Controller::initializeGLComp()
{
    const char *shaderNames[2] = {
        "CtrlerShader",
        "CtrlerMultiShader"
    };
    const char *vpaths[2] ={
        "shader/vertex/ctrler_vertex.glsl",
        "shader/vertex/ctrler_multview_vertex.glsl"
    };
    const char *fpaths[2] = {
        "shader/fragment/ctrler_fragment.glsl",
        "shader/fragment/ctrler_fragment.glsl"};
    //Initialize shader.
    for (uint32_t mode = DrawMode_General; mode < DrawMode_MaxModeMumber; ++mode) {
        mShaders[mode] = Shader::findShader(vpaths[mode], fpaths[mode]);
        if (mShaders[mode] != nullptr) {
            LOGI("(%d[%p]): Shader find!!!", mCtrlerType, this);
        } else {
            Context * context = Context::getInstance();
            EnvWrapper ew = context->getEnv();
            JNIEnv * env = ew.get();

            AssetFile vfile(context->getAssetManager(), vpaths[mode]);
            AssetFile ffile(context->getAssetManager(), fpaths[mode]);
            if (!vfile.open() || !ffile.open()) {
                LOGE("(%d[%p]): Unable to read shader files!!!", mCtrlerType, this);
                return;
            }

            char *vstr = vfile.toString();
            char *fstr = ffile.toString();

            //LogD(mName, "%s\n%s\n", vpath, vstr);
            //LogD(mName, "%s\n%s\n", fpath, fstr);

            mShaders[mode] = std::make_shared<Shader>(shaderNames[mode], vpaths[mode], vstr, fpaths[mode], fstr);
            bool ret = mShaders[mode]->compile();

            delete [] vstr;
            delete [] fstr;
            if (ret == false) {
                LOGE("(%d[%p]): Compile shader error!!!", mCtrlerType, this);
            } else {
                Shader::putShader(mShaders[mode]);
            }
        }
        //
        mDiffTexLocations[mode] = mShaders[mode]->getUniformLocation("diffTexture");
        mMatrixLocations[mode]  = mShaders[mode]->getUniformLocation("matrix");
        mUseEffectLocations[mode] = mShaders[mode]->getUniformLocation("useEffect");
        mEffectColorLocations[mode] = mShaders[mode]->getUniformLocation("effectColor");
        LOGI("(%d[%p]): Mode[%d]: diffTexture(%d) matrix(%d) useEffect(%d) effectColor(%d)", mCtrlerType, this, 
            mode,
            mDiffTexLocations[mode], 
            mMatrixLocations[mode],
            mUseEffectLocations[mode],
            mEffectColorLocations[mode]);
    }
    //Ray
    float s = 0.00125f;
    float dis = 1.0f;
    float rayVertices[15] = {
           s,    s, -0.003f,
          -s,    s, -0.003f,
          -s,   -s, -0.003f,
           s,   -s, -0.003f,
        0.0f, 0.0f, -dis
    };

    float rayTexCoords[10] = {
        0 ,0,
        0 ,0,
        0 ,0,
        0 ,0,
        0 ,0
    };

    uint32_t rayIndices[18]{
        0, 1, 2,
        0, 2, 3,
        0, 4, 1,
        0, 3, 4,
        2, 4, 3,
        1, 4, 2
    };

    mRayMesh.createVertexBufferData(VertexAttrib_Vertices, rayVertices, 15, 3);
    mRayMesh.createVertexBufferData(VertexAttrib_TexCoords, rayTexCoords, 10, 2);
    mRayMesh.createIndexBufferData(rayIndices, 18, 3);
    mRayMesh.createVAO();
}

void Controller::releaseGLComp()
{
    mRayMesh.releaseGLComp();
}

void Controller::initializeCtrlerModelGLComp()
{
    //1. Initialize meshes.
    LOGI("(%d[%p]): Initialize meshes(%d)", mCtrlerType, this, (*mCachedData).compInfos.size);
    mEmitterPose = Matrix4();

    for (uint32_t wvrCompID = 0; wvrCompID < (*mCachedData).compInfos.size; ++wvrCompID) {
        uint32_t ctrlerCompID = getCompIdxByName((*mCachedData).compInfos.table[wvrCompID].name);
        if (ctrlerCompID < E_TO_UINT(CtrlerComp_MaxCompNumber)) {
            mCompMeshes[ctrlerCompID].createVertexBufferData(
                VertexAttrib_Vertices,
                (*mCachedData).compInfos.table[wvrCompID].vertices.buffer,
                (*mCachedData).compInfos.table[wvrCompID].vertices.size,
                (*mCachedData).compInfos.table[wvrCompID].vertices.dimension);

            /* //don't use normal
            mCompMeshes[ctrlerCompID].createVertexBufferData(
                VertexAttrib_Normals,
                (*mCachedData).compInfos.table[wvrCompID].normals.buffer,
                (*mCachedData).compInfos.table[wvrCompID].normals.size,
                (*mCachedData).compInfos.table[wvrCompID].normals.dimension);
            */
            
            mCompMeshes[ctrlerCompID].createVertexBufferData(
                VertexAttrib_TexCoords,
                (*mCachedData).compInfos.table[wvrCompID].texCoords.buffer,
                (*mCachedData).compInfos.table[wvrCompID].texCoords.size,
                (*mCachedData).compInfos.table[wvrCompID].texCoords.dimension);
            
            mCompMeshes[ctrlerCompID].createIndexBufferData(
                (*mCachedData).compInfos.table[wvrCompID].indices.buffer,
                (*mCachedData).compInfos.table[wvrCompID].indices.size,
                (*mCachedData).compInfos.table[wvrCompID].indices.type);

            mCompMeshes[ctrlerCompID].createVAO();
            mCompDefaultDraw[ctrlerCompID] = (*mCachedData).compInfos.table[wvrCompID].defaultDraw;
            //copy mat in ctrler space.
            mCompLocalMats[ctrlerCompID].set((*mCachedData).compInfos.table[wvrCompID].localMat);
            mCompTexID[ctrlerCompID] = (*mCachedData).compInfos.table[wvrCompID].texIndex;
            mCompExistFlags[ctrlerCompID] = true;

            if (ctrlerCompID == CtrlerComp_Emitter) {
                mEmitterPose = mCompLocalMats[ctrlerCompID];
            }
        } else {
            bool isExist = false;
            std::list<ExtraMesh>::iterator emIter;
            for (emIter = mExtraMeshes.begin(); emIter != mExtraMeshes.end(); ++emIter) {
                if ((*emIter).mName.compare((*mCachedData).compInfos.table[wvrCompID].name) == 0) {
                    //Shouldn't enter here. API won't return deplicated component.
                    isExist = true;
                    (*emIter).mDefaultDraw = (*mCachedData).compInfos.table[wvrCompID].defaultDraw;
                    break;
                }
            }
            LOGI("(%d[%p]) : Comp[%s] doesn't support in current controller model. Add to extra is isExist == false(real status:%d).", mCtrlerType, this, (*mCachedData).compInfos.table[wvrCompID].name, isExist);
            
            if (isExist == false) {
                mExtraMeshes.push_back(ExtraMesh());
                std::list<ExtraMesh>::iterator lastEleIter = mExtraMeshes.end();
                --lastEleIter;
                ExtraMesh &newExtraMesh = (*lastEleIter);
                newExtraMesh.mName = (*mCachedData).compInfos.table[wvrCompID].name;
                newExtraMesh.mDefaultDraw = (*mCachedData).compInfos.table[wvrCompID].defaultDraw;
                newExtraMesh.mMesh.createVertexBufferData(
                    VertexAttrib_Vertices,
                    (*mCachedData).compInfos.table[wvrCompID].vertices.buffer,
                    (*mCachedData).compInfos.table[wvrCompID].vertices.size,
                    (*mCachedData).compInfos.table[wvrCompID].vertices.dimension);
                
                /* //don't use normal
                newExtraMesh.mMesh.createVertexBufferData(
                    VertexAttrib_Normals,
                    (*mCachedData).compInfos.table[wvrCompID].normals.buffer,
                    (*mCachedData).compInfos.table[wvrCompID].normals.size,
                    (*mCachedData).compInfos.table[wvrCompID].normals.dimension);
                */
                
                newExtraMesh.mMesh.createVertexBufferData(
                    VertexAttrib_TexCoords,
                    (*mCachedData).compInfos.table[wvrCompID].texCoords.buffer,
                    (*mCachedData).compInfos.table[wvrCompID].texCoords.size,
                    (*mCachedData).compInfos.table[wvrCompID].texCoords.dimension);
                
                newExtraMesh.mMesh.createIndexBufferData(
                    (*mCachedData).compInfos.table[wvrCompID].indices.buffer,
                    (*mCachedData).compInfos.table[wvrCompID].indices.size,
                    (*mCachedData).compInfos.table[wvrCompID].indices.type);
                
                newExtraMesh.mMesh.createVAO();
                newExtraMesh.mLocalMat.set((*mCachedData).compInfos.table[wvrCompID].localMat);
                newExtraMesh.mTexID = (*mCachedData).compInfos.table[wvrCompID].texIndex;
            }
        }
    }

    //1.1 one volume key check.
    if (mCompExistFlags[CtrlerComp_VolumeKey] == true && 
        mCompExistFlags[CtrlerComp_VolumeUpKey] == false &&
        mCompExistFlags[CtrlerComp_VolumeDownKey] == false ) {
        mIsOneVolumeKey = true;
    } else {
        mIsOneVolumeKey = false;
    }

    //1.2 calculate touchpad plane matrix.
    mRadius = (*mCachedData).touchpadPlane.radius;
    mFloatingDistance = (*mCachedData).touchpadPlane.floatingDistance;

    mTouchPadPlaneMat[ 0] = (*mCachedData).touchpadPlane.u.v[0];
    mTouchPadPlaneMat[ 1] = (*mCachedData).touchpadPlane.u.v[1];
    mTouchPadPlaneMat[ 2] = (*mCachedData).touchpadPlane.u.v[2];
    mTouchPadPlaneMat[ 3] = 0.0f;
    
    mTouchPadPlaneMat[ 4] = (*mCachedData).touchpadPlane.v.v[0];
    mTouchPadPlaneMat[ 5] = (*mCachedData).touchpadPlane.v.v[1];
    mTouchPadPlaneMat[ 6] = (*mCachedData).touchpadPlane.v.v[2];
    mTouchPadPlaneMat[ 7] = 0.0f;

    mTouchPadPlaneMat[ 8] = (*mCachedData).touchpadPlane.w.v[0];
    mTouchPadPlaneMat[ 9] = (*mCachedData).touchpadPlane.w.v[1];
    mTouchPadPlaneMat[10] = (*mCachedData).touchpadPlane.w.v[2];
    mTouchPadPlaneMat[11] = 0.0f;

    mTouchPadPlaneMat[12] = (*mCachedData).touchpadPlane.center.v[0] + mFloatingDistance * mTouchPadPlaneMat[4];
    mTouchPadPlaneMat[13] = (*mCachedData).touchpadPlane.center.v[1] + mFloatingDistance * mTouchPadPlaneMat[5];
    mTouchPadPlaneMat[14] = (*mCachedData).touchpadPlane.center.v[2] + mFloatingDistance * mTouchPadPlaneMat[6];
    mTouchPadPlaneMat[15] = 1.0f;

    mIsNeedRevertInputY = ((*mCachedData).touchpadPlane.valid == false);

    //2. Initialize texture.
    uint32_t wvrBitmapSize = (*mCachedData).bitmapInfos.size;
    mTextureTable.resize(wvrBitmapSize);
    LOGI("(%d[%p]): Initialize WVRTextures(%d)", mCtrlerType, this, wvrBitmapSize);
    for (uint32_t texID = 0; texID < wvrBitmapSize; ++texID) {
        mTextureTable[texID] = nullptr;
        mTextureTable[texID] = Texture::loadTextureFromBitmapWithoutCached(
            (*mCachedData).bitmapInfos.table[texID]);
    }

    //3. Intialize Battery Texture.
    mBatLvTex.resize((*mCachedData).batteryLevels.size);
    mBatMinLevels.resize((*mCachedData).batteryLevels.size);
    mBatMaxLevels.resize((*mCachedData).batteryLevels.size);
    for (uint32_t lv = 0; lv < (*mCachedData).batteryLevels.size; ++lv) {
        mBatMinLevels[lv] = (*mCachedData).batteryLevels.minLvTable[lv];
        mBatMaxLevels[lv] = (*mCachedData).batteryLevels.maxLvTable[lv];
        if ((*mCachedData).batteryLevels.texTable[lv].bitmap != nullptr) {
            mBatLvTex[lv] = Texture::loadTextureFromBitmapWithoutCached(
                (*mCachedData).batteryLevels.texTable[lv]);
        }
    }

    LOGI("(%d[%p]): Initialize End!!!", mCtrlerType, this);
}

void Controller::initializeCtrlerModelAnimData(WVR_CtrlerModelAnimData_t *iAnimData)
{
    if (iAnimData != nullptr) {
        for (uint32_t wvrCompID = 0; wvrCompID < (*iAnimData).size; ++wvrCompID) {
            uint32_t ctrlerCompID = getCompIdxByName((*iAnimData).animDatas[wvrCompID].name);
            if (ctrlerCompID < E_TO_UINT(CtrlerComp_MaxCompNumber)) {
                mAnimationNodes[ctrlerCompID].setAnimation((*iAnimData).animDatas[wvrCompID]);
            }
        }
    }
}

void Controller::drawCtrlerBody(DrawModeEnum iMode, const Matrix4 iMVPs[DrawMode_MaxModeMumber])
{
    glEnable(GL_DEPTH_TEST);

    uint32_t ctrlerCompID = CtrlerComp_Body;
    Matrix4 finalMats[2];
    uint32_t matNumber = 1;
    std::vector<GLfloat> glMats;
    if (iMode == DrawMode_General) {
        finalMats[0] = iMVPs[0] * mCompLocalMats[ctrlerCompID];
        matNumber = 1;
        glMats.resize(16);
        memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
    } else {
        finalMats[0] = iMVPs[0] * mCompLocalMats[ctrlerCompID];
        finalMats[1] = iMVPs[1] * mCompLocalMats[ctrlerCompID];
        matNumber = 2;
        glMats.resize(32);
        memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
        memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
    }

    mTargetShader = mShaders[iMode].get();

    if (mTargetShader != nullptr) {      
        mTargetShader->useProgram();
        glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());

        if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {
            glActiveTexture(GL_TEXTURE0);
            mTextureTable[mCompTexID[ctrlerCompID]]->bindTexture();
            glUniform1i(mDiffTexLocations[iMode], 0);
        }

        glUniform1i(mUseEffectLocations[iMode], 0);
        glUniform4f(mEffectColorLocations[iMode], 1.0f, 1.0f, 1.0f, 1.0f);
        mCompMeshes[ctrlerCompID].draw();

        if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {
            mTextureTable[mCompTexID[ctrlerCompID]]->unbindTexture();
        }
        mTargetShader->unuseProgram();
    }

    glDisable(GL_DEPTH_TEST);
}

void Controller::drawCtrlerBattery(DrawModeEnum iMode, const Matrix4 iMVPs[DrawMode_MaxModeMumber])
{
    if (mIsShowBattery == false || mCompExistFlags[CtrlerComp_Battery] == false) {
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(
        GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,
        GL_ONE, GL_ONE);

    uint32_t ctrlerCompID = CtrlerComp_Battery;

    Matrix4 finalMats[2];
    uint32_t matNumber = 1;
    std::vector<GLfloat> glMats;
    if (iMode == DrawMode_General) {
        finalMats[0] = iMVPs[0] * mCompLocalMats[ctrlerCompID];
        matNumber = 1;
        glMats.resize(16);
        memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
    } else {
        finalMats[0] = iMVPs[0] * mCompLocalMats[ctrlerCompID];
        finalMats[1] = iMVPs[1] * mCompLocalMats[ctrlerCompID];
        matNumber = 2;
        glMats.resize(32);
        memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
        memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
    }

    mTargetShader = mShaders[iMode].get();

    if (mTargetShader != nullptr) {
        if (mBatteryLevel >= 0 && mBatteryLevel < mBatLvTex.size()) {
            if (mBatLvTex[mBatteryLevel] != nullptr) {
                //draw.
                mTargetShader->useProgram();
                glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());
                glActiveTexture(GL_TEXTURE0);
                mBatLvTex[mBatteryLevel]->bindTexture();
                glUniform1i(mDiffTexLocations[iMode], 0);
                glUniform1i(mUseEffectLocations[iMode], 0);
                glUniform4f(mEffectColorLocations[iMode], 1.0f, 1.0f, 1.0f, 1.0f);
                //
                mCompMeshes[ctrlerCompID].draw();
                //
                mBatLvTex[mBatteryLevel]->unbindTexture();
                mTargetShader->unuseProgram();
            }
        }
    }

    glBlendFuncSeparate(
        GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,
        GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
}

void Controller::drawCtrlerTouchPad(DrawModeEnum iMode, const Matrix4 iMVPs[DrawMode_MaxModeMumber])
{
    glEnable(GL_DEPTH_TEST);

    if (mCompExistFlags[CtrlerComp_TouchPad] == true) {
        //draw touchpad_touch
        if (mCompStates[CtrlerComp_TouchPad] == CtrlerBtnState_Tapped && mCompExistFlags[CtrlerComp_TouchPad_Touch] == true) {
            WVR_Axis_t axis = WVR_GetInputAnalogAxis(mCtrlerType, WVR_InputId_Alias1_Touchpad);
            //1. calculate touchpad touch pos.
            float invAxisY = 1.0f;
            if (mIsNeedRevertInputY == true) {
                invAxisY = -1.0f;
            }

            Vector3 Tp;
            //Pout = Pc + Ax * R + Ay * R;
            Tp.x = axis.x * mRadius;
            Tp.y = mFloatingDistance;
            Tp.z = invAxisY * axis.y * mRadius;
            Matrix4 offsetMat;
            offsetMat[12] = Tp.x;
            offsetMat[13] = Tp.y;
            offsetMat[14] = Tp.z;
            offsetMat[15] = 1.0f;

            Matrix4 dotFinalMat;
            
            //2. draw touchpad dot.
            //2.1 find ctrler touchpad model. (touchpad in its comp space should be identity)
            uint32_t ctrlerCompID = 0;
            ctrlerCompID = CtrlerComp_TouchPad_Touch;
            dotFinalMat = mTouchPadPlaneMat * offsetMat;

            Matrix4 finalMats[2];
            uint32_t matNumber = 1;
            std::vector<GLfloat> glMats;
            if (iMode == DrawMode_General) {
                finalMats[0] = iMVPs[0] * dotFinalMat;
                matNumber = 1;
                glMats.resize(16);
                memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
            } else {
                finalMats[0] = iMVPs[0] * dotFinalMat;
                finalMats[1] = iMVPs[1] * dotFinalMat;
                matNumber = 2;
                glMats.resize(32);
                memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
                memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
            }

            mTargetShader = mShaders[iMode].get();

            //2.2 draw.
            glDisable(GL_CULL_FACE);
            mTargetShader->useProgram();
            glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());

            if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {
                glActiveTexture(GL_TEXTURE0);
                mTextureTable[mCompTexID[ctrlerCompID]]->bindTexture();
                glUniform1i(mDiffTexLocations[iMode], 0);
            }

            glUniform1i(mUseEffectLocations[iMode], 1);
            glUniform4f(mEffectColorLocations[iMode], mBtnEffect[0], mBtnEffect[1], mBtnEffect[2], mBtnEffect[3]);

            mCompMeshes[ctrlerCompID].draw();

            if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {
                mTextureTable[mCompTexID[ctrlerCompID]]->unbindTexture();
            }

            mTargetShader->unuseProgram();
            glEnable(GL_CULL_FACE);
        }
        //draw touchpad
        if (mCompStates[CtrlerComp_TouchPad] == CtrlerBtnState_Pressed || mCompDefaultDraw[CtrlerComp_TouchPad] == true) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(0.0f, -100.0f); // -100.0 units means push the depth forward 100 units.

            uint32_t ctrlerCompID = CtrlerComp_TouchPad;
            
            Matrix4 finalMats[2];
            uint32_t matNumber = 1;
            std::vector<GLfloat> glMats;
            if (iMode == DrawMode_General) {
                finalMats[0] = iMVPs[0] * mCompLocalMats[ctrlerCompID];
                matNumber = 1;
                glMats.resize(16);
                memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
            } else {
                finalMats[0] = iMVPs[0] * mCompLocalMats[ctrlerCompID];
                finalMats[1] = iMVPs[1] * mCompLocalMats[ctrlerCompID];
                matNumber = 2;
                glMats.resize(32);
                memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
                memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
            }

            mTargetShader = mShaders[iMode].get();
            if (mTargetShader != nullptr) {
                mTargetShader->useProgram();
                glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());

                if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {   
                    glActiveTexture(GL_TEXTURE0);
                    mTextureTable[mCompTexID[ctrlerCompID]]->bindTexture();
                    glUniform1i(mDiffTexLocations[iMode], 0);
                }
                
                if (mCompStates[CtrlerComp_TouchPad] == CtrlerBtnState_Pressed) {
                    glUniform1i(mUseEffectLocations[iMode], 1);
                    glUniform4f(mEffectColorLocations[iMode], mBtnEffect[0], mBtnEffect[1], mBtnEffect[2], mBtnEffect[3]);
                } else {
                    glUniform1i(mUseEffectLocations[iMode], 0);
                    glUniform4f(mEffectColorLocations[iMode], 1.0f, 1.0f, 1.0f, 1.0f);
                }

                mCompMeshes[ctrlerCompID].draw();

                if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {
                    mTextureTable[mCompTexID[ctrlerCompID]]->unbindTexture();
                }

                mTargetShader->unuseProgram();
            }

            glDisable(GL_POLYGON_OFFSET_FILL);
        }
    }

    glDisable(GL_DEPTH_TEST);
}

void Controller::drawCtrlerButtonEffect(DrawModeEnum iMode, const Matrix4 iMVPs[DrawMode_MaxModeMumber])
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(0.0f, -100.0f); // -100.0 units means push the depth forward 100 units.

    for (uint32_t ctrlerCompID = CtrlerComp_AppButton; ctrlerCompID < CtrlerComp_MaxCompNumber; ++ctrlerCompID) {
        //Don't draw non button effect.
        if (ctrlerCompID == CtrlerComp_TouchPad_Touch ||
            ctrlerCompID == CtrlerComp_BeamOrigin ||
            ctrlerCompID == CtrlerComp_Emitter ||
            ctrlerCompID == CtrlerComp_Battery) {
            continue;
        }

        if (mCompExistFlags[ctrlerCompID] == true) {
            Matrix4 finalMats[2];
            uint32_t matNumber = 1;
            std::vector<GLfloat> glMats;
            Matrix4 finalCompMat;
            bool isBtnPressed = (mCompStates[ctrlerCompID] == CtrlerBtnState_Pressed);

            if (mIsUseAnimations == false) {
                finalCompMat = mCompLocalMats[ctrlerCompID];
            } else {
                if (mAnimationNodes[ctrlerCompID].isValid() == true) {
                    WVR_Axis_t axis = WVR_GetInputAnalogAxis(mCtrlerType, sControllerCompIDTable[ctrlerCompID]);
                    finalCompMat = mAnimationNodes[ctrlerCompID].getAnimationMatrix(axis.x, axis.y, isBtnPressed);
                } else {
                    finalCompMat = mCompLocalMats[ctrlerCompID];
                }
            }

            if (iMode == DrawMode_General) {
                finalMats[0] = iMVPs[0] * finalCompMat;
                matNumber = 1;
                glMats.resize(16);
                memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
            } else {
                finalMats[0] = iMVPs[0] * finalCompMat;
                finalMats[1] = iMVPs[1] * finalCompMat;
                matNumber = 2;
                glMats.resize(32);
                memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
                memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
            }

            mTargetShader = mShaders[iMode].get();

            if (mTargetShader != nullptr) {
                if (isBtnPressed == true || mCompDefaultDraw[ctrlerCompID] == true) {
                    mTargetShader->useProgram();
                    glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());
                    if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {
                        glActiveTexture(GL_TEXTURE0);
                        mTextureTable[mCompTexID[ctrlerCompID]]->bindTexture();
                        glUniform1i(mDiffTexLocations[iMode], 0);
                    }

                    if (mIsUseAnimations == false) {
                        if (isBtnPressed == true) {
                            glUniform1i(mUseEffectLocations[iMode], 1);
                            glUniform4f(mEffectColorLocations[iMode], mBtnEffect[0], mBtnEffect[1], mBtnEffect[2], mBtnEffect[3]);
                        } else {
                            glUniform1i(mUseEffectLocations[iMode], 0);
                            glUniform4f(mEffectColorLocations[iMode], 1.0f, 1.0f, 1.0f, 1.0f);
                        }
                    } else {
                        if (isBtnPressed == true && mAnimationNodes[ctrlerCompID].mUseBlueEffect == true) {
                            glUniform1i(mUseEffectLocations[iMode], 1);
                            glUniform4f(mEffectColorLocations[iMode], mBtnEffect[0], mBtnEffect[1], mBtnEffect[2], mBtnEffect[3]);
                        } else {
                            glUniform1i(mUseEffectLocations[iMode], 0);
                            glUniform4f(mEffectColorLocations[iMode], 1.0f, 1.0f, 1.0f, 1.0f);
                        }
                    }

                    mCompMeshes[ctrlerCompID].draw();

                    if (mCompTexID[ctrlerCompID] >= 0 && mCompTexID[ctrlerCompID] < mTextureTable.size()) {
                        mTextureTable[mCompTexID[ctrlerCompID]]->unbindTexture();
                    }

                    mTargetShader->unuseProgram();
                }
            }
        }
    }

    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_DEPTH_TEST);
}

void Controller::drawCtrlerRay(DrawModeEnum iMode, const Matrix4 iMVPs[DrawMode_MaxModeMumber])
{
    glEnable(GL_DEPTH_TEST);

    Matrix4 finalMats[2];
    uint32_t matNumber = 1;
    std::vector<GLfloat> glMats;
    if (iMode == DrawMode_General) {
        finalMats[0] = iMVPs[0] * mEmitterPose;
        matNumber = 1;
        glMats.resize(16);
        memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
    } else {
        finalMats[0] = iMVPs[0] * mEmitterPose;
        finalMats[1] = iMVPs[1] * mEmitterPose;
        matNumber = 2;
        glMats.resize(32);
        memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
        memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
    }

    mTargetShader = mShaders[iMode].get();

    if (mTargetShader != nullptr) {
        mTargetShader->useProgram();

        glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());
        glUniform1i(mDiffTexLocations[iMode], 0);
        glUniform1i(mUseEffectLocations[iMode], 1);
        glUniform4f(mEffectColorLocations[iMode], mBtnEffect[0], mBtnEffect[1], mBtnEffect[2], mBtnEffect[3]);
        //
        mRayMesh.draw();
        //
        mTargetShader->unuseProgram();
    }

    glDisable(GL_DEPTH_TEST);
}


void Controller::drawCtrlerExtraMeshes(DrawModeEnum iMode, const Matrix4 iMVPs[DrawMode_MaxModeMumber])
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(
        GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,
        GL_ONE, GL_ONE);


    mTargetShader = mShaders[iMode].get();

    std::list<ExtraMesh>::iterator emIter;
    for (emIter = mExtraMeshes.begin(); emIter != mExtraMeshes.end(); ++emIter) {
        if ((*emIter).mDefaultDraw == true) {
            Matrix4 finalMats[2];
            uint32_t matNumber = 1;
            std::vector<GLfloat> glMats;
            if (iMode == DrawMode_General) {
                finalMats[0] = iMVPs[0] * (*emIter).mLocalMat;
                matNumber = 1;
                glMats.resize(16);
                memcpy(glMats.data(), finalMats[0].get(), 16 * sizeof(GLfloat));
            } else {
                finalMats[0] = iMVPs[0] * (*emIter).mLocalMat;
                finalMats[1] = iMVPs[1] * (*emIter).mLocalMat;
                matNumber = 2;
                glMats.resize(32);
                memcpy(glMats.data()     , finalMats[0].get(), 16 * sizeof(GLfloat));
                memcpy(glMats.data() + 16, finalMats[1].get(), 16 * sizeof(GLfloat));
            }

            mTargetShader->useProgram();

            glUniformMatrix4fv(mMatrixLocations[iMode], matNumber, false, glMats.data());

            glActiveTexture(GL_TEXTURE0);
            if ((*emIter).mTexID >= 0 && (*emIter).mTexID < mTextureTable.size()) {
                mTextureTable[(*emIter).mTexID]->bindTexture();
            }

            glUniform1i(mDiffTexLocations[iMode], 0);
            glUniform1i(mUseEffectLocations[iMode], 0);
            glUniform4f(mEffectColorLocations[iMode], 1.0f, 1.0f, 1.0f, 1.0f);

            (*emIter).mMesh.draw();

            if ((*emIter).mTexID >= 0 && (*emIter).mTexID < mTextureTable.size()) {
                mTextureTable[(*emIter).mTexID]->unbindTexture();
            }

            mTargetShader->unuseProgram();
        }
    }

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
}

void Controller::releaseCtrlerModelGLComp()
{
    for (uint32_t ctrlerCompID = 0; ctrlerCompID < CtrlerComp_MaxCompNumber; ++ctrlerCompID) {
        mCompMeshes[ctrlerCompID].releaseGLComp();
    }

    std::list<ExtraMesh>::iterator emIter;
    for (emIter = mExtraMeshes.begin(); emIter != mExtraMeshes.end(); ++emIter) {
        (*emIter).mMesh.releaseGLComp();
    }

    mExtraMeshes.clear();

    LOGI("(%d[%p]): release meshes done.", mCtrlerType, this);

    for (uint32_t texID = 0; texID < mTextureTable.size(); ++texID) {
        if (mTextureTable[texID] != nullptr) {
            delete mTextureTable[texID];
        }
        mTextureTable[texID] = nullptr;
    }
    mTextureTable.clear();
    LOGI("(%d[%p]): release texture done.", mCtrlerType, this);

    for (uint32_t compID = 0; compID < CtrlerComp_MaxCompNumber; ++compID) {
        mCompTexID[compID] = -1;
        mCompExistFlags[compID] = false;
        mCompStates[compID] = CtrlerBtnState_None;
    }

    for (uint32_t lv = 0; lv < mBatLvTex.size(); ++lv) {
        if (mBatLvTex[lv] != nullptr) {
            delete mBatLvTex[lv];
        }
        mBatLvTex[lv] = nullptr;
    }
    mBatLvTex.clear();
    mBatLvTex.shrink_to_fit();
    mBatMinLevels.clear();
    mBatMinLevels.shrink_to_fit();
    mBatMaxLevels.clear();
    mBatMaxLevels.shrink_to_fit();
    LOGI("(%d[%p]): release battery done.", mCtrlerType, this);
}

bool Controller::isThisCtrlerType(WVR_DeviceType iCtrlerType) const
{
    return (iCtrlerType == mCtrlerType);
}

WVR_DeviceType Controller::getCtrlerType() const
{
    return mCtrlerType;
}

void Controller::switchCtrlerType()
{
    WVR_DeviceType oldType = mCtrlerType;
    if (mCtrlerType == WVR_DeviceType_Controller_Left) {
        mCtrlerType = WVR_DeviceType_Controller_Right;
    } else {
        mCtrlerType = WVR_DeviceType_Controller_Left;
    }
    LOGI("(%d[%p]): switch ctrler type from(%d) to (%d)", mCtrlerType, this, oldType, mCtrlerType);
}

void Controller::resetButtonEffects()
{
    for (uint32_t compID = 0; compID < CtrlerComp_MaxCompNumber; ++compID) {
        mCompStates[compID] = CtrlerBtnState_None;
    }
}

void Controller::handleDisconnected()
{
    //release controller gl resource.
    LOGI("(%d[%p]): Disconnected.", mCtrlerType, this);
    mCurrentRenderModelName = "";
    mCurrentRenderModelName.shrink_to_fit();
    releaseCtrlerModelGLComp();
    //Critical section.
    {
        std::lock_guard<std::mutex> lockGuard(mCachedDataMutex);
        mInitialized = false;
    }
}

void Controller::refreshButtonStatus(const WVR_Event_t &iEvent)
{
    uint32_t eventBtnID;
    CtrlerCompEnum btnID;
    uint32_t eventBtnActionID;
    CtrlerBtnStateEnum btnState;
    WVR_InputMappingPair element;
    //Btn
    if (WVR_GetInputMappingPair(iEvent.input.device.deviceType, iEvent.input.inputId, &element) == true) {
        LOGI("controller input id (source ,dest) is : (%d, %d)",static_cast<uint32_t>(element.source.id), static_cast<uint32_t>(iEvent.input.inputId));
        eventBtnID = static_cast<uint32_t>(element.source.id);
    } else {
        LOGI("WVR_GetInputMappingPair return false");
		eventBtnID = static_cast<uint32_t>(iEvent.input.inputId);
    }

    switch (eventBtnID) {
    case WVR_InputId_Alias1_System:
        btnID = CtrlerComp_HomeButton;
        break;
    case WVR_InputId_Alias1_Menu:
        btnID = CtrlerComp_AppButton;
        break;
    case WVR_InputId_Alias1_Volume_Up:
        if (mIsOneVolumeKey == true) {
            btnID = CtrlerComp_VolumeKey;
        } else {
            btnID = CtrlerComp_VolumeUpKey;
        }
        break;
    case WVR_InputId_Alias1_Volume_Down:
        if (mIsOneVolumeKey == true) {
            btnID = CtrlerComp_VolumeKey;
        } else {
            btnID = CtrlerComp_VolumeDownKey;
        }
        break;
    case WVR_InputId_Alias1_Touchpad:
        btnID = CtrlerComp_TouchPad;
        break;
    case WVR_InputId_Alias1_Bumper:
        //20200227 Deprecate DigitalTrigger and replace it by Bumper.(start at runtime 3.2.0)
        btnID = CtrlerComp_BumperKey;
        break;
    case WVR_InputId_Alias1_Trigger:
        btnID = CtrlerComp_TriggerKey;
        break;
    case WVR_InputId_Alias1_Grip:
        btnID = CtrlerComp_Grip;
        break;
    case WVR_InputId_Alias1_DPad_Down:
        btnID = CtrlerComp_DPad_Down;
        break;
    case WVR_InputId_Alias1_DPad_Up:
        btnID = CtrlerComp_DPad_Up;
        break;
    case WVR_InputId_Alias1_DPad_Left:
        btnID = CtrlerComp_DPad_Left;
        break;
    case WVR_InputId_Alias1_DPad_Right:
        btnID = CtrlerComp_DPad_Right;
        break;
    case WVR_InputId_Alias1_Thumbstick:
        btnID = CtrlerComp_Thumbstick;
        break;
    case WVR_InputId_Alias1_A:
        btnID = CtrlerComp_ButtonA;
        break;
    case WVR_InputId_Alias1_B:
        btnID = CtrlerComp_ButtonB;
        break;
    case WVR_InputId_Alias1_X:
        btnID = CtrlerComp_ButtonX;
        break;
    case WVR_InputId_Alias1_Y:
        btnID = CtrlerComp_ButtonY;
        break;
    default:
        btnID = CtrlerComp_MaxCompNumber;
        break;
    }


    //ActionID
    eventBtnActionID = (uint32_t)iEvent.input.device.common.type;
    switch(eventBtnActionID){
    case WVR_EventType_TouchTapped :
    {
        btnState = CtrlerBtnState_Tapped;
    }
    break;
    case WVR_EventType_ButtonPressed :
    {
        btnState = CtrlerBtnState_Pressed;
    }
    break;
    case WVR_EventType_ButtonUnpressed :
    {
        //Unpressed will go back to unpaded.
        btnState = CtrlerBtnState_Tapped;
    }
    break;
    case WVR_EventType_TouchUntapped :
    {
        //Touch untapped means the btn is not touched.
        btnState = CtrlerBtnState_None;
    }
    break;
    default :
    {
        btnState = CtrlerBtnState_None;
    }
    break;
    }

    if (btnID < CtrlerComp_MaxCompNumber) {
        mCompStates[static_cast<uint32_t>(btnID)] = btnState;
        //20200227 If the btn is BumperKey, we need to update action state for DigitalTrigger. (Runtime < 3.2.0)
        if (btnID == CtrlerComp_BumperKey) {
            mCompStates[static_cast<uint32_t>(CtrlerComp_DigitalTriggerKey)] = btnState;
        }
    }
}

//---- protected function.
void Controller::refreshBatteryStatus()
{
    if (mCompExistFlags[CtrlerComp_Battery] == false) {
        return;
    }
    std::chrono::system_clock::time_point current = std::chrono::system_clock::now();
    float diffs = std::chrono::duration_cast<std::chrono::seconds>(
        current - mLastUpdateTime).count();
    if (diffs >= mCalmDownTime) {
        mLastUpdateTime = current;
        float power = WVR_GetDeviceBatteryPercentage(mCtrlerType);
        for (uint32_t lv = 0; lv < mBatMinLevels.size(); ++lv) {
            uint32_t percentage = static_cast<uint32_t>(power * 100.0f);
            if (percentage >= mBatMinLevels[lv] && percentage <= mBatMaxLevels[lv]) {
                mBatteryLevel = lv;
                break;
            }
        }
    }
}

Matrix4 Controller::getEmitterPose()
{
    return mEmitterPose;
}
