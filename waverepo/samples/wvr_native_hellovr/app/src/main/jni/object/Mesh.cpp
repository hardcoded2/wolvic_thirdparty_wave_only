#define LOG_TAG "APMesh"

#include <GLES3/gl31.h>
#include <GLES3/gl3ext.h>
#include <log.h>

#include "Mesh.h"

#define E_TO_UINT(enum) static_cast<uint32_t>(enum)

Mesh::Mesh()
: mVAttribBuffers{0}
, mVAttribDimension{0}
, mIndicesBuffer(0)
, mIndiceSize(0)
, mFaceType(0)
, mVAOID(0)
{
}

Mesh::~Mesh()
{
    releaseGLComp();
}

void Mesh::setName(const std::string &iName)
{
    mName = iName;
}

std::string Mesh::getName() const
{
    return mName;
}

void Mesh::createVertexBufferData(VertexAttribEnum iVALocation, float *iData, uint32_t iSize, uint32_t iDimension)
{
    if (iVALocation == VertexAttrib_MaxDefineValue || iData == nullptr || iSize == 0 || iDimension == 0) {
        LOGE("Parameter invalid!!! iVALocation(%d), iData(%p), iSize(%u), iDim(%u)",
            iVALocation, iData, iSize, iDimension);
        return;
    }
    uint32_t &vaBufID = mVAttribBuffers[E_TO_UINT(iVALocation)];
    uint32_t &vaBufDim = mVAttribDimension[E_TO_UINT(iVALocation)];
    //1. delete old buffer.
    if (glIsBuffer(vaBufID) == GL_TRUE) {
        glDeleteBuffers(1, &vaBufID);
        vaBufID = 0;
        vaBufDim = 0;
    }
    //2. allocate new buffer
    vaBufDim = iDimension;
    glGenBuffers(1, &vaBufID);
    glBindBuffer(GL_ARRAY_BUFFER, vaBufID);
    glBufferData(GL_ARRAY_BUFFER, iSize * sizeof(float), iData, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    LOGI("Create VA(%d)[%d]", iVALocation, vaBufID);
}

void Mesh::createVertexBoneIDData(const uint32_t *iData, uint32_t iSize, uint32_t iDimension)
{
    uint32_t &vaBufID = mVAttribBuffers[E_TO_UINT(VertexAttrib_BoneID)];
    uint32_t &vaBufDim = mVAttribDimension[E_TO_UINT(VertexAttrib_BoneID)];
    //1. delete old buffer.
    if (glIsBuffer(vaBufID) == GL_TRUE) {
        glDeleteBuffers(1, &vaBufID);
        vaBufID = 0;
        vaBufDim = 0;
    }
    //2. allocate new buffer
    vaBufDim = iDimension;
    glGenBuffers(1, &vaBufID);
    glBindBuffer(GL_ARRAY_BUFFER, vaBufID);
    glBufferData(GL_ARRAY_BUFFER, iSize * sizeof(uint32_t), iData, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    LOGI("Create VA(%d)[%d]", VertexAttrib_BoneID, vaBufID);
}

void Mesh::createIndexBufferData(uint32_t *iData, uint32_t iSize, uint32_t iType)
{
    if (iData == nullptr || iSize == 0 || iType == 0) {
        LOGE("Parameter invalid!!! iData(%p), iSize(%u), iType(%u)", iData, iSize, iType);
        return;
    }
    //1. delete old buffer.
    if (glIsBuffer(mIndicesBuffer) == GL_TRUE) {
        glDeleteBuffers(1, &mIndicesBuffer);
        mIndicesBuffer = 0;
        mFaceType = 0;
    }
    //2. allocate new buffer
    mFaceType = iType;
    mIndiceSize = iSize;
    glGenBuffers(1, &mIndicesBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, iSize * sizeof(uint32_t), iData, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Mesh::createVAO()
{
    if (glIsVertexArray(mVAOID) == GL_TRUE) {
        glDeleteVertexArrays(1, &mVAOID);
    }
    glGenVertexArrays(1, &mVAOID);
    glBindVertexArray(mVAOID);

    for (uint32_t vaID = 0; vaID < VertexAttrib_MaxDefineValue; ++vaID) {
        uint32_t &vaBufID = mVAttribBuffers[vaID];
        uint32_t &vaBufDim = mVAttribDimension[vaID];
        if (glIsBuffer(vaBufID) == GL_TRUE) {
            glBindBuffer(GL_ARRAY_BUFFER, vaBufID);
            glEnableVertexAttribArray(vaID);
            glVertexAttribPointer(vaID, vaBufDim, GL_FLOAT, GL_FALSE, sizeof(float) * vaBufDim, 0);
        }
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBuffer);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    /*
    LOGI("M[%s] createVAO[%d](v:%d,n:%d,t:%d,c:%d,i:%d)",
        mName.c_str(),
        mVAOID,
        mVAttribBuffers[0],
        mVAttribBuffers[1],
        mVAttribBuffers[2],
        mVAttribBuffers[3],
        mIndicesBuffer);
    //*/
}

void Mesh::draw()
{
    for (uint32_t vaID = 0; vaID < VertexAttrib_MaxDefineValue; ++vaID) {
        uint32_t &vaBufID = mVAttribBuffers[vaID];
        uint32_t &vaBufDim = mVAttribDimension[vaID];
        if (glIsBuffer(vaBufID) == GL_TRUE) {
            glBindBuffer(GL_ARRAY_BUFFER, vaBufID);
            glEnableVertexAttribArray(vaID);
            if (vaID != VertexAttrib_BoneID) {
                glVertexAttribPointer(vaID, vaBufDim, GL_FLOAT, GL_FALSE, sizeof(float) * vaBufDim, 0);
            } else {
                glVertexAttribPointer(vaID, vaBufDim, GL_UNSIGNED_INT, GL_FALSE, sizeof(uint32_t) * vaBufDim, 0);
            }
        }
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndicesBuffer);
    glDrawElements(GL_TRIANGLES, mIndiceSize, GL_UNSIGNED_INT, 0 );
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Mesh::releaseGLComp()
{
    
    for (uint32_t vaID; vaID < VertexAttrib_MaxDefineValue; ++vaID) {
        if (glIsBuffer(mVAttribBuffers[vaID]) == GL_TRUE) {
            glDeleteBuffers(1, &mVAttribBuffers[vaID]);
        }
        mVAttribBuffers[vaID] = 0;
    }

    if (glIsBuffer(mIndicesBuffer) == GL_TRUE) {
        glDeleteBuffers(1, &mIndicesBuffer);
    }
    mIndicesBuffer = 0;

    if (glIsVertexArray(mVAOID) == GL_TRUE) {
        glDeleteVertexArrays(1, &mVAOID);
        LOGI("Release VAO [%s].", mName.c_str());
    }
    mVAOID = 0;
}
