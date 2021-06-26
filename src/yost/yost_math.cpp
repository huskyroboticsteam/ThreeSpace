/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation
 * This Source Code Form is subject to the terms of the YOST 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/


#define _USE_MATH_DEFINES 

#include <stdio.h>
#include <math.h>
#include "yost_math.hpp"

Vector3::Vector3()
{
    data[0] = 0.0f;
    data[1] = 0.0f;
    data[2] = 0.0f;
}

Vector3::Vector3(float x, float y, float z)
{
    data[0] = x;
    data[1] = y;
    data[2] = z;
}

Orient::Orient()
{
    data[QUAT_X] = 0.0f;
    data[QUAT_Y] = 0.0f;
    data[QUAT_Z] = 0.0f;
    data[QUAT_W] = 1.0f;
}

Orient::Orient(float x, float y, float z, float w)
{
    data[QUAT_X] = x;
    data[QUAT_Y] = y;
    data[QUAT_Z] = z;
    data[QUAT_W] = w;
}

void vectorMul(Vector3* vec, float scalar)
{
    vec->data[0] *= scalar;
    vec->data[1] *= scalar;
    vec->data[2] *= scalar;
}

void vectorAdd(Vector3* src1, Vector3* src2, Vector3* dest)
{
    dest->data[0] = src1->data[0] + src2->data[0];
    dest->data[1] = src1->data[1] + src2->data[1];
    dest->data[2] = src1->data[2] + src2->data[2];
}

void vectorDiff(Vector3* from, Vector3* to, Vector3* diff)
{
    diff->data[0] = to->data[0] - from->data[0];
    diff->data[1] = to->data[1] - from->data[1];
    diff->data[2] = to->data[2] - from->data[2];
}

void vectorCross(Vector3* src1, Vector3* src2, Vector3* dest)
{
    dest->data[0] = src1->data[1] * src2->data[2] - src1->data[2] * src2->data[1];
    dest->data[1] = src1->data[2] * src2->data[0] - src1->data[0] * src2->data[2];
    dest->data[2] = src1->data[0] * src2->data[1] - src1->data[1] * src2->data[0];
}

float vectorDot(Vector3* src1, Vector3* src2)
{
    return src1->data[0] * src2->data[0] + src1->data[1] * src2->data[1] + src1->data[2] * src2->data[2];
}

void vectorNormalize(Vector3* vec)
{
    float mag2 = vec->data[0] * vec->data[0] + vec->data[1] * vec->data[1] + vec->data[2] * vec->data[2];
    if (mag2 == 0.0f)
        return;

    float mag = sqrtf(mag2);

    vec->data[0] /= mag;
    vec->data[1] /= mag;
    vec->data[2] /= mag;
}

float vectorLength(Vector3* vec)
{
    float mag2 = vec->data[0] * vec->data[0] + vec->data[1] * vec->data[1] + vec->data[2] * vec->data[2];
    if (mag2 == 0.0f)
        return 0.0f;

    return sqrtf(mag2);
}

void vectorConvertAxes(Vector3* vec, char* fromOrder, char* toOrder, char* negate)
{
    Vector3 tempVec = Vector3();
    tempVec.data[fromOrder[0]] = vec->data[toOrder[0]];
    tempVec.data[fromOrder[1]] = vec->data[toOrder[1]];
    tempVec.data[fromOrder[2]] = vec->data[toOrder[2]];

    for (int i = 0; i < 3; i++)
    {
        tempVec.data[i] *= negate[i];
    }

    *vec = tempVec;
}

void orientMul(Orient* left, Orient* right, Orient* out)
{
    out->data[QUAT_W] =
        left->data[QUAT_W] * right->data[QUAT_W] -
        left->data[QUAT_X] * right->data[QUAT_X] -
        left->data[QUAT_Y] * right->data[QUAT_Y] -
        left->data[QUAT_Z] * right->data[QUAT_Z];

    out->data[QUAT_X] =
        left->data[QUAT_W] * right->data[QUAT_X] +
        left->data[QUAT_X] * right->data[QUAT_W] +
        left->data[QUAT_Y] * right->data[QUAT_Z] -
        left->data[QUAT_Z] * right->data[QUAT_Y];

    out->data[QUAT_Y] =
        left->data[QUAT_W] * right->data[QUAT_Y] +
        left->data[QUAT_Y] * right->data[QUAT_W] +
        left->data[QUAT_Z] * right->data[QUAT_X] -
        left->data[QUAT_X] * right->data[QUAT_Z];

    out->data[QUAT_Z] =
        left->data[QUAT_W] * right->data[QUAT_Z] +
        left->data[QUAT_Z] * right->data[QUAT_W] +
        left->data[QUAT_X] * right->data[QUAT_Y] -
        left->data[QUAT_Y] * right->data[QUAT_X];
}

void orientInverse(Orient* orient)
{
    orient->data[QUAT_X] = -orient->data[QUAT_X];
    orient->data[QUAT_Y] = -orient->data[QUAT_Y];
    orient->data[QUAT_Z] = -orient->data[QUAT_Z];
}

void orientRotate(Orient* orient, Vector3* vec)
{
    Orient mrot;
    mrot.data[QUAT_X] = vec->data[0];
    mrot.data[QUAT_Y] = vec->data[1];
    mrot.data[QUAT_Z] = vec->data[2];
    mrot.data[QUAT_W] = 0.0f;

    Orient cquat;
    cquat.data[QUAT_X] = -orient->data[QUAT_X];
    cquat.data[QUAT_Y] = -orient->data[QUAT_Y];
    cquat.data[QUAT_Z] = -orient->data[QUAT_Z];
    cquat.data[QUAT_W] = orient->data[QUAT_W];

    Orient res;

    orientMul(orient, &mrot, &res);
    orientMul(&res, &cquat, &mrot);

    vec->data[0] = mrot.data[QUAT_X];
    vec->data[1] = mrot.data[QUAT_Y];
    vec->data[2] = mrot.data[QUAT_Z];
}

void orientNormalize(Orient* orient)
{
    float mag = orient->data[0] * orient->data[0];
    mag += orient->data[1] * orient->data[1];
    mag += orient->data[2] * orient->data[2];
    mag += orient->data[3] * orient->data[3];
    mag = sqrtf(mag);

    if (mag > 0)
    {
        orient->data[0] /= mag;
        orient->data[1] /= mag;
        orient->data[2] /= mag;
        orient->data[3] /= mag;
    }
}

bool isRightHanded(char* order, char* negate)
{
    Vector3 axis0, axis1, axis2;

    axis0.data[order[0]] = 1;
    axis1.data[order[1]] = 1;
    axis2.data[order[2]] = 1;

    for (int i = 0; i < 3; i++)
    {
        axis0.data[i] *= negate[i];
        axis1.data[i] *= negate[i];
        axis2.data[i] *= negate[i];
    }

    Vector3 crossAxis, tempVect;
    vectorCross(&axis0, &axis1, &crossAxis);
    vectorNormalize(&crossAxis);
    vectorDiff(&crossAxis, &axis2, &tempVect);
    bool reg = !(vectorLength(&tempVect) < 0.0001f);

    return reg;
}

void orientConvertAxes(Orient* quat, char* fromOrder, char* toOrder, char* negate)
{
    Vector3 tempVect = Vector3(quat->data[0], quat->data[1], quat->data[2]);
    vectorConvertAxes(&tempVect, fromOrder, toOrder, negate);

    if (isRightHanded(toOrder, negate))
    {
        vectorMul(&tempVect, -1);
    }

    *quat = Orient(tempVect.data[0], tempVect.data[1], tempVect.data[2],quat->data[3]);
}

Orient getOrientFromAxisAngle(Vector3* axis, float angle)
{
    Vector3 n_axis = *axis;
    vectorNormalize(&n_axis);

    float ang_cos = cosf(angle / 2.0f);
    float ang_sin = sinf(angle / 2.0f);

    Orient out;
    out.data[QUAT_X] = n_axis.data[0] * ang_sin;
    out.data[QUAT_Y] = n_axis.data[1] * ang_sin;
    out.data[QUAT_Z] = n_axis.data[2] * ang_sin;
    out.data[QUAT_W] = ang_cos;

    return out;
}

Orient getOrientFromVectors(Vector3* vec_from, Vector3* vec_to)
{
    Vector3 n_vec_from = *vec_from;
    Vector3 n_vec_to = *vec_to;

    vectorNormalize(&n_vec_from);
    vectorNormalize(&n_vec_to);

    Vector3 vec_cross;
    vectorCross(&n_vec_from, &n_vec_to, &vec_cross);
    float dot_val = vectorDot(&n_vec_from, &n_vec_to);


    if (dot_val <= -0.9999)
    {
        dot_val = -1.0f;
        Vector3 temp_vect;

        Vector3 tmp = Vector3(1, 0, 0);
        vectorCross(&tmp, &n_vec_from, &temp_vect);
        if (vectorLength(&temp_vect) < 0.00001)
        {
            tmp = Vector3(0, 0, 1);
            vectorCross(&tmp, &n_vec_from, &temp_vect);
        }
        vectorNormalize(&temp_vect);
        Orient orient = getOrientFromAxisAngle(&temp_vect, (float)M_PI);
        return orient;
    }
    else if (dot_val >= 0.9999)
    {
        return Orient(0.0, 0.0, 0.0, 1.0f);
    }

    float orient_angle = acosf(dot_val);

    Orient orient = getOrientFromAxisAngle(&vec_cross, orient_angle);
    orientNormalize(&orient);
    return orient;
}

void convertOrientToRotationMatrix(Orient* orient, Matrix3x3* matrix)
{
    float fTx = 2.0f*orient->data[QUAT_X];
    float fTy = 2.0f*orient->data[QUAT_Y];
    float fTz = 2.0f*orient->data[QUAT_Z];
    float fTwx = fTx*orient->data[QUAT_W];
    float fTwy = fTy*orient->data[QUAT_W];
    float fTwz = fTz*orient->data[QUAT_W];
    float fTxx = fTx*orient->data[QUAT_X];
    float fTxy = fTy*orient->data[QUAT_X];
    float fTxz = fTz*orient->data[QUAT_X];
    float fTyy = fTy*orient->data[QUAT_Y];
    float fTyz = fTz*orient->data[QUAT_Y];
    float fTzz = fTz*orient->data[QUAT_Z];

    matrix->data[0] = 1.0f - (fTyy + fTzz);
    matrix->data[1] = fTxy - fTwz;
    matrix->data[2] = fTxz + fTwy;
    matrix->data[3] = fTxy + fTwz;
    matrix->data[4] = 1.0f - (fTxx + fTzz);
    matrix->data[5] = fTyz - fTwx;
    matrix->data[6] = fTxz - fTwy;
    matrix->data[7] = fTyz + fTwx;
    matrix->data[8] = 1.0f - (fTxx + fTyy);
}