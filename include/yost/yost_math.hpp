#pragma once
/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include<math.h>

#define QUAT_X 0
#define QUAT_Y 1
#define QUAT_Z 2
#define QUAT_W 3

/********************************************//**
* This structure represents a 3d vector.
***********************************************/
struct Vector3
{
    /********************************************//**
    * \brief Default constructor.  Initializes the vector to (0,0,0).
    ***********************************************/
    Vector3();

    /********************************************//**
    * \brief Constructor.  Initializes the vector to (x,y,z).
    ***********************************************/
    Vector3(float x, float y, float z);

    float data[3];
};

/********************************************//**
* This structure represents an orientation in the form of a quaternion.
***********************************************/
struct Orient
{
    /********************************************//**
    * \brief Default constructor.  Initializes the quaternion to (0,0,0,1).
    * The quaternion is stored in X,Y,Z,W order.
    ***********************************************/
    Orient();

    /********************************************//**
    * \brief Constructor.  Initializes the quaternion to (x,y,z,w).
    ***********************************************/
    Orient(float x, float y, float z, float w);

    float data[4];
};

/********************************************//**
* This structure represents a 3x3 matrix in row major form.
***********************************************/
struct Matrix3x3
{
    float data[9];
};

/********************************************//**
* \brief Multiply a vector by a scalar.
* \param vec The vector to multiply by the scalar.
* \param scalar The scalar to multiply the vector by.
***********************************************/
void vectorMul(Vector3* vec, float scalar);

/********************************************//**
* \brief Takes the sum of two vectors.
* \param src1 The first vector.
* \param src2 The second vector.
* \param dest The destination vector which will recieve the sum of the two.
***********************************************/
void vectorAdd(Vector3* src1, Vector3* src2, Vector3* dest);

/********************************************//**
* \brief Takes the difference of two vectors.
* \param from The vector at the tail end of the difference vector.
* \param to The vector at the head end of the difference vector. 
* \param diff The resultant difference vector.
***********************************************/
void vectorDiff(Vector3* from, Vector3* to, Vector3* diff);

/********************************************//**
* \brief Takes the cross product of two vectors.
* \param src1 The first vector.
* \param src2 The second vector.
* \param dest The destination vector which will recieve the cross product of the two.
***********************************************/
void vectorCross(Vector3* src1, Vector3* src2, Vector3* dest);

/********************************************//**
* \brief Takes the dot product of two vectors.
* \param src1 The first vector.
* \param src2 The second vector.
* \return The dot product of the vectors.
***********************************************/
float vectorDot(Vector3* src1, Vector3* src2);

/********************************************//**
* \brief Normalizes a vector.  Has no effect if the vector is of 0 magnitude.
* \param vec The vector to normalize.
***********************************************/
void vectorNormalize(Vector3* vec);

/********************************************//**
* \brief Finds the length of a vector.
* \param vec The vector to find the length of.
* \return The length of the vector.
***********************************************/
float vectorLength(Vector3* vec);

/********************************************//**
* \brief Converts the vectors coordinate space.
* \param vec The Vector3 that is to be converted.
* \param order The order of directions.
* \param negate Negate the order of the same index.
***********************************************/
void vectorConvertAxes(Vector3* vec, char* fromOrder, char* toOrder, char* negate);

/********************************************//**
* \brief Multiplies two quaternions.
* \param left The first quaternion.
* \param right The second quaternion.
* \param out The resulting quaternion.
***********************************************/
void orientMul(Orient* left, Orient* right, Orient* out);

/********************************************//**
* \brief Inverts an orientation.
* \param orient The orientation to invert.
***********************************************/
void orientInverse(Orient* orient);

/********************************************//**
* \brief Rotates a vector by the given rotation.
* \param orient The orientation to rotate the vector by.
* \param vec The vector to be rotated.  The rotated form will be stored in this vector as well.
***********************************************/
void orientRotate(Orient* orient, Vector3* vec);

/********************************************//**
* \brief Normalizes a quaternion.  Has no effect if the quaternion is of 0 magnitude.
* \param orient The quaternion to normalize.
***********************************************/
void orientNormalize(Orient* orient);

bool isRightHanded(char* order, char* negate);

/********************************************//**
* \brief Converts the orient coordinate space.
* \param vec The Orient that is to be converted.
* \param order The order of directions.
* \param negate Negate the order of the same index.
***********************************************/
void orientConvertAxes(Orient* quat, char* fromOrder, char* toOrder, char* negate);

/********************************************//**
* \brief Creates an orientation from an axis and an angle.
* \param axis The axis of rotation.
* \param angle The angle in radians.
* \return The new orientation.
***********************************************/
Orient getOrientFromAxisAngle(Vector3* axis, float angle);

/********************************************//**
* \brief Creates an orientation that would rotate vec_from to vec_to.
* \param vec_from Origin vector.
* \param vec_to Destination vector.
* \return An orientation that, when rotating vec_from, would produce vec_to.
***********************************************/
Orient getOrientFromVectors(Vector3* vec_from, Vector3* vec_to);

/********************************************//**
* \brief Converts a quaternion to a matrix.
* \param orient Input quaternion.
* \param matrix Output matrix.
***********************************************/
void convertOrientToRotationMatrix(Orient* orient, Matrix3x3* matrix);