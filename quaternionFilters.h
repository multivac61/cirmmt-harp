#ifndef QUATERNIONFILTERS_H
#define QUATERNIONFILTERS_H

#include "RTMath.h"

void MadgwickQuaternionUpdate(RTQuaternion& q, const RTVector3& accData, const RTVector3& gyrData, const RTVector3& magData, float beta, float zeta, float delta_t);
void MahonyQuaternionUpdate(float *q, const RTVector3& accData, const RTVector3& gyrData, const RTVector3& magData, float delta_t);

#endif  //  QUATERNIONFILTERS_H