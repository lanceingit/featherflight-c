#pragma once

typedef float Quaternion[4]; 

void quaternion_set(Quaternion q, float w, float x, float y, float z);
void quaternion_copy(Quaternion q_to, Quaternion q_from);
void quaternion_mul(Quaternion q_out, Quaternion q1, Quaternion q2);
Quaternion* quaternion_mul_cal(Quaternion q1, Quaternion q2);
void quaternion_from_yaw(Quaternion q, float yaw);
Quaternion* quaternion_normalize(Quaternion q);
void quaternion_conjugate(Vector v_out, Quaternion q, Vector v);
Vector* quaternion_conjugate_inversed_cal(Quaternion q, Vector v);