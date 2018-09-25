#include "quaternion.h"
#include "vector.h"
#include "matrix.h"
#include <math.h>


static Quaternion tmp0;

static Vector vtmp0;
static Matrix mtmp0;

Quaternion quaternion_set(float w, float x, float y, float z)
{
    tmp0.w = w;
    tmp0.x = x;
    tmp0.y = y;
    tmp0.z = z;

    return tmp0;
}

Quaternion quaternion_add(Quaternion q1, Quaternion q2)
{
    tmp0.w = q1.w + q2.w;
    tmp0.x = q1.x + q2.x;
    tmp0.y = q1.y + q2.y;
    tmp0.z = q1.z + q2.z;

    return tmp0;    
}

Quaternion quaternion_mul(Quaternion q1, Quaternion q2)
{
    tmp0.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    tmp0.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    tmp0.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    tmp0.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

    return tmp0;    
}

Quaternion quaternion_scaler(Quaternion q, float s)
{
    tmp0.w = q.w * s;
    tmp0.x = q.x * s;
    tmp0.y = q.y * s;
    tmp0.z = q.z * s;

    return tmp0;    
}

Quaternion quaternion_div(Quaternion q1, Quaternion q2)
{
    // float norm = q.length_squared();
    // return Quaternion(
    //         (  data[0] * q.data[0] + data[1] * q.data[1] + data[2] * q.data[2] + data[3] * q.data[3]) / norm,
    //         (- data[0] * q.data[1] + data[1] * q.data[0] - data[2] * q.data[3] + data[3] * q.data[2]) / norm,
    //         (- data[0] * q.data[2] + data[1] * q.data[3] + data[2] * q.data[0] - data[3] * q.data[1]) / norm,
    //         (- data[0] * q.data[3] - data[1] * q.data[2] + data[2] * q.data[1] + data[3] * q.data[0]) / norm
    // );
    
    return tmp0;
}

Quaternion quaternion_from_yaw(float yaw) 
{
    tmp0.w = cosf(yaw / 2.0f);
    tmp0.x = 0.0f;
    tmp0.y = 0.0f;
    tmp0.z = sinf(yaw / 2.0f);
    
    return tmp0;
}

Quaternion quaternion_from_matrix(Matrix m) 
{
    if(!(m.row == 4 && m.column == 1)) return tmp0;

    tmp0.w = MAT(m, 0, 0);
    tmp0.x = MAT(m, 1, 0);
    tmp0.y = MAT(m, 2, 0);
    tmp0.z = MAT(m, 3, 0);

    return tmp0;
}

Quaternion quaternion_from_dcm(Matrix m) 
{
    if(!(m.row == 3 && m.column == 3)) return tmp0;

    float t = matrix_trace(m);
    if (t > 0.0f) {
        t = sqrtf(1.0f + t);
        tmp0.w = 0.5f * t;
        t = 0.5f / t;
        tmp0.x = (MAT(m,2,1) - MAT(m,1,2)) * t;
        tmp0.y = (MAT(m,0,2) - MAT(m,2,0)) * t;
        tmp0.z = (MAT(m,1,0) - MAT(m,0,1)) * t;
    } else if (MAT(m,0,0) > MAT(m,1,1) && MAT(m,0,0) > MAT(m,2,2)) {
        t = sqrt(1.0f + MAT(m,0,0) - MAT(m,1,1) - MAT(m,2,2));
        tmp0.x = 0.5f * t;
        t = 0.5f / t;
        tmp0.w = (MAT(m,2,1) - MAT(m,1,2)) * t;
        tmp0.y = (MAT(m,1,0) + MAT(m,0,1)) * t;
        tmp0.z = (MAT(m,0,2) + MAT(m,2,0)) * t;
    } else if (MAT(m,1,1) > MAT(m,2,2)) {
        t = sqrt(1.0f - MAT(m,0,0) + MAT(m,1,1) - MAT(m,2,2));
        tmp0.y = 0.5f * t;
        t = 0.5f / t;
        tmp0.w = (MAT(m,0,2) - MAT(m,2,0)) * t;
        tmp0.x = (MAT(m,1,0) + MAT(m,0,1)) * t;
        tmp0.z = (MAT(m,2,1) + MAT(m,1,2)) * t;
    } else {
        t = sqrt(1.0f - MAT(m,0,0) - MAT(m,1,1) + MAT(m,2,2));
        tmp0.z = 0.5f * t;
        t = 0.5f / t;
        tmp0.w = (MAT(m,1,0) - MAT(m,0,1)) * t;
        tmp0.x = (MAT(m,0,2) + MAT(m,2,0)) * t;
        tmp0.y = (MAT(m,2,1) + MAT(m,1,2)) * t;
    }

    return tmp0;
}

float quaternion_length(Quaternion q)
{
    return (sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z));
}

Quaternion quaternion_normalize(Quaternion q)
{
    float length = quaternion_length(q);

    tmp0.w = q.w / length;
    tmp0.x = q.x / length;
    tmp0.y = q.y / length;  
    tmp0.z = q.z / length;  

    return tmp0;
}

Vector quaternion_conjugate(Quaternion q, Vector v)
{
    float q0q0 = q.w * q.w;
    float q1q1 = q.x * q.x;
    float q2q2 = q.y * q.y;
    float q3q3 = q.z * q.z;

    vtmp0.x = v.x * (q0q0 + q1q1 - q2q2 - q3q3) +
              v.y * 2.0f * (q.x * q.y - q.w * q.z) +
              v.z * 2.0f * (q.w * q.y + q.x * q.z);

    vtmp0.y = v.x * 2.0f * (q.x * q.y + q.w * q.z) +
              v.y * (q0q0 - q1q1 + q2q2 - q3q3) +
              v.z * 2.0f * (q.y * q.z - q.w * q.x);

    vtmp0.z = v.x * 2.0f * (q.x * q.z - q.w * q.y) +
              v.y * 2.0f * (q.w * q.x + q.y * q.z) +
              v.z * (q0q0 - q1q1 - q2q2 + q3q3);

    return vtmp0;
}

Vector quaternion_conjugate_inversed(Quaternion q, Vector v)
{
    float q0q0 = q.w * q.w;
    float q1q1 = q.x * q.x;
    float q2q2 = q.y * q.y;
    float q3q3 = q.z * q.z;

    vtmp0.x =  v.x * (q0q0 + q1q1 - q2q2 - q3q3) +
               v.y * 2.0f * (q.x * q.y + q.w * q.z) +
               v.z * 2.0f * (q.x * q.z - q.w * q.y);
   
    vtmp0.y =  v.x * 2.0f * (q.x * q.y - q.w * q.z) +
               v.y * (q0q0 - q1q1 + q2q2 - q3q3) +
               v.z * 2.0f * (q.y * q.z + q.w * q.x);
   
    vtmp0.z =  v.x * 2.0f * (q.x * q.z + q.w * q.y) +
               v.y * 2.0f * (q.y * q.z - q.w * q.x) +
               v.z * (q0q0 - q1q1 - q2q2 + q3q3);

    return vtmp0;
}

Quaternion quaternion_derivative(Quaternion q, Vector v) 
{
    float Q_data[] = {
        q.w, -q.x, -q.y, -q.z,
        q.x,  q.w, -q.z,  q.y,
        q.y,  q.z,  q.w, -q.x,
        q.z, -q.y,  q.x,  q.w
    };
    Matrix Q = {4, 4, Q_data};

    float V_data[] = {0.0f, v.x, v.y, v.z};
    Matrix V = {4, 1, V_data};

    mtmp0 = matrix_scalar(matrix_mul(Q, V), 0.5f);

    tmp0 = quaternion_from_matrix(mtmp0);

    return tmp0;
}
