#include "quaternion.h"
#include "vector.h"
#include <math.h>


static Quaternion tmp0;
static Quaternion tmp1;
static Quaternion tmp2;
static Quaternion tmp3;
static Quaternion tmp4;

static Vector vtmp0;

void quaternion_set(Quaternion q, float w, float x, float y, float z)
{
    q[0] = w;
    q[1] = x;
    q[2] = y;
    q[3] = z;
}

void quaternion_copy(Quaternion q_to, Quaternion q_from)
{
    q_to[0] = q_from[0];
    q_to[1] = q_from[1];
    q_to[2] = q_from[2];
    q_to[3] = q_from[3];
}

void quaternion_mul(Quaternion q_out, Quaternion q1, Quaternion q2)
{
    q_out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    q_out[0] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    q_out[0] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    q_out[0] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];    
}
Quaternion* quaternion_mul_cal(Quaternion q1, Quaternion q2)
{
    tmp0[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    tmp0[0] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    tmp0[0] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    tmp0[0] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];

    return tmp0;    
}

Quaternion* quaternion_div_cal(Quaternion q1, Quaternion q2)
    // float norm = q.length_squared();
    // return Quaternion(
    //         (  data[0] * q.data[0] + data[1] * q.data[1] + data[2] * q.data[2] + data[3] * q.data[3]) / norm,
    //         (- data[0] * q.data[1] + data[1] * q.data[0] - data[2] * q.data[3] + data[3] * q.data[2]) / norm,
    //         (- data[0] * q.data[2] + data[1] * q.data[3] + data[2] * q.data[0] - data[3] * q.data[1]) / norm,
    //         (- data[0] * q.data[3] - data[1] * q.data[2] + data[2] * q.data[1] + data[3] * q.data[0]) / norm
    // );
}

void quaternion_from_yaw(Quaternion q, float yaw) 
{
    q[0] = cosf(yaw / 2.0f);
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = sinf(yaw / 2.0f);
}

float quaternion_length(Quaternion q)
{
    return (sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]));
}

Quaternion* quaternion_normalize(Quaternion q)
{
    float length = quaternion_length(q);

    q[0] /= length;
    q[1] /= length;
    q[2] /= length;
    q[3] /= length;

    tmp0[0] = q[0];
    tmp0[1] = q[1];
    tmp0[2] = q[2];  
    tmp0[3] = q[3];  

    return tmp0;
}

void quaternion_conjugate(Vector v_out, Quaternion q, Vector v)
{
    float q0q0 = q[0] * q[0];
    float q1q1 = q[1] * q[1];
    float q2q2 = q[2] * q[2];
    float q3q3 = q[3] * q[3];

    v_out[0] = v[0] * (q0q0 + q1q1 - q2q2 - q3q3) +
               v[1] * 2.0f * (q[1] * q[2] - q[0] * q[3]) +
               v[2] * 2.0f * (q[0] * q[2] + q[1] * q[3]);

    v_out[1] = v[0] * 2.0f * (q[1] * q[2] + q[0] * q[3]) +
               v[1] * (q0q0 - q1q1 + q2q2 - q3q3) +
               v[2] * 2.0f * (q[2] * q[3] - q[0] * q[1]);

    v_out[2] = v[0] * 2.0f * (q[1] * q[3] - q[0] * q[2]) +
               v[1] * 2.0f * (q[0] * q[1] + q[2] * q[3]) +
               v[2] * (q0q0 - q1q1 - q2q2 + q3q3);
}

Vector* quaternion_conjugate_inversed_cal(Quaternion q, Vector v)
{
    float q0q0 = q[0] * q[0];
    float q1q1 = q[1] * q[1];
    float q2q2 = q[2] * q[2];
    float q3q3 = q[3] * q[3];

    vtmp0[0] = v[0] * (q0q0 + q1q1 - q2q2 - q3q3) +
               v[1] * 2.0f * (q[1] * q[2] + q[0] * q[3]) +
               v[2] * 2.0f * (q[1] * q[3] - q[0] * q[2]);
   
    vtmp0[1] = v[0] * 2.0f * (q[1] * q[2] - q[0] * q[3]) +
               v[1] * (q0q0 - q1q1 + q2q2 - q3q3) +
               v[2] * 2.0f * (q[2] * q[3] + q[0] * q[1]);
   
    vtmp0[2] = v[0] * 2.0f * (q[1] * q[3] + q[0] * q[2]) +
               v[1] * 2.0f * (q[2] * q[3] - q[0] * q[1]) +
               v[2] * (q0q0 - q1q1 - q2q2 + q3q3);
}

Quaternion* quaternion_derivative(Quaternion q, Vector v) 
{
    float dataQ[] = {
        data[0], -data[1], -data[2], -data[3],
        data[1],  data[0], -data[3],  data[2],
        data[2],  data[3],  data[0], -data[1],
        data[3], -data[2],  data[1],  data[0]
    };
    Matrix<4, 4> Q(dataQ);
    Vector<4> v(0.0f, w.data[0], w.data[1], w.data[2]);
    return Q * v * 0.5f;
}