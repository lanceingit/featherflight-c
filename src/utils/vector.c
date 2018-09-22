#include <math.h>
#include "vector.h"

static Vector tmp0;
static Vector tmp1;
static Vector tmp2;
static Vector tmp3;
static Vector tmp4;

void vector_zero(Vector v)
{
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;    
}

void vector_set(Vector v, float x, float y, float z)
{
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

Vector* vector_get(float x, float y, float z)
{
    tmp0[0] = x;
    tmp0[1] = y;
    tmp0[2] = z;

    return tmp0;
}

void vector_copy(Vector v_to, Vector v_from)
{
    v_to[0] = v_from[0];
    v_to[1] = v_from[1];
    v_to[2] = v_from[2];
}

void vector_cross(Vector v_out, Vector v1,  Vector v2)
{
    v_out[0] =  v1[1]*v2[2] - v1[2]*v2[1];
    v_out[1] = -v1[0]*v2[2] + v1[2]*v2[0];
    v_out[2] =  v1[0]*v2[1] - v1[1]*v2[0];    
}

float vector_scalar_cal(Vector v1,  Vector v2)
{
    return (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]);
}

void vector_add(Vector v_out, Vector v1, Vector v2)
{
    v_out[0] = v1[0] + v2[0];
    v_out[1] = v1[1] + v2[1];
    v_out[2] = v1[2] + v2[2];
}

void vector_sub(Vector v_out, Vector v1, Vector v2)
{
    v_out[0] = v1[0] - v2[0];
    v_out[1] = v1[1] - v2[1];
    v_out[2] = v1[2] - v2[2];    
}

Vector* vector_mul(Vector v, float s)
{
    tmp0[0] = s * v[0];
    tmp0[1] = s * v[1];
    tmp0[2] = s * v[2];

    return tmp0;
}

float vector_length(Vector v)
{
    return (sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
}

Vector* vector_normalized(Vector v)
{
    float length = vector_length(v);

    v[0] /= length;
    v[1] /= length;
    v[2] /= length;

    tmp0[0] = v[0];
    tmp0[1] = v[1];
    tmp0[2] = v[2];  

    return tmp0;
} 

