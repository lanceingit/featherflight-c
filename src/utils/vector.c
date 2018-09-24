#include <math.h>
#include "vector.h"

static Vector tmp0;
static Vector tmp1;
static Vector tmp2;
static Vector tmp3;
static Vector tmp4;

Vector vector_set(float x, float y, float z)
{
    tmp0.x = x;
    tmp0.y = y;
    tmp0.z = z; 

    return tmp0;
}

Vector vector_cross(Vector v1,  Vector v2)
{
    tmp0.x =  v1.y*v2.z - v1.z*v2.y;
    tmp0.y = -v1.x*v2.z + v1.z*v2.x;
    tmp0.z =  v1.x*v2.y - v1.y*v2.x;    
}

float vector_scalar(Vector v1,  Vector v2)
{
    return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}

Vector vector_add(Vector v1,  Vector v2)
{
    tmp0.x = v1.x + v2.x;
    tmp0.y = v1.y + v2.y;
    tmp0.z = v1.z + v2.z;
}

Vector vector_sub(Vector v1,  Vector v2)
{
    tmp0.x = v1.x - v2.x;
    tmp0.y = v1.y - v2.y;
    tmp0.z = v1.z - v2.z;    
}

Vector vector_mul(Vector v, float s)
{
    tmp0.x = s * v.x;
    tmp0.y = s * v.y;
    tmp0.z = s * v.z;

    return tmp0;
}

float vector_length(Vector v)
{
    return (sqrtf(v.x*v.x + v.y*v.y + v.z*v.z));
}

Vector vector_normalized(Vector v)
{
    float length = vector_length(v);

    tmp0.x = v.x / length;
    tmp0.y = v.y / length;
    tmp0.z = v.z / length;  

    return tmp0;
} 

Vector vector_reverse(Vector v)
{
    tmp0.x = -v.x;
    tmp0.y = -v.y;
    tmp0.z = -v.z;

    return tmp0;
}

