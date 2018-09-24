#pragma once

typedef struct 
{
    float x;
    float y;
    float z;    
} Vector; 


Vector vector_set(float x, float y, float z);
Vector vector_cross(Vector v1,  Vector v2);
float vector_scalar(Vector v1,  Vector v2);
Vector vector_add(Vector v1,  Vector v2);
Vector vector_sub(Vector v1,  Vector v2);
Vector vector_mul(Vector v, float s);
float vector_length(Vector v);
Vector vector_normalized(Vector v);
Vector vector_reverse(Vector v);
