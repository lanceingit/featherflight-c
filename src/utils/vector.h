#pragma once

typedef float Vector[3];  

void vector_zero(Vector v);
void vector_set(Vector v, float x, float y, float z);
Vector* vector_get(float x, float y, float z);
void vector_copy(Vector v_to, Vector v_from);
void vector_cross(Vector v_out, Vector v1,  Vector v2);
float vector_scalar_cal(Vector v1,  Vector v2);
void vector_add(Vector v_out, Vector v1, Vector v2);
void vector_sub(Vector v_out, Vector v1, Vector v2);
Vector* vector_mul_cal(Vector v, float s);
Vector* vector_normalized(Vector v);
float vector_length(Vector v);
Vector* vector_reverse_cal(Vector v);
