#pragma once

#include <stdint.h>
#include "vector.h"

#define MAT(m, r, c) (m.data[r*m.row + c])

typedef struct {
    uint8_t row;
    uint8_t column;
    float* data;
} Matrix;

float matrix_item(Matrix m, uint8_t row, uint8_t column);
float matrix_trace(Matrix m);
Matrix matrix_mul(Matrix m1, Matrix m2);
Matrix matrix_scalar(Matrix m, float s);
Matrix matrix_add(Matrix m1, Matrix m2);
Matrix matrix_sub(Matrix m1, Matrix m2);
Matrix matrix_transpose(Matrix m);
void matrix_set_row(Matrix m, uint8_t r, Vector v);
