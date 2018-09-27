#include "matrix.h"
#include <stdlib.h>
#include <string.h>

static float tmp_buf[30*30];
static Matrix tmp0 = {30,30,tmp_buf};

float matrix_item(Matrix m, uint8_t row, uint8_t column)
{
    return m.data[row*m.row + column];
}

void matrix_separate(Matrix* m)
{
    uint16_t len = m->row*m->column*sizeof(float);
    float* buf = malloc(len);       //FIXME:use own malloc
    if(buf != NULL) {
        memcpy(buf, m->data, len);
        m->data = buf;
    }
}

void matrix_destroy(Matrix* m)
{
    free(m->data);
    m->data = NULL;
}

float matrix_trace(Matrix m)
{
    if(!(m.row==m.column)) return 0;

    float res=0;
    for(uint8_t i = 0; i < m.row; i++) {
        res += MAT(m, i, i);
    }
    return res;    
}

Matrix matrix_mul(Matrix m1, Matrix m2)
{
    if(!(m1.row==m2.column && m1.column==m2.row)) return tmp0;

    tmp0.row = m1.row;
    tmp0.column = m2.column;

    for (uint8_t i = 0; i < m1.row; i++) {
        for (uint8_t j = 0; j < m2.column; j++) {
            for (uint8_t k = 0; k < m1.column; k++) {
                MAT(tmp0, i, k) += MAT(m1, i, j) * MAT(m2, j, k);
            }
        }
    }        
    return tmp0;    
}

Matrix matrix_scalar(Matrix m, float s)
{
    tmp0.row = m.row;
    tmp0.column = m.column;

    for (uint8_t i = 0; i < m.row; i++) {
        for (uint8_t j = 0; j < m.column; j++) {
            MAT(tmp0, i, j) = MAT(m, i, j) * s;
        }
    }
    return tmp0;    
}

Matrix matrix_add(Matrix m1, Matrix m2)
{
    if(!(m1.row==m2.row && m1.column==m2.column)) return tmp0;

    tmp0.row = m1.row;
    tmp0.column = m1.column;

    for (uint8_t i = 0; i < m1.row; i++) {
        for (uint8_t j = 0; j < m1.column; j++) {
            MAT(tmp0, i, j) = MAT(m1, i, j) + MAT(m2, i, j);
        }
    }
    return tmp0;    
}

Matrix matrix_sub(Matrix m1, Matrix m2)
{
    if(!(m1.row==m2.row && m1.column==m2.column)) return tmp0;

    tmp0.row = m1.row;
    tmp0.column = m1.column;

    for (uint8_t i = 0; i < m1.row; i++) {
        for (uint8_t j = 0; j < m1.column; j++) {
            MAT(tmp0, i, j) = MAT(m1, i, j) - MAT(m2, i, j);
        }
    }
    return tmp0; 
}

Matrix matrix_transpose(Matrix m)
{
    return tmp0; 
}

void matrix_set_row(Matrix m, uint8_t r, Vector v)
{
//    for (uint8_t i=0; i<m.column; i++) {
//        MAT(m, r, i) = row(i, 0);
//    }    
    MAT(m, r, 0) = v.x;
    MAT(m, r, 0) = v.y;
    MAT(m, r, 0) = v.z;    
}
