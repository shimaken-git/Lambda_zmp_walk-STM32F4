#ifndef __VECTOR_H__ 
#define __VECTOR_H__

// ChangeLog(2011.1.15) vct_minus’Ç‰Á

#include <stdio.h>
#include <math.h>
#include "lambda.h"

// mat[0][0] mat[0][1] mat[0][2]
// mat[1][0] mat[1][1] mat[1][2]
// mat[2][0] mat[2][1] mat[2][2]

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

void vct_set(FLOAT a1, FLOAT a2, FLOAT a3, FLOAT *v);
void vct_set_E(int axis, FLOAT *v);
void vct_copy(FLOAT *v1, FLOAT *v2);
void vct_add(FLOAT *v1, FLOAT *v2);
void vct_sub(FLOAT *v1, FLOAT *v2);
void vct_mul(FLOAT m, FLOAT *v);
void vct_minus(FLOAT *v);
FLOAT vct_vct(FLOAT *v1, FLOAT *v2);
void vct_rotate(FLOAT a, int axis, FLOAT *v);

void mat_set(FLOAT a11, FLOAT a12, FLOAT a13, FLOAT a21, FLOAT a22, FLOAT a23, FLOAT a31, FLOAT a32, FLOAT a33, FLOAT v[][3]);
void mat_rotate(FLOAT a, int axis, FLOAT m[][3]);
void mat_copy(FLOAT m1[][3], FLOAT m2[][3]); // m2[][] = m1[][]
void mat_wedge(FLOAT *v, FLOAT m[][3]);
void mat_trans(FLOAT m[][3]);               // m[][] = m[][]T
void mat_mat(FLOAT m1[][3], FLOAT m2[][3]); // m2[][] = m1[][] * m2[][]

//FLOAT *vct_plus(FLOAT *v1, FLOAT *v2);
FLOAT *vct_mul2(FLOAT m, FLOAT *v, FLOAT *mv); // mv[] = m * v[]
FLOAT *vct_cross(FLOAT *v1, FLOAT *v2, FLOAT *v3); //v3[] = v1[] * v2[]
FLOAT *mat_vct(FLOAT m[][3], FLOAT *v, FLOAT *mv); // mv[] = m[][] * v[]
void vct_print(FLOAT *v);
#endif

