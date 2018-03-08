//vector.cpp
#include <math.h>
#include "lambda.h"
//#include "vector.h"

void vct_set(FLOAT a1, FLOAT a2, FLOAT a3, FLOAT *v)
{
  v[0] = a1;
  v[1] = a2;
  v[2] = a3;
}

void vct_set_E(int axis, FLOAT *v)
{
  switch(axis){
  case AXIS_X:
    vct_set(1, 0, 0, v);
    break;
  case AXIS_Y:
    vct_set(0, 1, 0, v);
    break;
  case AXIS_Z:
    vct_set(0, 0, 1, v);
    break;
  }
}

void vct_copy(FLOAT *v1, FLOAT *v2)
{
  int i;
  for(i = 0; i < 3; i++){
    *v2 = *v1;
    v1++;
    v2++;
  }
}

void vct_add(FLOAT *v1, FLOAT *v2)
{
  int i;
  for(i = 0; i < 3; i++){
    *v2 += *v1;
    v1++;
    v2++;
  }
}

void vct_sub(FLOAT *v1, FLOAT *v2)
{
  int i;
  for(i = 0; i < 3; i++){
    *v2 -= *v1;
    v1++;
    v2++;
  }
}

void vct_mul(FLOAT m, FLOAT *v)
{
  int i;
  for(i = 0; i < 3; i++){
    *v *= m;
    v++;
  }
}

void vct_minus(FLOAT *v)
{
  int i;
  for(i = 0; i < 3; i++){
    *v = -*v;
    v++;
  }
}


FLOAT vct_vct(FLOAT *v1, FLOAT *v2)
{
  FLOAT r = 0;
  int i;
  for(i = 0; i < 3; i++){
    r += *v1 * *v2;
    v1++;
    v2++;
  }
  return r;
}

void vct_rotate(FLOAT a, int axis, FLOAT *v)
{
  FLOAT rx, ry, rz;
  FLOAT s = sinf(a);
  FLOAT c = cosf(a);
  switch(axis){
  case AXIS_X:
    rx = v[0];
    ry = v[1] * c - v[2] * s;
    rz = v[1] * s + v[2] * c;
    break;
  case AXIS_Y:
    rx = v[0] * c + v[2] * s;
    ry = v[1];
    rz = - v[0] * s + v[2] * c;
    break;
  case AXIS_Z:
    rx = v[0] * c - v[1] * s;
    ry = v[0] * s + v[1] * c;
    rz = v[2];
    break;
  }
  v[0] = rx;
  v[1] = ry;
  v[2] = rz;
}

void mat_set(FLOAT a11, FLOAT a12, FLOAT a13, FLOAT a21, FLOAT a22, FLOAT a23, FLOAT a31, FLOAT a32, FLOAT a33, FLOAT v[][3])
{
  v[0][0] = a11;
  v[0][1] = a12;
  v[0][2] = a13;
  v[1][0] = a21;
  v[1][1] = a22;
  v[1][2] = a23;
  v[2][0] = a31;
  v[2][1] = a32;
  v[2][2] = a33;
}

void mat_rotate(FLOAT a, int axis, FLOAT m[][3])
{
  FLOAT c = cosf(a);
  FLOAT s = sinf(a);
  FLOAT r[3][3];
  switch(axis){
  case AXIS_X:
    mat_set( 1, 0, 0, 0, c, -s, 0, s, c, r);
    break;
  case AXIS_Y:
    mat_set( c, 0, s, 0, 1, 0, -s, 0, c, r);
    break;
  case AXIS_Z:
    mat_set( c, -s, 0, s, c, 0, 0, 0, 1, r);
    break;
  }
  mat_mat(r, m);
}

void mat_copy(FLOAT m1[][3], FLOAT m2[][3]) // m2[][] = m1[][]
{
  int i;
  for(i = 0; i < 3; i++){
    vct_copy(m1[i], m2[i]);
  }
}

void mat_wedge(FLOAT *v, FLOAT m[][3])
{
  vct_set(0, -v[2], v[1], m[0]);
  vct_set(v[2], 0, -v[0], m[1]);
  vct_set(-v[1], v[0], 0, m[2]);
}

void mat_trans(FLOAT m[][3])               // m[][] = m[][]T
{
  int i, j;
  FLOAT r[3][3];
  for(i = 0; i < 3; i++){
    for(j = 0; j < 3; j++){
      r[j][i] = m[i][j];
    }
  }
  mat_copy(r, m);
}

void mat_mat(FLOAT m1[][3], FLOAT m2[][3]) // m2[][] = m1[][] * m2[][]
{
  int i, j, k;
  FLOAT r[3][3];
  for(i = 0; i < 3; i++){
    for(j = 0; j < 3; j++){
      r[i][j] = 0;
      for(k = 0; k < 3; k++){
        r[i][j] += m1[i][k] * m2[k][j];
      }
    }
  }
  mat_copy(r, m2);
}

FLOAT *vct_mul2(FLOAT m, FLOAT *v, FLOAT *mv)  // mv[] = m * v[]
{
  FLOAT *rr;
  int i;
  rr = mv;
  for(i = 0; i < 3; i++){
    *rr = *v * m;
    v++;
    rr++;
  }
  return mv;
}

FLOAT *vct_cross(FLOAT *v1, FLOAT *v2, FLOAT *v3) //v1 * v2
{
  v3[0] = v1[1] * v2[2] - v1[2] * v2[1];
  v3[1] = v1[2] * v2[0] - v1[0] * v2[2];
  v3[2] = v1[0] * v2[1] - v1[1] * v2[0];
  return v3;
}

FLOAT *mat_vct(FLOAT m[][3], FLOAT *v, FLOAT *mv) // mv[] = m[][] * v[]
{
  int i, j;
  for(i = 0; i < 3; i++){
    mv[i] = 0;
    for(j = 0; j < 3; j++){
      mv[i] += m[i][j] * v[j];
    }
  }
  return mv;
}

void vct_print(FLOAT *v)
{
#ifdef TEST
  printf("<vct_print>%f,%f,%f\n", v[0], v[1], v[2]);
#else
//  tmcom_puts("<vct_print>"); tmcom_outFLOAT(v[0]); tmcom_putc(','); tmcom_outFLOAT(v[1]); tmcom_putc(','); tmcom_outFLOAT(v[2]); tmcom_putc('\n');
#endif
}

void mat_print(FLOAT m[][3])
{
  int i, j;
  for(i = 0; i < 3; i++){
    for(j = 0; j < 3; j++){
#ifdef TEST
      printf("%e ", m[i][j]);
    }
    printf("\n");
#else
//      tmcom_outFLOAT(m[i][j]); tmcom_putc(' ');
    }
//    tmcom_puts("\n");
#endif
  }
}
