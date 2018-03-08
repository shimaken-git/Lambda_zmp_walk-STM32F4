/*************************************************/
/*   File        :lbposture.c                    */
/*   Version     :v1.0.2                         */
/*   Date        :2017/02/07                     */
/*   Author      :Kenji Shimada                  */
/*   動力学フィルターに対応                      */
/*************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include "lambda.h"
#include "lbposture.h"
#include "vector.h"

//#define LBPDEBUG

void tmcom_outval(long int val);
void tmcom_outvalf(int _order, char fill, long val);
void tmcom_outfloat(FLOAT val);
void tmcom_out2h(unsigned char val);
void tmcom_puts(char *send_data);
void tmcom_putc(char c);

#define LBP_WIDTH_BASE 91.0                      // 20170429 構造変更反映
FLOAT lbp_width_base = LBP_WIDTH_BASE;
FLOAT hwb = LBP_WIDTH_BASE * 0.5;
//FLOAT hwbr = 32.0;

Lbp lbp;

char s[100];

void lbp_ini(Lbp *lbp)
{
  lbp->p[LBP_WIDTH] = 80.0;
  lbp->p[LBP_DEPTH] = 0.0;
  lbp->p[LBP_HEIGHT] = 330.0;
  lbp->p[LBP_HUNG] = 0.0;
  lbp->p[LBP_YAW] = 0.0;
  lbp->p[LBP_ROLL] = 0.0;
  lbp->p[LBP_PITCH] = 0.0;
  lbp->p[LBP_YAW_RIGHT] = 0.0;
  lbp->p[LBP_YAW_LEFT] = 0.0;
  lbp->p[LBP_ROLL_RIGHT] = 0.0;
  lbp->p[LBP_ROLL_LEFT] = 0.0;
  lbp->o[X] = 0.0;
  lbp->o[Y] = 0.0;
  lbp->o[Z] = 0.0;
  lbp->g[X] = 0.0;
  lbp->g[Y] = 0.0;
  lbp->g[Z] = 0.0;
}

void lbp_copy(Lbp *lbp, Lbp *lbp2)
{
  int i;
  FLOAT *f1, *f2;

  f1 = lbp->p;
  f2 = lbp2->p;
  for (i = 0; i < LBP_P_QTY; i++){
    //lbp2->p[i] = lbp->p[i];
    *f2 = *f1;
    f1++;
    f2++;
  }
  for(i = 0; i < 3; i++){
    lbp2->g[i] = lbp->g[i];
    lbp2->o[i] = lbp->o[i];
  }
}

void lbp_add(Lbp *lbp, Lbp *lbp2)
{
  int i;

  for (i = 0; i < LBP_P_QTY; i++){
    lbp2->p[i] += lbp->p[i];
  }
  for(i = 0; i < 3; i++){
    lbp2->g[i] += lbp->g[i];
    lbp2->o[i] += lbp->o[i];
  }
}

void lbp_sub(Lbp *lbp, Lbp *lbp2, Lbp *lbp3)
{
  int i;

  for (i = 0; i < LBP_P_QTY; i++){
    lbp3->p[i] = lbp->p[i] - lbp2->p[i];
  }
  for(i = 0; i < 3; i++){
    lbp3->g[i] = lbp->g[i] - lbp2->g[i];
    lbp3->o[i] = lbp->o[i] - lbp2->o[i];
  }
}

void lbp_rot(FLOAT rt, Lbp *lbp)
{
  lbp_rotate_2d(-rt, lbp->p);  // width,depth を回転
  lbp_rotate_2d(-rt, lbp->o);  // offset を回転
  lbp_rotate_2d(-rt, lbp->g);  // g_offset を回転
  lbp->p[LBP_YAW] -= rt;
  lbp->p[LBP_YAW_RIGHT] -= rt;
  lbp->p[LBP_YAW_LEFT] -= rt;
}

void lbp_cnv_lbp_leg(Lbp *lbp, FLOAT *gp, FLOAT *r, FLOAT *l)
{
  FLOAT body_yaw[2];

  body_yaw[X] = gp[X];
  body_yaw[Y] = gp[Y];
  lbp_rotate_2d(lbp->p[LBP_YAW], body_yaw);

  r[LEG_X] = lbp->p[LBP_WIDTH] * 0.5 - lbp->o[X] - lbp->g[X] + body_yaw[X];
  r[LEG_Y] = lbp->p[LBP_DEPTH] * 0.5 - lbp->o[Y] - lbp->g[Y] + body_yaw[Y];
  l[LEG_X] = r[LEG_X] - lbp->p[LBP_WIDTH];
  l[LEG_Y] = r[LEG_Y] - lbp->p[LBP_DEPTH];
  lbp_rotate_2d(-lbp->p[LBP_YAW], r);
  lbp_rotate_2d(-lbp->p[LBP_YAW], l);
  r[LEG_X] -= hwb;
  l[LEG_X] += hwb;
  FLOAT x_r = hwb * cosf(lbp->p[LBP_ROLL]) - hwb;
  r[LEG_X] -= x_r;
  l[LEG_X] += x_r;

  r[LEG_Z] = - lbp->p[LBP_HEIGHT] + ((lbp->p[LBP_HUNG] + lbp->o[Z]) > 0 ? lbp->p[LBP_HUNG] + lbp->o[Z] : 0);
  l[LEG_Z] = - lbp->p[LBP_HEIGHT] - ((lbp->p[LBP_HUNG] + lbp->o[Z]) < 0 ? lbp->p[LBP_HUNG] + lbp->o[Z] : 0);
  FLOAT z_r = hwb * sinf(lbp->p[LBP_ROLL]);
  r[LEG_Z] += z_r;
  l[LEG_Z] -= z_r;
  r[LEG_ROLL] = lbp->p[LBP_ROLL];
  r[LEG_PITCH] = lbp->p[LBP_PITCH];
  r[LEG_YAW] = lbp->p[LBP_YAW_RIGHT] - lbp->p[LBP_YAW];
  l[LEG_ROLL] = lbp->p[LBP_ROLL];
  l[LEG_PITCH] = lbp->p[LBP_PITCH];
  l[LEG_YAW] = lbp->p[LBP_YAW_LEFT] - lbp->p[LBP_YAW];

//  sprintf(s, "%f %f %f %f %f %f\r\n", r[0], r[1], r[2], r[3], r[4], r[5]);
//  tmcom_puts(s);
//  sprintf(s, "%f %f %f %f %f %f\r\n", l[0], l[1], l[2], l[3], l[4], l[5]);
//  tmcom_puts(s);
}

unsigned short lbp_cnv_l_sv(Lbp *lbp, FLOAT *gp, FLOAT *j)
{
  FLOAT r[6], l[6];
  unsigned short reslt0, reslt1;

  lbp_cnv_lbp_leg(lbp, gp, r, l);
  
  reslt0 = (unsigned short)biarticular_ik(r, j, IK_RIGHT);
  reslt1 = (unsigned short)biarticular_ik(l, &j[LEG_LEFT], IK_LEFT);
  return reslt0 | reslt1 << 8;

  /*
    offset, g_offsetはbody_yawの影響を受けてはならない。
    胴体の向きではなく、ロボットの正面に対してのオフセット値とする。
    なぜならば、カーブ歩行を考えた場合、胴体を旋回しながら胴体に対してオフセットを与えねばならず、
    オフセット値が胴体の影響を受けるならば、オフセット値を胴体の姿勢にあわせて換算せねばならず、煩雑になってしまうため。
    */
  /*offset : ロボットの正面に対するオフセット値　能動的な値であるため、LBP_YAWに関わらない*/
  /*g_offset : ロボットの正面に対するオフセット値　ではあるが、ロボットの重心とロボット原点を一致させるための補正値のため受動的な値となる
    そのため、姿勢が変化するべき値。ただし、計算上LBP_YAWを考慮するのではなく、LBP_YAWが変化する場合に外部で変更しなければならない。*/
  /*gp : よく覚えていない*/
}


void lbp_cnv_sv_l(FLOAT *j, FLOAT *gp, Lbp *lbp)
{
}

void lbp_print(Lbp *lbp)
{
#ifndef GOPRO
  tmcom_puts("width depth hight hung\r\n");
  sprintf(s, "%3.1f %3.1f %3.1f %3.1f\r\n", lbp->p[LBP_WIDTH],lbp->p[LBP_DEPTH],lbp->p[LBP_HEIGHT],lbp->p[LBP_HUNG]);
  tmcom_puts(s);
  tmcom_puts("yaw   roll  pitch yaw_r yaw_l roll_r roll_l\r\n");
  sprintf(s, "%3.1f %3.1f %3.1f %3.1f %3.1f %3.1f  %3.1f\r\n", lbp->p[LBP_YAW]*180/M_PI,lbp->p[LBP_ROLL]*180/M_PI,lbp->p[LBP_PITCH]*180/M_PI,lbp->p[LBP_YAW_RIGHT]*180/M_PI,lbp->p[LBP_YAW_LEFT]*180/M_PI,lbp->p[LBP_ROLL_RIGHT]*180/M_PI,lbp->p[LBP_ROLL_LEFT]*180/M_PI);
  tmcom_puts(s);
  sprintf(s, "offset x:%f y:%f z:%f\r\n", lbp->o[X],lbp->o[Y],lbp->o[Z]);
  tmcom_puts(s);
  sprintf(s, "gp     x:%f y:%f z:%f\r\n", lbp->g[X],lbp->g[Y],lbp->g[Z]);
  tmcom_puts(s);
#endif
}

void lbp_rotate_2d(FLOAT a, FLOAT *p)
{
  FLOAT s = sinf(a);
  FLOAT c = cosf(a);
  FLOAT tx = p[X];
  FLOAT ty = p[Y];
  p[X] = tx * c - ty * s;
  p[Y] = tx * s + ty * c;
}

