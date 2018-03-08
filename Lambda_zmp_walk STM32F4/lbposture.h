/***********************************************/
/*   File        :lbposture.h                  */
/*   Version     :v1.0.2                       */
/*   Date        :2017/02/07                   */
/*   Author      :Kenji Shimada                */
/*   動力学フィルターに対応                    */
/***********************************************/

#ifndef __LBPOSTURE_H__ 
#define __LBPOSTURE_H__
#include <stdio.h>
#include "lambda.h"

#define LBP_WIDTH      0
#define LBP_DEPTH      1
#define LBP_HEIGHT     2
#define LBP_HUNG       3
#define LBP_YAW        4
#define LBP_ROLL       5
#define LBP_PITCH      6
#define LBP_YAW_RIGHT  7
#define LBP_YAW_LEFT   8
#define LBP_ROLL_RIGHT 9
#define LBP_ROLL_LEFT  10

#define LBP_P_QTY      11

#define LEG_X     0
#define LEG_Y     1
#define LEG_Z     2
#define LEG_ROLL  3
#define LEG_PITCH 4
#define LEG_YAW   5

#define RIGHT 0
#define LEFT  1
#define BOTH  -1

#define X 0
#define Y 1
#define Z 2


typedef struct _lbp{
  FLOAT p[LBP_P_QTY];
  FLOAT g[3];
  FLOAT o[3];
} Lbp;

void lbp_ini(Lbp *lbp);
void lbp_copy(Lbp *lbp, Lbp *lbp2);
void lbp_add(Lbp *lbp, Lbp *lbp2);  // lbp + lbp2 => lbp2
void lbp_sub(Lbp *lbp, Lbp *lbp2, Lbp *lbp3); // lbp - lbp2 => lbp3
void lbp_rot(FLOAT rt, Lbp *lbp);
void lbp_cnv_lbp_leg(Lbp *lbp, FLOAT *gp, FLOAT *r, FLOAT *l);
unsigned short lbp_cnv_l_sv(Lbp *lbp, FLOAT *gp, FLOAT *j);
void lbp_cnv_sv_l(FLOAT *j, FLOAT *gp, Lbp *lbp);
void lbp_print(Lbp *lbp);
void lbp_rotate_2d(FLOAT a, FLOAT *p);

extern FLOAT lbp_width_base;
extern Lbp lbp;

#endif
