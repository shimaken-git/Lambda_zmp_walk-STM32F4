#ifndef __ZMPWALK_H__ 
#define __ZMPWALK_H__
#include <stdio.h>
#include <math.h>
#include "lambda.h"
#include "list.h"
#include "lbposture.h"

// zmp_make_walk_motion()��atrack.d[1]�Ɏc���x���r���
#define ROOT_SIGN_RIGHT 1
#define ROOT_SIGN_LEFT -1
#define ROOT_SIGN_BOTH  0

#define SHIFT_ZMP_X_RIGHT 0
#define SHIFT_ZMP_X_LEFT  1
#define SHIFT_ZMP_Y_RIGHT 2
#define SHIFT_ZMP_Y_LEFT  3

#define NOMODE  -1
#define STRATE 0
#define CURVE  1
#define TURN   2
#define SIDE   3

#define F_STEP 40
#define D_STEP 4   //rebuild���̌v�Z���ԕ�
#define CP_SPACE_THRESHOLD 0

extern FLOAT ga;
//extern FLOAT shift_zmp[];

typedef struct _motion_para{
	int id;
	int nstep;
	Node *zn;
	FLOAT d_width, s_hung, d_yaw, d_foot_yaw;
	FLOAT t_depth, s_depth;
	FLOAT s_hung_offset;
	FLOAT b_hung_offset;  // ���O�X�e�[�W��hung_offset
	FLOAT dr_hung_offset;  // = b_hung_offset / delay
	int cnt;
	int new_step;
	int fst_stage;  // �����d�S�ړ�   // _side
	int end_step; // �ŏI���t���O
	int step_stage;
	FLOAT at;  // �ݐω�]�p�x  // _turn
	FLOAT as;  // 1���ł̉�]�p�x
	// �e��p�x�ɂ��Đ������K�v�B
	// a3 : ����ZMP�ւ̃x�N�g���p�x
	// a4 : ����ZMP�����_�Ƃ������{�b�g���_�ւ̃x�N�g���p�x�@��step_stage�̔���Ɏg���Ă���悤�Ȃ̂ŕs�v���H
	// _turn �ł�a3=���{�b�g�̉�]�p�x�ɂȂ�B�ݐω�]�p�x�ɉ�������p�x�ɂȂ邪�A
	// _curve �ł�a4�ƃZ�b�g�łȂ���ΈӖ����Ȃ��Ȃ��l�ƂȂ�B�ia4�Ƃ̍��傫�����step stage�Ɣ���j
	// _curve �̏ꍇ�� a2-a1 ���ݐω�]�p�x�ɉ�������p�x�ƂȂ邪�A�ݐς����Ƃ��Aa1-a0���ݐω�]�p�x�ƂȂ�B

	// _turn : as = a3;
	// _curve _strate : as = a2 - a1;

	FLOAT wd[2];
	FLOAT fst_step_arange;
	int frame_cnt;
	int scnt;
	int firsttime;
	int _p;        //�x���r
	int _pn;       //�x���r�̋t�A�V�r
	FLOAT tz[3];

} Motion_para;

typedef struct _prediction{
	FLOAT bx, by;
	FLOAT bdx, bdy, bddx, bddy;
	FLOAT sezx, sezy;
	FLOAT bux, buy;
} Prediction;

void zmp_make_plan(int spivot, int npivot, FLOAT width, FLOAT step, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp);
void zmp_make_plan_side(int spivot, int npivot, FLOAT width, FLOAT step, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp);
void zmp_make_plan_curve(int spivot, int npivot, FLOAT width, FLOAT step, FLOAT radius, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp);
void zmp_make_plan_turn(int spivot, int npivot, FLOAT s_width, FLOAT width, FLOAT step, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp);

void fact_ini(FLOAT *k, FLOAT *fii, FLOAT height);
void prediction_ini(Prediction *pred);
void prediction_copy(Prediction *_from, Prediction *_to);
void motion_para_ini(Motion_para *mtp);
void motion_para_copy(Motion_para *_from, Motion_para *_to);
void zmp_walk_online(FLOAT dt, Node *tz, FLOAT* gt, FLOAT* zx, FLOAT* zy, Prediction *pred, int* reset, FLOAT *ini);
void dynamics_filter(FLOAT dt, Node *dz, FLOAT* gt, FLOAT* rz, Prediction *pred, int* reset);
int zmp_make_walk_motion(FLOAT *gt, Node *zmp, Lbp *lbp, int spivot, int npivot, FLOAT hung, int delay, FLOAT *gangle, Motion_para *mtp, Node** cp, int *cp_pivot);
void zmp_arange(List *zmp, List *azmp, FLOAT shift_start, FLOAT shift_end, FLOAT shift_range);
int zmp_pivot_sign(int pivot);
int zmp_pivot(int sign);
void zmp_get_robot_origin(Lbp *lbp, int pivot, FLOAT *o);
int free_leg(int pivot);
void zmp_get_robot_gloval_position(FLOAT *zmp, Lbp *lbp, int pivot, FLOAT angle, FLOAT *gp);
int nstep_side(int n, int md);
void rotate_2d_p(FLOAT a, FLOAT *p);

void up_down_action_ini(FLOAT interval, FLOAT pace);
FLOAT up_down_action(int count, FLOAT interval, FLOAT accel);
void walk_follow_action_ini(int count, int delay);
FLOAT walk_follow_action(int count, int dir, int pivot, FLOAT max);
FLOAT walk_follow_back(int count);
void walk_tilt_accumulate(int count, FLOAT threshold, FLOAT *acc, FLOAT zmp);

extern FLOAT fi[];
extern FLOAT ki[];
extern FLOAT next_step[];
extern int wfa_sw;
#endif
