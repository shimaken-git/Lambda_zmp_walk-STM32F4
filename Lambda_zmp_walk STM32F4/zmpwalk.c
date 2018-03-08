//zmpwalk.c

// ChangeLog(2011.1.15) zmp_make_plan()�ύX�@zmp_make_walk_motion()�ύX
// ChangeLog(2011.1.16) zmp_make_plan_curve()�ύX�@zmp_make_plan_turn()�ύX
// ChangeLog(2011.2.21) zmp_make_plan_turn()�ύX

#define _USE_MATH_DEFINES
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lambda.h"
#include "vector.h"
#include "list.h"
#include "zmpwalk.h"
#include "lbposture.h"
#include "multi_mass_point_model.h"

//#define TEST


static int depth_func_mode = 0;
#define DEPTH_FUNC1 mtp->s_depth + mtp->t_depth * (1 - cosf(((mtp->scnt - delay - mtp->cnt + 1.0) / (mtp->scnt - delay)) * M_PI)) * 0.5
#define DEPTH_FUNC2 mtp->s_depth + mtp->t_depth * sinf(((mtp->scnt - delay - mtp->cnt + 1.0) / (mtp->scnt - delay)) * M_PI * 0.5)

static int hung_func_mode = 0;
#define HUNG_FUNC1 mtp->s_hung * (1 - cosf(((mtp->scnt - delay - mtp->cnt + 1.0) / (mtp->scnt - delay)) * 2 * M_PI)) * 0.5
#define HUNG_FUNC2 mtp->s_hung * sinf(((mtp->scnt - delay - mtp->cnt + 1.0) / (mtp->scnt - delay)) * M_PI * 0.5)

static FLOAT hung_offset = 0;

//mode
#define _strate 0
#define _curve  1
#define _turn   2
#define _side   3

FLOAT ga = 9800.0;

FLOAT next_step[3];  //�ŏI�����ŏI����Ȃ������ꍇ�̎��̖ڕWZMP���W
int next_step_flag;

/////////////////////////////////////////
////    ZMP �����p�p�����[�^ 2010.12.21
//FLOAT param.f[4] = { 0, 0, 0, 0 };
////    �������S����ZMP�����炷�p�����[�^
/////////////////////////////////////////


void zmp_make_plan(int spivot, int npivot, FLOAT width, FLOAT step, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp)
{
  // spivot : �����̎����B�����̏ꍇ�� -1
  // npivot : �����̎����������̏ꍇ��1���ڂ̎���(��������o���ꍇ��npivot=RIGHT)
  // ChangeLog(2011.1.15) param.f[_shift_zmp_**]���T�|�[�g  //20171020 shift_zmp����
  int nstep;
  Node *z;
  FLOAT sft;
  int f_cnt = 0;
  int step_frm = (int)(pace / dt);
  int st_frm;
  int p;

  next_step_flag = 0;
  list_clear(zmp);
  
  if(spivot < 0){  // �Î~����X�^�[�g
    st_frm = step_frm * 14 / 10;
    sft = 0;
	p = spivot;
  }else{
    st_frm = 0;
    npivot = spivot;
    sft = width * (spivot % 2 ? -0.5 : 0.5);
  }
  int ed_frm = step_frm * 14 / 10;

  while(f_cnt < st_frm + (n + 1) * step_frm + ed_frm){  //������ n �̎��AZMP�_�� n+1 �ƂȂ�B�@�E�|�E�|�E�|�E�@�|�������A�E��ZMP�_
    z = list_push_back(zmp);
	z->di[0] = _strate;
    if(st_frm && f_cnt < st_frm){
      vct_set(offset[X] + ini[X], offset[Y] + ini[Y], offset[Z], z->d);
	  z->di[1] = p;
    }else if(f_cnt < st_frm + (n + 1) * step_frm){
      nstep = (f_cnt - st_frm) / step_frm;
      p = (npivot + nstep) % 2;
      vct_set(width * zmp_pivot_sign(npivot + nstep) * 0.5 + param.f[p] - sft, nstep * step + param.f[p + 2], offset[Z], z->d);
	  z->di[1] = p;
	  rotate_2d_p(offset[Z], z->d);
	  z->d[X] += offset[X];
	  z->d[Y] += offset[Y];
	}
	else if (f_cnt < st_frm + (n + 1) * step_frm + ed_frm){
      vct_set(0 - sft, n * step, offset[Z], z->d);
	  z->di[1] = -1;
	  rotate_2d_p(offset[Z], z->d);
	  z->d[X] += offset[X];
	  z->d[Y] += offset[Y];
	  if(!next_step_flag){
	      nstep = (f_cnt - st_frm) / step_frm;
	      p = (npivot + nstep) % 2;
//	      vct_set(width * zmp_pivot_sign(npivot + nstep) * 0.5 + param.f[p] - sft, nstep * step + param.f[p + 2], offset[Z], next_step);
	      vct_set(width * zmp_pivot_sign(npivot + nstep) * 0.5 + param.f[p] - sft, n * step + param.f[p + 2], offset[Z], next_step);
		  rotate_2d_p(offset[Z], next_step);
		  next_step[X] += offset[X];
		  next_step[Y] += offset[Y];
		  next_step_flag = 1;
	  }
	}
    f_cnt++;
  }
}

void zmp_make_plan_side(int spivot, int npivot, FLOAT width, FLOAT step, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp)
{
	//FLOAT *ini : �����p���덷���z�����邽�߁A�ڕWZMP���덷�Ɠ��l�ɂ��邽�߂̓��͕ϐ� spivot = -1�̎��Ɏg�p�����
  int nstep;
  Node *z;
  FLOAT sft;
  int f_cnt = 0;
  int step_frm = (int)(pace / dt);
  int st_frm;
  int md;

  next_step_flag = 0;
  list_clear(zmp);

  // ����F�ړ�������step�̕����Ō��܂�Bstep > 0 �ŉE�ɁAstep < 0 �ō��Ɉړ��B
  //       �Î~����X�^�[�g�̏ꍇ�Anpivot���ŏ��̎����ƂȂ�B
  //       npivot=RIGHT,step>0�̏ꍇ�Awidth�����������ʁ�����������
  //       npivot=RIGHT,step<0�̏ꍇ�Awidth���g���遨���ʁ��g���遨���ʁ@�ƕ������ƂɂȂ�B
  //       n�͕����Bn=0�̏ꍇ�Awidth��step�������ω����ďI���Bwidth��ω������邽�߂Ɏg�p���邱�Ƃ�z��B
  //         �������A���̏ꍇ�͌p�����s���R�ƂȂ�Ǝv����B
  //
  //       �X�^�[�g����npivot��step�̕����œ��삪�ς�邽�߁A�p���̏ꍇ�ɂ�npivot���Q�Ƃ���B
  //       �p�����[�h�̎�(spivot>=0�̎��j�A
  //         �o���Ɠ������������ŊJ�n����ꍇ��(npivot==spivot) width����
  //         �o���ƈႤ���������ŊJ�n����ꍇ��(npivot!=spivot) width�����A�܂��͊g����Ŏn�܂�B
  //       �����́Anstep_size()�ɓn�������Ő��䂵�Ă���B
  
  if(spivot < 0){  // �Î~����X�^�[�g
    st_frm = step_frm * 14 / 10;
    sft = 0;
    md = 0;
  }else{
    st_frm = 0;
    md = npivot ^ (spivot % 2);
    npivot = spivot;
    sft = width * (spivot % 2 ? -0.5 : 0.5);
  }
  int ed_frm = step_frm * 14 / 10;

  while(f_cnt < st_frm + (n + 1) * step_frm + ed_frm){
    z = list_push_back(zmp);
	z->di[0] = _side;
	if (st_frm && f_cnt < st_frm){
      vct_set(offset[X] + ini[X], offset[Y] + ini[Y], 0, z->d);
	  z->di[1] = md;
    }else if(f_cnt < st_frm + (n + 1) * step_frm){
      nstep = (f_cnt - st_frm) / step_frm;
      vct_set(offset[X] + width * zmp_pivot_sign(npivot + nstep) * 0.5 - sft + nstep_side(nstep, md) * step, offset[Y], 0, z->d);
	  z->di[1] = md;
	}
	else if (f_cnt < st_frm + (n + 1) * step_frm + ed_frm){
      vct_set(offset[X] + 0 - sft + nstep_side(n, md) * step, offset[Y], 0, z->d);
	  z->di[1] = -1;
	  if(!next_step_flag){
	      nstep = (f_cnt - st_frm) / step_frm;
	      vct_set(offset[X] + width * zmp_pivot_sign(npivot + nstep) * 0.5 - sft + nstep_side(nstep, md) * step, offset[Y], 0, next_step);
		  next_step_flag = 1;
	  }
	}
    f_cnt++;
  }
}

void zmp_make_plan_curve(int spivot, int npivot, FLOAT width, FLOAT step, FLOAT radius, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp)
{
  // spivot : 0:right 1:left -1:both ���p�����̎���
  // npivot : ���̎����@spivot��-1�̎������Q�Ƃ���B
  // offset : ���{�b�g�̃O���[�o�����W offset[Z]�̓O���[�o���p�x
  // ChangeLog(2011.1.15) param.f[_shift_zmp_**]���T�|�[�g  //20171020 shift_zmp����

  int nstep;
  Node *z;
  FLOAT rf, rc, ro, r, ra;

  int f_cnt = 0;
  int step_frm = (int)(pace / dt);
  int st_frm;
  int p;

  next_step_flag = 0;
  list_clear(zmp);

  if(radius != 0){
    //1���ł̐���p�x�����߂�
    rf = radius + (radius > 0 ? width : -width);  //pivot�Ɋ֌W�Ȃ�rf = radius + width
    rc = (radius + rf) * 0.5;
    if(spivot < 0)
      ro = rc;
    else if(radius * zmp_pivot_sign(spivot) < 0)
      ro = radius;
    else
      ro = rf;
    ra = step / rf;
    //printf("ro = %f ra = %f\n", ro, ra);
    //��]���S�́A(ro, 0)
  }
  if(spivot < 0){  // �Î~����X�^�[�g
    st_frm = step_frm * 14 / 10;
  }else{
    st_frm = 0;
    npivot = spivot;
  }
  int ed_frm = step_frm * 14 / 10;
  while(f_cnt < st_frm + (n + 1) * step_frm + ed_frm){
    z = list_push_back(zmp);
	z->di[0] = _curve;
	if (st_frm && f_cnt < st_frm){
      vct_set(offset[X] + ini[X], offset[Y] + ini[Y], offset[Z], z->d);
	  z->di[1] = -1;
	}
	else if (f_cnt < st_frm + (n + 1) * step_frm){
      nstep = (f_cnt - st_frm) / step_frm;
      p = (npivot + nstep) % 2;
      if(radius * zmp_pivot_sign(npivot + nstep) < 0){ // -<
        r = radius;
      }else{ // ->
        r = rf;
      }
      vct_set(r + param.f[p], param.f[p + 2], offset[Z] + ra * nstep, z->d);
	  z->di[1] = p;
	  rotate_2d_p(ra * nstep, z->d);
      z->d[X] -= ro;
	  rotate_2d_p(offset[Z], z->d);
	  z->d[X] += offset[X];
	  z->d[Y] += offset[Y];
	  //vct_set(r * cosf(ra * nstep) - ro, r * sinf(ra * nstep), 0, z->d);
    }else if(f_cnt < st_frm + (n + 1) * step_frm + ed_frm){
      vct_set(rc * cosf(ra * n) - ro, rc * sinf(ra * n), offset[Z] + ra * n, z->d);
	  z->di[1] = -1;
	  rotate_2d_p(offset[Z], z->d);
	  z->d[X] += offset[X];
	  z->d[Y] += offset[Y];
	  if(!next_step_flag){
	      nstep = (f_cnt - st_frm) / step_frm;
	      p = (npivot + nstep) % 2;
	      if(radius * zmp_pivot_sign(npivot + nstep) < 0){ // -<
	        r = radius;
	      }else{ // ->
	        r = rf;
	      }
//	      vct_set(r + param.f[p], param.f[p + 2], offset[Z] + ra * nstep, next_step);
	      vct_set(r + param.f[p], param.f[p + 2], offset[Z] + ra * n, next_step);
//		  rotate_2d_p(ra * nstep, next_step);
		  rotate_2d_p(ra * n, next_step);
	      next_step[X] -= ro;
		  rotate_2d_p(offset[Z], next_step);
		  next_step[X] += offset[X];
		  next_step[Y] += offset[Y];
		  next_step_flag = 1;
	  }
	}
    f_cnt++;
  }
}

void zmp_make_plan_turn(int spivot, int npivot, FLOAT s_width, FLOAT width, FLOAT step, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp){
  // spivot : 0:right 1:left -1:both ���p�����̎���
  // npivot : �����x���r��-1�̎��̎x���r����Ɏw��B�i���̊֐��ł͌p�����s�̏ꍇ�ɂ��Q�Ƃ���̂Œ��Ӂj
  // s_width : ���s�J�n���̋r���B�����x���r��-1�̎��ɗL���B
  // ChangeLog(2011.1.15) param.f[_shift_zmp_**]���T�|�[�g  //20171020 shift_zmp����
  // ChangeLog(2011.2.21) s_width���T�|�[�g

  //FLOAT o[2];  // ��]���S
  int nstep;
  Node *z;
  FLOAT ra;

  int f_cnt = 0;  // �t���[���J�E���^�[
  int step_frm = (int)(pace / dt);  // 1���̃t���[����
  int st_frm;  // �����d�S�ړ��X�e�[�W�̃t���[�����@�p�����s�̏ꍇ��0������
  int final_step = 0;  // �t�@�C�i���X�e�[�W�t���O
  int bstep = 0;  // �O���nstep
  int p; //����
  int f; //�V�r
  FLOAT bfr_step[2];  // �O��܂ł̕��ł�ZMP���W
  FLOAT one_step[2];  // �����1���ł�ZMP�ړ���
  FLOAT _bfr_step[2];

  next_step_flag = 0;
  list_clear(zmp);

  //����F�O��̈ʒu�ɁA�����1���ł̍�����������`�ŖڕWZMP�𐶐����Ă���B
  //      bfr_step[]�ɑO��̈ʒu���i�[����Ă���Aone_step[]�ɍ����1���ł̈ړ������i�[�����B
  //      bfr_step[]�̍X�V�̓t���[�����V�������ɂȂ������ɍs����inew_step == 1�̎��j
  //      �ŐV��ZMP���W��bfr_step[] + rotate(nstep * ra, one_step[]) �œ�����Bra:1���ł̐���p�x
  
  //1���ł̐���p�x�����߂�
  ra = step / width * zmp_pivot_sign(npivot + 1);
  //����F�^�[���̏ꍇ�A�ǂ����]�̕��s�ł��邩�́A�ŏ��̎����ƃX�e�b�v�̕����Ō��܂�B
  //�@�@�@�Ȃ̂ŁA�p�����s�̏ꍇ��npivot���L���ł���A�w�肳�ꂽnpivot�ɂĐ���p�x�̕��������߂�K�v������B

  if(spivot < 0){  // �Î~����X�^�[�g
    st_frm = step_frm * 14 / 10;
  }else{
    st_frm = 0;
    npivot = spivot;
  }
  //�ŏ��́u�X�e�b�v�X�e�[�W�v�̂��߂�bfr_step[]�ݒ�
//  bfr_step[X] = width * (spivot < 0 ? 0.5 : 1) * zmp_pivot_sign(npivot + 1);
  bfr_step[X] = s_width * (spivot < 0 ? 0.5 : 1) * zmp_pivot_sign(npivot + 1); // add s_width
  bfr_step[Y] = 0;
  //����F���O�̒��O�̎��r���W�����Ă����B�Î~����̃X�^�[�g�̏ꍇ�A�ŏ��̎����̔��΂̋r������
  //      �������Ƃ������W�����Ă����B�����ɂ�param.f[_shift_zmp_**]�͉����Ȃ��B

  int ed_frm = step_frm * 14 / 10;
  while(f_cnt < st_frm + (n + 1) * step_frm + ed_frm){
    z = list_push_back(zmp);
	z->di[0] = _turn;
	if (st_frm && f_cnt < st_frm){  // �u�Î~����X�^�[�g�v�ŁA�����X�e�[�W
      vct_set(offset[X] + ini[X], offset[Y] + ini[Y], offset[Z], z->d);
	  z->di[1] = -1;
	}
	else if (f_cnt < st_frm + (n + 1) * step_frm){  // �X�e�b�v�X�e�[�W
      nstep = (f_cnt - st_frm) / step_frm;
      p = (npivot + nstep) % 2;
      f = (p + 1) % 2;
      if(bstep < nstep){ // �V���ɓ������̂�bfr_step[]�X�V(bfr_step[]�ɂ�param.f[_shfit_zmp_**]���܂܂Ȃ�)
        one_step[X] = (bstep ? width : s_width) * zmp_pivot_sign(npivot + bstep);
        one_step[Y] = 0;
        rotate_2d_p(bstep * ra, one_step);
        bfr_step[X] += one_step[X];
        bfr_step[Y] += one_step[Y];
        bstep = nstep;
      }
      one_step[X] = (nstep ? width : s_width) * zmp_pivot_sign(npivot + nstep) + param.f[p];
      one_step[Y] = param.f[p + 2];
      rotate_2d_p(nstep * ra, one_step);
      vct_set(bfr_step[X] + one_step[X], bfr_step[Y] + one_step[Y], offset[Z] + ra * nstep, z->d);
	  z->di[1] = p;
	  rotate_2d_p(offset[Z], z->d);
	  z->d[X] += offset[X];
	  z->d[Y] += offset[Y];
	}
	else if (f_cnt < st_frm + (n + 1) * step_frm + ed_frm){  // �ŏI�X�e�[�W
	  if(!next_step_flag){
		  nstep = (f_cnt - st_frm) / step_frm;
		  p = (npivot + nstep) % 2;
		  f = (p + 1) % 2;
		  if(bstep < nstep){
			one_step[X] = (bstep ? width : s_width) * zmp_pivot_sign(npivot + bstep);
			one_step[Y] = 0;
			rotate_2d_p(bstep * ra, one_step);
			_bfr_step[X] = bfr_step[X] + one_step[X];
			_bfr_step[Y] = bfr_step[Y] + one_step[Y];
			bstep = nstep;
		  }
		  one_step[X] = (nstep ? width : s_width) * zmp_pivot_sign(npivot + nstep) + param.f[p];
		  one_step[Y] = param.f[p + 2];
//		  rotate_2d_p(nstep * ra, one_step);
		  rotate_2d_p(n * ra, one_step);
//		  vct_set(_bfr_step[X] + one_step[X], _bfr_step[Y] + one_step[Y], offset[Z] + ra * n, next_step);
		  vct_set(_bfr_step[X] + one_step[X], _bfr_step[Y] + one_step[Y], offset[Z] + ra * nstep, next_step);
		  rotate_2d_p(offset[Z], next_step);
		  next_step[X] += offset[X];
		  next_step[Y] += offset[Y];
		  next_step_flag = 1;
	  }
      if(!final_step){  // ���̃u���b�N�͂��̃X�e�[�W�ɓ������ŏ��̃t���[���������s
        one_step[X] = width * zmp_pivot_sign(npivot + n);
        one_step[Y] = 0;
        rotate_2d_p(n * ra, one_step);
        bfr_step[X] += one_step[X];
        bfr_step[Y] += one_step[Y];
        final_step = 1;  // ���̃u���b�N�����Ɏ��s�������Ƃ��������߂�1�Ƃ���B
      }
      one_step[X] = s_width * 0.5 * zmp_pivot_sign(npivot + n + 1);
      one_step[Y] = 0;
      rotate_2d_p(n * ra, one_step);  // �ŏI���ł̓^�[�����Ȃ����߁A(n+1)�ł͂Ȃ��An�Ƃ���B
      vct_set(bfr_step[X] + one_step[X], bfr_step[Y] + one_step[Y], offset[Z] + ra * n, z->d);
	  z->di[1] = -1;
	  rotate_2d_p(offset[Z], z->d);
	  z->d[X] += offset[X];
	  z->d[Y] += offset[Y];
	}
    f_cnt++;
  }
}

#if 1    // height 320-71=249 0.02s
int f_stp = F_STEP;
FLOAT fi[] = {
		-1502.3, 347.52, 341.47, 295.22, 260.61, 230.31, 203.51, 179.83, 158.90, 140.41,
		 124.07, 109.63, 96.874, 85.600, 75.639, 66.837, 59.059, 52.187, 46.114, 40.748,
		 36.006, 31.816, 28.113, 24.842, 21.951, 19.397, 17.139, 15.145, 13.383, 11.825,
		 10.449, 9.2332, 8.1587, 7.2093, 6.3704, 5.6291, 4.9740, 4.3952, 3.8837, 3.4318 };
static FLOAT gj[F_STEP];

FLOAT ki[] = {1.7217e+003, 2.9604e+004, 5.3864e+003, 1.2570e+002};
#endif
#if 0    // height 300
int f_stp = F_STEP;
FLOAT fi[] = { -1283.6,	275.36,	266.28,	233.93,	208.87,	186.59,	166.68,	148.89,	133.01,	118.81,
				106.14,	94.811,	84.694,	75.657,	67.584,	60.373,	53.931,	48.176,	43.035,	38.443,
				34.341,	30.677,	27.404,	24.48,	21.867,	19.534,	17.45,	15.588,	13.924,	12.439,
				11.111,	9.9258,	8.8667,	7.9206,	7.0754,	6.3204,	5.646,	5.0436,	4.5054,	4.0246 };
static FLOAT gj[F_STEP];

FLOAT ki[] = {1449.5, 27177, 5367.3, 125.86};
#endif
#if 0    // height 300+100
int f_stp = F_STEP;
FLOAT fi[] = { -998.93, 189.26, 180.50, 161.91, 146.79, 133.10, 120.69, 109.44, 99.237, 89.984,
				81.594, 73.986,  67.088, 60.833, 55.161, 50.018, 45.354, 41.126, 37.291, 33.814,
				30.661, 27.803, 25.210, 22.860, 20.728, 18.796, 17.043, 15.454, 14.013, 12.707,
				11.522, 10.448, 9.4735, 8.5902, 7.7893, 7.0630, 6.4045, 5.8073, 5.2659, 4.7749};
 static FLOAT gj[F_STEP];

FLOAT ki[] = {1.1071e+003, 2.3752e+004, 5.3331e+003, 1.2597e+002};
#endif

static FLOAT ks, k1, k2, k3;
static FLOAT c1 = 1, c2 = 0, c3;

void fact_ini(FLOAT *k, FLOAT *fii, FLOAT height)
{
	for (int j = 0; j < f_stp; j++){
		gj[j] = 0;
		for (int i = j; i < f_stp; i++){
			gj[j] += fii[i];
		}
	}
	c3 = -height / ga;
	ks = k[0];
	k1 = k[1];
	k2 = k[2];
	k3 = k[3];
}

void prediction_ini(Prediction *pred)
{
	pred->bx = 0;
	pred->by = 0;
	pred->bdx = 0;
	pred->bdy = 0;
	pred->bddx = 0;
	pred->bddy = 0;
	pred->sezx = 0;
	pred->sezy = 0;
	pred->bux = 0;
	pred->buy = 0;
}


void prediction_copy(Prediction *_from, Prediction *_to)
{
	memcpy(_to, _from, sizeof(Prediction));
/*	_to->bx = _from->bx;
	_to->by = _from->by;
	_to->bdx = _from->bdx;
	_to->bdy = _from->bdy;
	_to->bddx = _from->bddx;
	_to->bddy = _from->bddy;
	_to->sezx = _from->sezx;
	_to->sezy = _from->sezy;
	_to->bux = _from->bux;
	_to->buy = _from->buy;
	*/
}


void motion_para_ini(Motion_para *mtp)
{
	mtp->nstep = 0;
	mtp->d_width = 0;
	mtp->s_hung = 0;
	mtp->d_yaw = 0;
	mtp->d_foot_yaw = 0;
	mtp->t_depth = 0;
	mtp->b_hung_offset = 0;  // ���O�X�e�[�W��hung_offset
	mtp->new_step = 0;
	mtp->at = 0;  // �ݐω�]�p�x  // _turn
	mtp->as = 0;  // 1���ł̉�]�p�x
	mtp->frame_cnt = 0;
	mtp->firsttime = 1;
}

void motion_para_copy(Motion_para *_from, Motion_para *_to)
{
	memcpy(_to, _from, sizeof(Motion_para));
}


void zmp_walk_online(FLOAT dt, Node *tz, FLOAT* gt, FLOAT* zx, FLOAT* zy, Prediction *pred, int *reset, FLOAT *ini)
{
	//FLOAT *ini : �����p���덷���z�����邽�߁A�ڕWZMP���덷�Ɠ��l�ɂ��邽�߂̓��͕ϐ� *reset=1 �̎��Abx,by�ɃZ�b�g�����
	static FLOAT dt2;
	static FLOAT dt3;
	FLOAT dx, ddx, dy, ddy;
	//FLOAT ezx, ezy;
	//FLOAT ux, uy;
	FLOAT sgrx, sgry;
	Node *nd;
	FLOAT bzx, bzy;
	if (*reset){
		dt2 = dt * dt;
		dt3 = dt2 * dt;
		pred->bx = ini[X];
		pred->by = ini[Y];
		pred->bdx = 0;
		pred->bdy = 0;
		pred->bddx = 0;
		pred->bddy = 0;
		pred->sezx = 0;
		pred->sezy = 0;
		pred->bux = 0;
		pred->buy = 0;
		*reset = 0;
	}
	gt[X] = pred->bx + pred->bdx * dt + pred->bddx * dt2 / 2 + pred->bux * dt3 / 6;
	gt[Y] = pred->by + pred->bdy * dt + pred->bddy * dt2 / 2 + pred->buy * dt3 / 6;
	gt[Z] = 0;
	dx = pred->bdx + pred->bddx * dt + pred->bux * dt2 / 4;
	dy = pred->bdy + pred->bddy * dt + pred->buy * dt2 / 4;
	ddx = pred->bddx + pred->bux * dt;
	ddy = pred->bddy + pred->buy * dt;
	*zx = gt[X] * c1 + ddx * c3;
	*zy = gt[Y] * c1 + ddy * c3;
	bzx = tz->d[0];  // nd == NULL�̏ꍇ������̂ŁAbzx,bzy�̐ݒ�͕K�v
	bzy = tz->d[1];  //
	//ezx = bzx - *zx;
	//ezy = bzy - *zy;
	pred->sezx += bzx - *zx;
	pred->sezy += bzy - *zy;
	sgrx = 0;
	sgry = 0;
	nd = tz->next;
	for (int j = 0; j < f_stp;j++){
		if (nd != NULL){
			sgrx += nd->d[0] * gj[j];
			sgry += nd->d[1] * gj[j];
			bzx = nd->d[0];
			bzy = nd->d[1];
			nd = nd->next;
		}
		else{
			sgrx += bzx * gj[j];
			sgry += bzy * gj[j];
		}
	}
	//ux = ks * pred->sezx - k1 * gt[X] - k2 * dx - k3 * ddx + sgrx;
	//uy = ks * pred->sezy - k1 * gt[Y] - k2 * dy - k3 * ddy + sgry;
	pred->bx = gt[X];
	pred->by = gt[Y];
	pred->bdx = dx;
	pred->bdy = dy;
	pred->bddx = ddx;
	pred->bddy = ddy;
	pred->bux = ks * pred->sezx - k1 * gt[X] - k2 * dx - k3 * ddx + sgrx;
	pred->buy = ks * pred->sezy - k1 * gt[Y] - k2 * dy - k3 * ddy + sgry;
}

void dynamics_filter(FLOAT dt, Node *dz, FLOAT* gt, FLOAT* rz, Prediction *pred, int *reset)
{
// FLOAT dt,  ���U�l(IN)
// Node *dz,   ZMP������(IN)
// FLOAT* gt, �d�S�ʒu�␳�l(OUT)
// FLOAT* rz  �␳�l�ł�ZMP(OUT)(�����p)
	//gj[]�̓O���[�o���ϐ����g���Ă���̂Œ��ӁB
	static FLOAT dt2;
	static FLOAT dt3;
	FLOAT dx, ddx, dy, ddy;
	FLOAT ezx, ezy;
	FLOAT ux, uy;
	FLOAT sgrx, sgry;
	Node *nd;
	FLOAT bzx, bzy;

	if (*reset){
		dt2 = dt * dt;
		dt3 = dt2 * dt;
		pred->bx = 0;
		pred->by = 0;
		pred->bdx = 0;
		pred->bdy = 0;
		pred->bddx = 0;
		pred->bddy = 0;
		pred->sezx = 0;
		pred->sezy = 0;
		pred->bux = 0;
		pred->buy = 0;
		*reset = 0;
	}
	gt[X] = pred->bx + pred->bdx * dt + pred->bddx * dt2 / 2 + pred->bux * dt3 / 6;
	gt[Y] = pred->by + pred->bdy * dt + pred->bddy * dt2 / 2 + pred->buy * dt3 / 6;
	gt[Z] = 0;
	dx = pred->bdx + pred->bddx * dt + pred->bux * dt2 / 4;
	dy = pred->bdy + pred->bddy * dt + pred->buy * dt2 / 4;
	ddx = pred->bddx + pred->bux * dt;
	ddy = pred->bddy + pred->buy * dt;
	rz[X] = gt[X] * c1 + ddx * c3;
	rz[Y] = gt[Y] * c1 + ddy * c3;
	bzx = dz->d[0];
	bzy = dz->d[1];
	ezx = bzx - rz[X];
	ezy = bzy - rz[Y];
	pred->sezx += ezx;
	pred->sezy += ezy;
	sgrx = 0;
	sgry = 0;
	nd = dz->next;
	for (int j = 0; j < f_stp; j++){
		if (nd != NULL){
			sgrx += nd->d[0] * gj[j];
			sgry += nd->d[1] * gj[j];
			bzx = nd->d[0];
			bzy = nd->d[1];
			nd = nd->next;
		}
		else{
			sgrx += bzx * gj[j];
			sgry += bzy * gj[j];
		}
	}
	ux = ks * pred->sezx - k1 * gt[X] - k2 * dx - k3 * ddx + sgrx;
	uy = ks * pred->sezy - k1 * gt[Y] - k2 * dy - k3 * ddy + sgry;
	pred->bx = gt[X];
	pred->by = gt[Y];
	pred->bdx = dx;
	pred->bdy = dy;
	pred->bddx = ddx;
	pred->bddy = ddy;
	pred->bux = ux;
	pred->buy = uy;
}



//�d�S�O�����󂯎���Ďp�����Ԃ��悤�ɂȂ��Ă��邪�A�d�S�ʒu���󂯎���Ďp����Ԃ��悤�ɕύX����B
//List* gt��while���[�v������̂ł�����P�P�ʂƂ��Ď��s����悤�ɂ���B
//����̂݁A��X�̕ϐ��ݒ�Ȃǂ��s������������B

int zmp_make_walk_motion(FLOAT *gt, Node *zmp, Lbp *lbp, int spivot, int npivot, FLOAT hung, int delay, FLOAT *gangle, Motion_para *mtp, Node** cp, int *cp_pivot)
{
	// gt �F�d�S�O��
	// zmp : �ڕWzmp
	// lbp : ���ݎp������ю���p��
	// spivot : �����̎����B�����̏ꍇ�� -1
	// npivot : �����̎����������̏ꍇ��1���ڂ̎���(��������o���ꍇ��npivot=RIGHT)
	// radius : mode = curve�̎��̂ݗL���@���s��]���a������
	// mode : 0:starte 1:side 2:curve 3:turn
	// gangle : ���{�b�g�̃O���[�o�����ʂ��i�[�����B

	//
    // ChangeLog(2011.1.15) param.f[_shift_zmp_**]���T�|�[�g  //20171020 shift_zmp����
	//                      lbp->o[Z]�̎g�����ύX�ɑΉ�
/*	static int nstep = 0;
	static Node *z, *zn;
	static FLOAT d_width = 0, s_hung = 0, d_yaw = 0, d_foot_yaw = 0;
	static FLOAT t_depth = 0, s_depth;
	static FLOAT s_hung_offset;
	static FLOAT b_hung_offset = 0;  // ���O�X�e�[�W��hung_offset
	static FLOAT dr_hung_offset;  // = b_hung_offset / delay
//	static FLOAT gp[3];
	static FLOAT rf, ro;
	static int cnt;
	static int new_step = 0;
	static int fst_stage;  // �����d�S�ړ�   // _side
	static int end_step; // �ŏI���t���O
	static int step_stage;
	//FLOAT ai = 0;  // �����p�x _curve�ɂ�radius>0 �̏ꍇ�ɂ̓΂�����B����ȊO��0������B
	static FLOAT at = 0;  // �ݐω�]�p�x  // _turn
	static FLOAT as = 0;  // 1���ł̉�]�p�x
	// �e��p�x�ɂ��Đ������K�v�B
	// a3 : ����ZMP�ւ̃x�N�g���p�x
	// a4 : ����ZMP�����_�Ƃ������{�b�g���_�ւ̃x�N�g���p�x�@��step_stage�̔���Ɏg���Ă���悤�Ȃ̂ŕs�v���H
	// _turn �ł�a3=���{�b�g�̉�]�p�x�ɂȂ�B�ݐω�]�p�x�ɉ�������p�x�ɂȂ邪�A
	// _curve �ł�a4�ƃZ�b�g�łȂ���ΈӖ����Ȃ��Ȃ��l�ƂȂ�B�ia4�Ƃ̍��傫�����step stage�Ɣ���j
	// _curve �̏ꍇ�� a2-a1 ���ݐω�]�p�x�ɉ�������p�x�ƂȂ邪�A�ݐς����Ƃ��Aa1-a0���ݐω�]�p�x�ƂȂ�B

	// _turn : as = a3;
	// _curve _strate : as = a2 - a1;




	static FLOAT wd[2];
	static int frame_cnt = 0;
	static int scnt;
	static int bmode;
	static int firsttime = 1;
*/
	Node *zn2;
	FLOAT rb_o[2];
	int mode = zmp->di[0];
	char s[100];

	#ifdef TEST
	static FILE *fp2;
	static FILE *fp3;
	#endif

	if (mtp->firsttime){  //�����ݒ�
#ifdef TEST
		if ((fp2 = fopen("C:\\Users\\kenji\\Documents\\zmp_walk_output.csv", "a")) == NULL){
			printf("file open error\n");
			exit(1);
		}
		fprintf(fp2, "frame,cnt,angle,robo-x, robo-y,gt-dx,gt-dy,gt-x,gt-y,offset-x,offset-y,offest-z,width,depth,hung,yaw, yaw-r, yaw-l, g-x, g-y, g-z\n");

		if ((fp3 = fopen("C:\\Users\\kenji\\Documents\\zmp_walk_log.csv", "a")) == NULL){
			printf("file open error\n");
			exit(1);
		}
#endif

		mtp->cnt = 0;
		//mtp->fst_step_arange = 0;
		if (spivot >= 0){
			mtp->fst_stage = 0;
			npivot = spivot;    //npivot�𐳋K��
		}
		else{
			mtp->fst_stage = 1;
		}
		mtp->firsttime = 0;
#ifdef TEST
		fprintf(fp3,"initialize parameters\n");
#endif
	}  //�����ݒ肱���܂�
	else{
		if(!mtp->cnt){
			if (mtp->fst_stage){    //�����n�߂̏d�S�ړ��̊Ԃ�fst_stage��1�̂܂܂Ƃ���
				mtp->fst_stage = 0;
			}else{
				mtp->fst_step_arange = 0;
			}
		}
	}
	{
		if (!mtp->cnt){
			//zmp�̕ω��_��T��
			mtp->zn = zmp;
			while (mtp->zn && zmp->d[X] == mtp->zn->d[X] && zmp->d[Y] == mtp->zn->d[Y]){
				mtp->cnt++;
				mtp->zn = mtp->zn->next;
			}
			if (!mtp->zn){
				// zn��zmp��I���܂ōs�����ꍇ
#ifdef TEST
				fprintf(fp3, "end of zmp list\n");
				fprintf(fp3, "[shift]zmp(%f:%f) count;%d\n", zmp->d[X], zmp->d[Y], mtp->cnt);
#endif
				mtp->step_stage = 0;
			}
			else{
				// �ŏI�����ǂ����𒲂ׂ�
				zn2 = mtp->zn;
				while (zn2 && mtp->zn->d[X] == zn2->d[X] && mtp->zn->d[Y] == zn2->d[Y]){
					zn2 = zn2->next;
				}
				if (!zn2){  // ���̕ω��_��������Ȃ��������ŏI��������
					mtp->end_step = 1;
				}
				else{
					mtp->end_step = 0;
				}
				// �x���r�̌���
				if (spivot < 0 && mtp->fst_stage){
					mtp->_p = (npivot + 1) % 2;      // _pn�����߂邽�߂ɒ�`
				}
				else{
					mtp->_p = (npivot + mtp->nstep) % 2;
				}
				mtp->_pn = (mtp->_p + 1) % 2;
				//wd[]�@����ZMP�ւ̃x�N�g���v�Z
				if (spivot < 0 && mtp->fst_stage){   // if(mtp->z == zmp) ����
					mtp->fst_step_arange = param.f[_faststep_arange] * (-mtp->_pn * 2 + 1);
					mtp->wd[X] = mtp->zn->d[X] - param.f[mtp->_pn] - mtp->fst_step_arange - zmp->d[X];
					mtp->wd[Y] = mtp->zn->d[Y] - param.f[mtp->_pn + 2] - zmp->d[Y];
				}
				else{
					if (!mtp->end_step){
						mtp->wd[X] = mtp->zn->d[X] - param.f[mtp->_pn] - (zmp->d[X] - param.f[mtp->_p] - mtp->fst_step_arange);
						mtp->wd[Y] = mtp->zn->d[Y] - param.f[mtp->_pn + 2] - (zmp->d[Y] - param.f[mtp->_p + 2]);
					}
					else{
						mtp->wd[X] = mtp->zn->d[X] - (zmp->d[X] - param.f[mtp->_p] - mtp->fst_step_arange);
						mtp->wd[Y] = mtp->zn->d[Y] - (zmp->d[Y] - param.f[mtp->_p + 2]);
					}
				}
				if (mtp->wd[Y] == 0) mtp->wd[Y] = 0;  // -0�ƂȂ�̂�h��
				rotate_2d_p(-mtp->at, mtp->wd);  //�ݐω�]�p�x���߂�

				mtp->tz[X] = param.f[mtp->_pn];
				mtp->tz[Y] = param.f[mtp->_pn + 2];    //Cf.tz[]��curve�����ł����ݒ肵�Ă��Ȃ��������ǂ����̂��ȁH
				rotate_2d_p(mtp->at, mtp->tz);
				mtp->as = mtp->zn->d[Z] - mtp->at;

				// step_stege�̌���
				if (!mtp->fst_stage){
#ifdef TEST
					fprintf(fp3, "[step]zmp(%f:%f) => (%f:%f) count;%d\n", zmp->d[X], zmp->d[Y], mtp->zn->d[X], mtp->zn->d[Y], mtp->cnt);
#endif
					mtp->step_stage = 1;
				}
				else{
					// �d�S�ړ��̂�
#ifdef TEST
					fprintf(fp3, "[shift]zmp(%f:%f) => (%f:%f) count;%d\n", zmp->d[X], zmp->d[Y], mtp->zn->d[X], mtp->zn->d[Y], mtp->cnt);
#endif
					mtp->step_stage = 0;
					//mtp->fst_stage = 0;�@�@�@//�����n�߂̏d�S�ړ��̊Ԃ�fst_stage��1�̂܂܂Ƃ���̂ŃR�����g�A�E�g
				}
				mtp->scnt = mtp->cnt;
			}
			mtp->new_step = 1;
		}
		//�u���s�v�Ȃ�Ε��s�p�p�����[�^�̐ݒ������B
		if (mtp->new_step){
			if (mtp->step_stage){
#ifdef TEST
				fprintf(fp3, "w=%f d=%f  zn=%f z=%f step:%d sign:%d npivot:%d spivot:%d\n", mtp->wd[X], mtp->wd[Y], mtp->zn->d[X], zmp->d[X], mtp->nstep, zmp_pivot_sign(npivot + mtp->nstep + 1), npivot, spivot);
#endif
				mtp->wd[X] *= zmp_pivot_sign(npivot + mtp->nstep + 1);
				mtp->wd[Y] *= zmp_pivot_sign(npivot + mtp->nstep + 1);
				if (mtp->end_step){ // �ŏI����wd[X]��2�{�ɂ���
#ifdef TEST
					fprintf(fp3, "last step !! \n");
#endif
					mtp->wd[X] *= 2;
					mtp->wd[Y] *= 2;
#ifdef TEST
				}
				else{
					fprintf(fp3, "not last step \n");
#endif
				}
#ifdef TEST
				fprintf(fp3, "lbp (w:%f d:%f) => (w:%f d:%f)\n", lbp->p[LBP_WIDTH], lbp->p[LBP_DEPTH], mtp->wd[X], mtp->wd[Y]);
#endif
				mtp->d_width = (mtp->wd[X] - lbp->p[LBP_WIDTH]) / (mtp->scnt - delay);
				mtp->t_depth = mtp->wd[Y] - lbp->p[LBP_DEPTH];
				mtp->s_depth = lbp->p[LBP_DEPTH];
				mtp->d_yaw = (mtp->as - lbp->p[LBP_YAW]) / mtp->cnt;
				mtp->d_foot_yaw = (mtp->as - lbp->p[LBP_YAW_RIGHT + free_leg(npivot + mtp->nstep)]) / (mtp->scnt - delay);
				mtp->s_hung = hung * zmp_pivot_sign(npivot + mtp->nstep + 1);
				mtp->s_hung_offset = hung_offset * zmp_pivot_sign(npivot + mtp->nstep + 1);
				//a->d[1] = zmp_pivot_sign(npivot + mtp->nstep);
			}
			else{
				mtp->d_width = 0;
				mtp->t_depth = 0;
				mtp->s_depth = 0;
				mtp->d_yaw = 0;
				mtp->d_foot_yaw = 0;
				mtp->s_hung = 0;
				mtp->s_hung_offset = 0;
				//a->d[1] = ROOT_SIGN_BOTH;
			}
			mtp->new_step = 0;
		}
		else{
			//a->d[1] = a->prev->d[1];
		}
		lbp->p[LBP_YAW] += mtp->d_yaw;
		rotate_2d_p(mtp->d_yaw, lbp->g);  //LBP_YAW�̕ω��ɂ�ĉ�]����̂ŊO���ŏC�����Ă�����
		vct_copy(gt, lbp->o);      //lbp->o = gt
		if (mtp->step_stage){
			vct_set(param.f[mtp->_p] + mtp->fst_step_arange, param.f[mtp->_p + 2], 0, mtp->tz);
			rotate_2d_p(mtp->at, mtp->tz);
			vct_minus(mtp->tz);        //tz = -tz
			vct_add(zmp->d, mtp->tz);    //tz += zmp
			vct_sub(mtp->tz, lbp->o);  //lbp->o -=tz
		}
		else{
			if (!mtp->fst_stage){    //�I�����C�������̗ݐό덷�΍�̂��߁Afst_stage����ZMP�Ƃ̍��������Ȃ��B
				vct_sub(zmp->d, lbp->o);  //lbp->o -= zmp
			}
		}
		rotate_2d_p(-mtp->at, lbp->o);
		lbp->o[Z] = 0;
		if (mtp->step_stage && mtp->scnt - mtp->cnt >= delay){
			lbp->p[LBP_DEPTH] = depth_func_mode ? DEPTH_FUNC2 : DEPTH_FUNC1;
			lbp->p[LBP_WIDTH] += mtp->d_width;
			lbp->p[LBP_YAW_RIGHT + free_leg(npivot + mtp->nstep)] += mtp->d_foot_yaw;
			lbp->p[LBP_HUNG] = (hung_func_mode ? HUNG_FUNC2 : HUNG_FUNC1) + mtp->s_hung_offset * (mtp->scnt - delay - mtp->cnt + 1.0) / (mtp->scnt - delay);
			if (mtp->cnt == 1){
				mtp->b_hung_offset = mtp->s_hung_offset;
				mtp->dr_hung_offset = mtp->b_hung_offset / delay;
			}
		}
		else{
			FLOAT tmp = mtp->b_hung_offset;
			mtp->b_hung_offset -= mtp->dr_hung_offset;
			if (fabs(tmp) < fabs(mtp->b_hung_offset)) mtp->b_hung_offset = 0;
			lbp->p[LBP_HUNG] = mtp->b_hung_offset;
		}
		if (mtp->step_stage){
			zmp_get_robot_origin(lbp, npivot + mtp->nstep, rb_o);
			lbp->o[X] -= rb_o[X];
			lbp->o[Y] -= rb_o[Y];
		}
		/*if (r = lbp_cnv_l_sv(lbp, gp, jo)){
#ifdef TEST
			fprintf(fp2, "zmp_make_walk_motion: lbp error : %d\n", r);
#endif
			//lbp_print(lbp);
		}*/
		*gangle = mtp->at + lbp->p[LBP_YAW];
		if (*gangle > M_PI) *gangle -= 2 * M_PI;
		else if (*gangle < -M_PI) *gangle += 2 * M_PI;
#ifdef TEST
		fprintf(fp2, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", mtp->frame_cnt, mtp->cnt, a->d[0], rb_o[X], rb_o[Y], zmp->d[0], zmp->d[1], gt[0], gt[1], lbp->o[X], lbp->o[Y], lbp->o[Z], lbp->p[LBP_WIDTH], lbp->p[LBP_DEPTH], lbp->p[LBP_HUNG], lbp->p[LBP_YAW], lbp->p[LBP_YAW_RIGHT], lbp->p[LBP_YAW_LEFT], lbp->g[X], lbp->g[Y], lbp->g[Z]);
#endif

		mtp->cnt--;
		if (!mtp->cnt && mtp->step_stage){
			lbp_rot(mtp->as, lbp);
			mtp->at += mtp->as;
			mtp->nstep++;
		}

		mtp->frame_cnt++;
	}
#ifdef TEST
	if (!zmp->next){
		fclose(fp2);
		fclose(fp3);
	}
#endif
	*cp = mtp->zn;
	if (mtp->end_step){
		*cp = NULL;
	}
	*cp_pivot = mtp->_pn;
	return(mtp->frame_cnt);
}




void zmp_arange(List *zmp, List *azmp, FLOAT shift_start, FLOAT shift_end, FLOAT shift_range)
{
	Node *z, *z_start;
  int step = 0;
  int cnt;
  FLOAT b_y;

  z = zmp->head;
  b_y = z->d[Y];
  while(z){
    z_start = z;
    cnt = 0;
    while(z && z->d[Y] == b_y){
#ifdef TEST
      printf("%f %f\n", z->d[X], z->d[Y]);
#endif
      cnt++;
      z = z->next;
    }
    if(z){
#ifdef TEST
      if(step == 0){
        printf("start_phase(%d) cnt = %d\n", step, cnt);
      }else{
        printf("step = %d, cnt = %d\n", step, cnt);
      }
#endif
      z = z_start;
      for(int i = 0; i < cnt; i++){
        list_push_back(azmp);
        azmp->tail->d[X] = z->d[X];
        azmp->tail->d[X] = z->d[Y];
        azmp->tail->d[Z] = z->d[Z];
        z = z->next;
      }
      z_start = z;
      b_y = z->d[Y];
      step++;
    }
  }
#ifdef TEST
  printf("end_phase(%d) cnt= %d\n", step, cnt);
#endif
}


int zmp_pivot_sign(int pivot)
{
// pivot��RIGHT(0)�Ȃ�1�ALEFT(1)�Ȃ�-1��Ԃ��B
  if(pivot < 0) return 0;
  else return (pivot % 2) * -2 + 1;
}

int zmp_pivot(int sign)
{
  // sign��1�Ȃ�RIGHT(0)�A-1�Ȃ�LEFT(1)��Ԃ��B
  if(sign == 0) return -1;
  else return (sign * -1 + 1) / 2;
}

void zmp_get_robot_origin(Lbp *lbp, int pivot, FLOAT *o)
{
  o[X] = zmp_pivot_sign(pivot + 1) * lbp->p[LBP_WIDTH] * 0.5;
  o[Y] = lbp->p[LBP_DEPTH] == 0 ? 0 : zmp_pivot_sign(pivot + 1) * lbp->p[LBP_DEPTH] * 0.5;  // -0 ��Ԃ��Ȃ��悤�ɁB
}

int free_leg(int pivot)
{
  return((pivot + 1) % 2);
}

void zmp_get_robot_gloval_position(FLOAT *zmp, Lbp *lbp, int pivot, FLOAT angle, FLOAT *gp)
     //zmp, lbp, pivot, angle����d�S�_�̃O���[�o�����W�����߂�B
{
  //zmp�����_�Ƃ������̃��{�b�g���_ro�̍��W�����߂�B
  zmp_get_robot_origin(lbp, pivot, gp);
  gp[X] += lbp->o[X];
  gp[Y] += lbp->o[Y];
  //zmp�̃O���[�o�����W�ƃ��{�b�g�̃O���[�o�����W�ɑ΂���p�x���烍�{�b�g���_�̃O���[�o�����W�����߂�B
  rotate_2d_p(angle, gp);
  gp[X] += zmp[X];
  gp[Y] += zmp[Y];
}

int nstep_side(int n, int md)
{
  if(md && n){
    n--;
  }
  if(n % 2){
    return (n + 1) / 2;
  }else{
    return n / 2 ;
  }
  //  n    0 1 2 3 4 5 6 ...
  // md==0 0 1 1 2 2 3 3 ...
  // md==1 0 0 1 1 2 2 3 ...
}

void rotate_2d_p(FLOAT a, FLOAT *p)
{
	FLOAT s = sinf(a);
	FLOAT c = cosf(a);
	FLOAT tx = p[X];
	FLOAT ty = p[Y];
	p[X] = tx * c - ty * s;
	p[Y] = tx * s + ty * c;
}

int step_counts;
int half_step;

void up_down_action_ini(FLOAT interval, FLOAT pace)
{
	step_counts = pace / interval;
	half_step = step_counts / 2;

}

FLOAT up_down_action(int count, FLOAT interval, FLOAT accel)
{
	int k = count < half_step ? count : half_step - count + half_step;
	FLOAT ki = k * interval;
	return accel * ki * ki;
}

int wfa_sw = 0;
int wfasc; //wfa_start_count
int wfad;
static FLOAT wfab_ = 0;

void walk_follow_action_ini(int count, int delay)
{
	if(!wfa_sw){
		wfasc = count+1 > delay ? count+1 : delay;
		wfad = delay;
		wfa_sw = 1;
	}
}

FLOAT walk_follow_back(int count)  //��������ɌĂԂ���
{
	FLOAT wfab;
	if(wfab_ != 0){
		if(count + 1 > wfad){
			wfab = (step_counts - count-1) * wfab_ /(step_counts - wfad);
			if(count+1 == step_counts){
				wfab_ = 0;
			}
			return(wfab);
		}else{
			return wfab_;
		}
	}else{
		return 0.0f;
	}
}

FLOAT walk_follow_action(int count, int dir, int pivot, FLOAT max)
{
	FLOAT wfa;
	if(wfa_sw && count+1 > wfasc){
		wfa = (count+1 - wfasc) * dir * (pivot * 2 - 1) * max / (step_counts - wfad);
		if(count+1 == step_counts){
			wfab_ = wfa;
			wfa_sw = 0;
		}
		return wfa;
	}else{
		return 0.0f;
	}
	//dir = 1 => forward
	//dir = -1  => backward
	//pivot = 0 => -1 => right
	//pivot = 1 => +1 => left
	//forward right => -
	//forward left => +
	//backward right => +
	//backward left => -
}

void walk_tilt_accumulate(int count, FLOAT threshold, FLOAT *acc, FLOAT zmp)
{
	if(count+1 == step_counts){
		*acc = 0;
	}else{
		if(fabs(zmp) > threshold){
			*acc += zmp;
		}
	}
}

/*
  ���i�ƁA�J�[�u�Ɛ���ł́A1����i�߂���̃��{�b�g�̉�]�p�x�̍l�������قȂ�B
  ���̂��߁AZMP�񂩂炾���ł͂��̈Ⴂ�𒊏o���邱�Ƃ͓���A���[�V������ς��邱�Ƃ��i�ȒP�ɂ́j�ł��Ȃ��B

  ��ZMP�񂪃}�N���łǂ����������Ɍ������Ă���̂��𕪐͂���΂ł��Ȃ��͂Ȃ��͂������B�B

  ���̕ӂ���l�������p�����[�^�̗^�������l����Γ���֐��ŏ������邱�Ƃ͉\���낤�B
  �������A�_���X�X�e�b�v�Ȃǂɍ��킹�����e�����ɉ��p����ɂ́uZMP�񂾂��ŕ��e�𐶐��A�A�v�Ƃ������Ƃ�
  ��������K�v�����邾�낤�B

 */
