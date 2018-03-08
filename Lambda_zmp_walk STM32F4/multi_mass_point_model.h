#ifndef __MULTI_MASS_POINT_MODEL_H__
#define __MULTI_MASS_POINT_MODEL_H__
#include <stdio.h>
#include <math.h>
#include "lambda.h"
#include "vector.h"
//#ifndef GOPRO�@�@�@�@�@�@�@��stbee�n��joint.h����ĂȂ������B
//#include "joint.h"
//#endif
#include "zmpwalk.h"

#define WITHOUT_LOAD    0
#define WITH_LOAD 1

#define MMPM_RIGHTLEG 0
#define MMPM_LEFTLEG  1
#define MMPM_RIGHTARM 2
#define MMPM_LEFTARM  3
#define MMPM_BODY     4

#define NOLOAD 0
#define LOAD 1

#define NO_DYN_FILTER 0
#define FORESEE  0
#define RESERVE 1
#define REMAKE  2

typedef struct _masspoint {
  FLOAT weight;
  FLOAT gp[3];    //�d�S���W�i���[�J�����W�j
  FLOAT ct[3];    //�R�l�N�^���W�i�e�̃��[�J�����W�j
  FLOAT I[3][3];
  FLOAT ang;    //(rad)�֐ߊp�x�͌Ҋ֐ߊ
  int jid;        //�֐�ID
  int axis;
  int flag;       //�v�Z�p�t���O
  //���_      (�\�����č\�z�̂��߂�2�ʂ̕ϐ������@�\�����v�Z�pgpos[0][]�@���ݍ��v�Z�pgpos[1][]�@�ύX���̖������č\�z�pgpos[2][])
  FLOAT gpos[3][3];          //��۰��ٍ��W�ɂ�����ʒu
  FLOAT bgpos[3][3];         //�O�ڰт�gpos
  FLOAT bbgpos[3][3];        // �O�X�ڰт�gpos
  FLOAT pdd[3][3];           //�����x
  FLOAT gposture[3][3][3];
  FLOAT bgposture[3][3][3];
  FLOAT bLp[3][3];
  //�֐ߓ_
  FLOAT jgpos[3];
  FLOAT axis_v[3];  // �֐ߎ��p��(�P�ʍs��)
  //�֐ߎ�
  FLOAT qd;             //�֐ߎ��̊p���x
  FLOAT qdd;            //�֐ߎ��̊p�����x
  FLOAT b_q;            //�O�ڰт̊֐ߊp�x
  FLOAT bb_q;           //�O�X�ڰт̊֐ߊp�x
  FLOAT v0[3], w[3];    //���_�̋�ԑ��x
  FLOAT v0d[3], wd[3];  //���_�̋�ԉ����x
  FLOAT u;              //�֐ߎ��Ŕ�������g���N
  
  struct _masspoint *next;    //�Z�탊�X�g����@�@�����e�������m�̃��X�g
  struct _masspoint *child;   //�q�����X�g����@�@�����̎q���̃��X�g
} massPoint;

massPoint *mp_set(
			FLOAT weight,
            FLOAT gp0, FLOAT gp1, FLOAT gp2,
            FLOAT ct0, FLOAT ct1, FLOAT ct2,
            FLOAT ixx, FLOAT ixy, FLOAT iyy, FLOAT ixz, FLOAT iyz, FLOAT izz,
            int _jid,  // _jid�͊֐�ID�������B0�`15�����B-1�Ŗ���
            int axis,
            massPoint *mp
			);

typedef struct _branch {
  //FLOAT ct[3];
  massPoint *mphead;
  //int touch;  //fix:0 free:1 notleg:2(free)
  FLOAT v0[3], w[3]; //�}�̋�ԑ��x
  FLOAT v0d[3], wd[3];  //�}�̋�ԉ����x
  FLOAT df;  //���ו��z���@distribution factor  ���̂�-1 ���̂��������v��1.0�ƂȂ�B
  int base_leg;  // �}���ǂ���̑����B
  struct _branch *next;
} Branch;
//mptail���Ȃ��̂́A���򃊃��N�̂��߁A�I�[����Ƃ͌���Ȃ�����



typedef struct _base {
  Branch *blist;
  FLOAT zmp[3][3];
  int ini_zmp[3];
  FLOAT pxc[3], pyc[3], pp[3];
  Branch *b[3];
} Base;

massPoint *mp_new(massPoint **mp);
void mp_print(massPoint *mp);

Branch *branch_new(Branch **b);
void branch_print(Branch *b);
void base_ini(Base *base);

void scan_mp(Base *base, void(*func)(massPoint*));
void base_get_gp(Base *base, FLOAT *j, int _from, int _to);  // _to �� _from���ꏏ�Ȃ�reset
int base_get_zmp(Base *base, FLOAT *gppos, FLOAT angle, FLOAT *j, FLOAT dt, int load, int idx);
void base_get_load(Base *base);
void base_make_bb_lbp(FLOAT gt[], FLOAT bzmp[][3], FLOAT bangle[], int bpivot[], FLOAT bgt[][3], Lbp blbp[]);
void adjmercury(Base *base);
FLOAT adj_joint_angle(FLOAT *t, FLOAT *p1, FLOAT *p2);
FLOAT noise_reduction(Node14 *np, int id);
FLOAT motor_speed(FLOAT t);
#endif
