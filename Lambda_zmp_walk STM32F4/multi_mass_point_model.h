#ifndef __MULTI_MASS_POINT_MODEL_H__
#define __MULTI_MASS_POINT_MODEL_H__
#include <stdio.h>
#include <math.h>
#include "lambda.h"
#include "vector.h"
//#ifndef GOPRO　　　　　　　→stbee系はjoint.h作ってなかった。
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
  FLOAT gp[3];    //重心座標（ローカル座標）
  FLOAT ct[3];    //コネクタ座標（親のローカル座標）
  FLOAT I[3][3];
  FLOAT ang;    //(rad)関節角度は股関節基準
  int jid;        //関節ID
  int axis;
  int flag;       //計算用フラグ
  //質点      (予見項再構築のために2面の変数を持つ　予見項計算用gpos[0][]　現在項計算用gpos[1][]　変更時の未来項再構築用gpos[2][])
  FLOAT gpos[3][3];          //ｸﾞﾛｰﾊﾞﾙ座標における位置
  FLOAT bgpos[3][3];         //前ﾌﾚｰﾑのgpos
  FLOAT bbgpos[3][3];        // 前々ﾌﾚｰﾑのgpos
  FLOAT pdd[3][3];           //加速度
  FLOAT gposture[3][3][3];
  FLOAT bgposture[3][3][3];
  FLOAT bLp[3][3];
  //関節点
  FLOAT jgpos[3];
  FLOAT axis_v[3];  // 関節軸姿勢(単位行列)
  //関節軸
  FLOAT qd;             //関節軸の角速度
  FLOAT qdd;            //関節軸の角加速度
  FLOAT b_q;            //前ﾌﾚｰﾑの関節角度
  FLOAT bb_q;           //前々ﾌﾚｰﾑの関節角度
  FLOAT v0[3], w[3];    //質点の空間速度
  FLOAT v0d[3], wd[3];  //質点の空間加速度
  FLOAT u;              //関節軸で発生するトルク
  
  struct _masspoint *next;    //兄弟リスト下り　　同じ親を持つモノのリスト
  struct _masspoint *child;   //子供リスト下り　　自分の子供のリスト
} massPoint;

massPoint *mp_set(
			FLOAT weight,
            FLOAT gp0, FLOAT gp1, FLOAT gp2,
            FLOAT ct0, FLOAT ct1, FLOAT ct2,
            FLOAT ixx, FLOAT ixy, FLOAT iyy, FLOAT ixz, FLOAT iyz, FLOAT izz,
            int _jid,  // _jidは関節IDを示す。0～15が足。-1で無効
            int axis,
            massPoint *mp
			);

typedef struct _branch {
  //FLOAT ct[3];
  massPoint *mphead;
  //int touch;  //fix:0 free:1 notleg:2(free)
  FLOAT v0[3], w[3]; //枝の空間速度
  FLOAT v0d[3], wd[3];  //枝の空間加速度
  FLOAT df;  //負荷分配率　distribution factor  胴体は-1 胴体を除く合計が1.0となる。
  int base_leg;  // 枝がどちらの足か。
  struct _branch *next;
} Branch;
//mptailがないのは、分岐リンクのため、終端が一つとは限らないため



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
void base_get_gp(Base *base, FLOAT *j, int _from, int _to);  // _to と _fromが一緒ならreset
int base_get_zmp(Base *base, FLOAT *gppos, FLOAT angle, FLOAT *j, FLOAT dt, int load, int idx);
void base_get_load(Base *base);
void base_make_bb_lbp(FLOAT gt[], FLOAT bzmp[][3], FLOAT bangle[], int bpivot[], FLOAT bgt[][3], Lbp blbp[]);
void adjmercury(Base *base);
FLOAT adj_joint_angle(FLOAT *t, FLOAT *p1, FLOAT *p2);
FLOAT noise_reduction(Node14 *np, int id);
FLOAT motor_speed(FLOAT t);
#endif
