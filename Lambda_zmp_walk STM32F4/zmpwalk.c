//zmpwalk.c

// ChangeLog(2011.1.15) zmp_make_plan()変更　zmp_make_walk_motion()変更
// ChangeLog(2011.1.16) zmp_make_plan_curve()変更　zmp_make_plan_turn()変更
// ChangeLog(2011.2.21) zmp_make_plan_turn()変更

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

FLOAT next_step[3];  //最終歩が最終じゃなかった場合の次の目標ZMP座標
int next_step_flag;

/////////////////////////////////////////
////    ZMP 調整用パラメータ 2010.12.21
//FLOAT param.f[4] = { 0, 0, 0, 0 };
////    足裏中心からZMPをずらすパラメータ
/////////////////////////////////////////


void zmp_make_plan(int spivot, int npivot, FLOAT width, FLOAT step, FLOAT pace, int n, FLOAT dt, FLOAT *offset, FLOAT *ini, List *zmp)
{
  // spivot : 初期の軸足。両足の場合は -1
  // npivot : 初期の軸足が両足の場合に1歩目の軸足(左足から出す場合はnpivot=RIGHT)
  // ChangeLog(2011.1.15) param.f[_shift_zmp_**]をサポート  //20171020 shift_zmp改め
  int nstep;
  Node *z;
  FLOAT sft;
  int f_cnt = 0;
  int step_frm = (int)(pace / dt);
  int st_frm;
  int p;

  next_step_flag = 0;
  list_clear(zmp);
  
  if(spivot < 0){  // 静止からスタート
    st_frm = step_frm * 14 / 10;
    sft = 0;
	p = spivot;
  }else{
    st_frm = 0;
    npivot = spivot;
    sft = width * (spivot % 2 ? -0.5 : 0.5);
  }
  int ed_frm = step_frm * 14 / 10;

  while(f_cnt < st_frm + (n + 1) * step_frm + ed_frm){  //歩数が n の時、ZMP点は n+1 となる。　・－・－・－・　－が歩数、・がZMP点
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
	//FLOAT *ini : 初期姿勢誤差を吸収するため、目標ZMPを誤差と同値にするための入力変数 spivot = -1の時に使用される
  int nstep;
  Node *z;
  FLOAT sft;
  int f_cnt = 0;
  int step_frm = (int)(pace / dt);
  int st_frm;
  int md;

  next_step_flag = 0;
  list_clear(zmp);

  // 解説：移動方向はstepの符号で決まる。step > 0 で右に、step < 0 で左に移動。
  //       静止からスタートの場合、npivotが最初の軸足となる。
  //       npivot=RIGHT,step>0の場合、widthが狭く→普通→狭く→普通
  //       npivot=RIGHT,step<0の場合、widthが拡がる→普通→拡がる→普通　と歩くことになる。
  //       nは歩数。n=0の場合、widthがstep分だけ変化して終了。widthを変化させるために使用することを想定。
  //         ただし、この場合は継続が不自然となると思われる。
  //
  //       スタート時のnpivotとstepの符号で動作が変わるため、継続の場合にもnpivotを参照する。
  //       継続モードの時(spivot>=0の時）、
  //         出足と同じ足が軸足で開始する場合は(npivot==spivot) width普通
  //         出足と違う足が軸足で開始する場合は(npivot!=spivot) width狭く、または拡がるで始まる。
  //       これらは、nstep_size()に渡す引数で制御している。
  
  if(spivot < 0){  // 静止からスタート
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
  // spivot : 0:right 1:left -1:both 引継ぎ時の軸足
  // npivot : 次の軸足　spivotが-1の時だけ参照する。
  // offset : ロボットのグローバル座標 offset[Z]はグローバル角度
  // ChangeLog(2011.1.15) param.f[_shift_zmp_**]をサポート  //20171020 shift_zmp改め

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
    //1歩での旋回角度を求める
    rf = radius + (radius > 0 ? width : -width);  //pivotに関係なくrf = radius + width
    rc = (radius + rf) * 0.5;
    if(spivot < 0)
      ro = rc;
    else if(radius * zmp_pivot_sign(spivot) < 0)
      ro = radius;
    else
      ro = rf;
    ra = step / rf;
    //printf("ro = %f ra = %f\n", ro, ra);
    //回転中心は、(ro, 0)
  }
  if(spivot < 0){  // 静止からスタート
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
  // spivot : 0:right 1:left -1:both 引継ぎ時の軸足
  // npivot : 初期支持脚が-1の時の支持脚を常に指定。（この関数では継続歩行の場合にも参照するので注意）
  // s_width : 歩行開始時の脚幅。初期支持脚が-1の時に有効。
  // ChangeLog(2011.1.15) param.f[_shift_zmp_**]をサポート  //20171020 shift_zmp改め
  // ChangeLog(2011.2.21) s_widthをサポート

  //FLOAT o[2];  // 回転中心
  int nstep;
  Node *z;
  FLOAT ra;

  int f_cnt = 0;  // フレームカウンター
  int step_frm = (int)(pace / dt);  // 1歩のフレーム数
  int st_frm;  // 初期重心移動ステージのフレーム数　継続歩行の場合は0が入る
  int final_step = 0;  // ファイナルステージフラグ
  int bstep = 0;  // 前回のnstep
  int p; //軸足
  int f; //遊脚
  FLOAT bfr_step[2];  // 前回までの歩でのZMP座標
  FLOAT one_step[2];  // 今回の1歩でのZMP移動量
  FLOAT _bfr_step[2];

  next_step_flag = 0;
  list_clear(zmp);

  //解説：前回の位置に、今回の1歩での差分を加える形で目標ZMPを生成している。
  //      bfr_step[]に前回の位置が格納されており、one_step[]に今回の1歩での移動分が格納される。
  //      bfr_step[]の更新はフレームが新しい歩になった時に行われる（new_step == 1の時）
  //      最新のZMP座標はbfr_step[] + rotate(nstep * ra, one_step[]) で得られる。ra:1歩での旋回角度
  
  //1歩での旋回角度を求める
  ra = step / width * zmp_pivot_sign(npivot + 1);
  //解説：ターンの場合、どちら回転の歩行であるかは、最初の軸足とステップの方向で決まる。
  //　　　なので、継続歩行の場合もnpivotが有効であり、指定されたnpivotにて旋回角度の符号を決める必要がある。

  if(spivot < 0){  // 静止からスタート
    st_frm = step_frm * 14 / 10;
  }else{
    st_frm = 0;
    npivot = spivot;
  }
  //最初の「ステップステージ」のためのbfr_step[]設定
//  bfr_step[X] = width * (spivot < 0 ? 0.5 : 1) * zmp_pivot_sign(npivot + 1);
  bfr_step[X] = s_width * (spivot < 0 ? 0.5 : 1) * zmp_pivot_sign(npivot + 1); // add s_width
  bfr_step[Y] = 0;
  //解説：直前の直前の軸脚座標を入れておく。静止からのスタートの場合、最初の軸足の反対の脚が軸足
  //      だったとした座標を入れておく。ここにはparam.f[_shift_zmp_**]は加えない。

  int ed_frm = step_frm * 14 / 10;
  while(f_cnt < st_frm + (n + 1) * step_frm + ed_frm){
    z = list_push_back(zmp);
	z->di[0] = _turn;
	if (st_frm && f_cnt < st_frm){  // 「静止からスタート」で、初期ステージ
      vct_set(offset[X] + ini[X], offset[Y] + ini[Y], offset[Z], z->d);
	  z->di[1] = -1;
	}
	else if (f_cnt < st_frm + (n + 1) * step_frm){  // ステップステージ
      nstep = (f_cnt - st_frm) / step_frm;
      p = (npivot + nstep) % 2;
      f = (p + 1) % 2;
      if(bstep < nstep){ // 新歩に入ったのでbfr_step[]更新(bfr_step[]にはparam.f[_shfit_zmp_**]を含まない)
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
	else if (f_cnt < st_frm + (n + 1) * step_frm + ed_frm){  // 最終ステージ
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
      if(!final_step){  // このブロックはこのステージに入った最初のフレームだけ実行
        one_step[X] = width * zmp_pivot_sign(npivot + n);
        one_step[Y] = 0;
        rotate_2d_p(n * ra, one_step);
        bfr_step[X] += one_step[X];
        bfr_step[Y] += one_step[Y];
        final_step = 1;  // このブロックを既に実行したことを示すために1とする。
      }
      one_step[X] = s_width * 0.5 * zmp_pivot_sign(npivot + n + 1);
      one_step[Y] = 0;
      rotate_2d_p(n * ra, one_step);  // 最終歩ではターンしないため、(n+1)ではなく、nとする。
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
	mtp->b_hung_offset = 0;  // 直前ステージのhung_offset
	mtp->new_step = 0;
	mtp->at = 0;  // 累積回転角度  // _turn
	mtp->as = 0;  // 1歩での回転角度
	mtp->frame_cnt = 0;
	mtp->firsttime = 1;
}

void motion_para_copy(Motion_para *_from, Motion_para *_to)
{
	memcpy(_to, _from, sizeof(Motion_para));
}


void zmp_walk_online(FLOAT dt, Node *tz, FLOAT* gt, FLOAT* zx, FLOAT* zy, Prediction *pred, int *reset, FLOAT *ini)
{
	//FLOAT *ini : 初期姿勢誤差を吸収するため、目標ZMPを誤差と同値にするための入力変数 *reset=1 の時、bx,byにセットされる
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
	bzx = tz->d[0];  // nd == NULLの場合があるので、bzx,bzyの設定は必要
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
// FLOAT dt,  離散値(IN)
// Node *dz,   ZMP差分列(IN)
// FLOAT* gt, 重心位置補正値(OUT)
// FLOAT* rz  補正値でのZMP(OUT)(ﾓﾆﾀｰ用)
	//gj[]はグローバル変数を使っているので注意。
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



//重心軌道を受け取って姿勢列を返すようになっているが、重心位置を受け取って姿勢を返すように変更する。
//List* gtのwhileループがあるのでこれを１単位として実行するようにする。
//初回のみ、種々の変数設定などを行う処理が入る。

int zmp_make_walk_motion(FLOAT *gt, Node *zmp, Lbp *lbp, int spivot, int npivot, FLOAT hung, int delay, FLOAT *gangle, Motion_para *mtp, Node** cp, int *cp_pivot)
{
	// gt ：重心軌道
	// zmp : 目標zmp
	// lbp : 現在姿勢および次回姿勢
	// spivot : 初期の軸足。両足の場合は -1
	// npivot : 初期の軸足が両足の場合に1歩目の軸足(左足から出す場合はnpivot=RIGHT)
	// radius : mode = curveの時のみ有効　歩行回転半径が入る
	// mode : 0:starte 1:side 2:curve 3:turn
	// gangle : ロボットのグローバル方位が格納される。

	//
    // ChangeLog(2011.1.15) param.f[_shift_zmp_**]をサポート  //20171020 shift_zmp改め
	//                      lbp->o[Z]の使い方変更に対応
/*	static int nstep = 0;
	static Node *z, *zn;
	static FLOAT d_width = 0, s_hung = 0, d_yaw = 0, d_foot_yaw = 0;
	static FLOAT t_depth = 0, s_depth;
	static FLOAT s_hung_offset;
	static FLOAT b_hung_offset = 0;  // 直前ステージのhung_offset
	static FLOAT dr_hung_offset;  // = b_hung_offset / delay
//	static FLOAT gp[3];
	static FLOAT rf, ro;
	static int cnt;
	static int new_step = 0;
	static int fst_stage;  // 初期重心移動   // _side
	static int end_step; // 最終歩フラグ
	static int step_stage;
	//FLOAT ai = 0;  // 初期角度 _curveにてradius>0 の場合にはπが入る。それ以外は0が入る。
	static FLOAT at = 0;  // 累積回転角度  // _turn
	static FLOAT as = 0;  // 1歩での回転角度
	// 各種角度について整理が必要。
	// a3 : 次のZMPへのベクトル角度
	// a4 : 今のZMPを原点としたロボット原点へのベクトル角度　→step_stageの判定に使っているようなので不要か？
	// _turn ではa3=ロボットの回転角度になる。累積回転角度に加えられる角度になるが、
	// _curve ではa4とセットでなければ意味をなさない値となる。（a4との差大きければstep stageと判定）
	// _curve の場合は a2-a1 が累積回転角度に加えられる角度となるが、累積せずとも、a1-a0が累積回転角度となる。

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

	if (mtp->firsttime){  //初期設定
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
			npivot = spivot;    //npivotを正規化
		}
		else{
			mtp->fst_stage = 1;
		}
		mtp->firsttime = 0;
#ifdef TEST
		fprintf(fp3,"initialize parameters\n");
#endif
	}  //初期設定ここまで
	else{
		if(!mtp->cnt){
			if (mtp->fst_stage){    //歩き始めの重心移動の間はfst_stageは1のままとする
				mtp->fst_stage = 0;
			}else{
				mtp->fst_step_arange = 0;
			}
		}
	}
	{
		if (!mtp->cnt){
			//zmpの変化点を探す
			mtp->zn = zmp;
			while (mtp->zn && zmp->d[X] == mtp->zn->d[X] && zmp->d[Y] == mtp->zn->d[Y]){
				mtp->cnt++;
				mtp->zn = mtp->zn->next;
			}
			if (!mtp->zn){
				// znがzmp列終了まで行った場合
#ifdef TEST
				fprintf(fp3, "end of zmp list\n");
				fprintf(fp3, "[shift]zmp(%f:%f) count;%d\n", zmp->d[X], zmp->d[Y], mtp->cnt);
#endif
				mtp->step_stage = 0;
			}
			else{
				// 最終歩かどうかを調べる
				zn2 = mtp->zn;
				while (zn2 && mtp->zn->d[X] == zn2->d[X] && mtp->zn->d[Y] == zn2->d[Y]){
					zn2 = zn2->next;
				}
				if (!zn2){  // 次の変化点が見つからなかった＝最終歩だった
					mtp->end_step = 1;
				}
				else{
					mtp->end_step = 0;
				}
				// 支持脚の決定
				if (spivot < 0 && mtp->fst_stage){
					mtp->_p = (npivot + 1) % 2;      // _pnを決めるために定義
				}
				else{
					mtp->_p = (npivot + mtp->nstep) % 2;
				}
				mtp->_pn = (mtp->_p + 1) % 2;
				//wd[]　次のZMPへのベクトル計算
				if (spivot < 0 && mtp->fst_stage){   // if(mtp->z == zmp) 改め
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
				if (mtp->wd[Y] == 0) mtp->wd[Y] = 0;  // -0となるのを防ぐ
				rotate_2d_p(-mtp->at, mtp->wd);  //累積回転角度分戻す

				mtp->tz[X] = param.f[mtp->_pn];
				mtp->tz[Y] = param.f[mtp->_pn + 2];    //Cf.tz[]はcurveだけでしか設定していなかったけどいいのかな？
				rotate_2d_p(mtp->at, mtp->tz);
				mtp->as = mtp->zn->d[Z] - mtp->at;

				// step_stegeの決定
				if (!mtp->fst_stage){
#ifdef TEST
					fprintf(fp3, "[step]zmp(%f:%f) => (%f:%f) count;%d\n", zmp->d[X], zmp->d[Y], mtp->zn->d[X], mtp->zn->d[Y], mtp->cnt);
#endif
					mtp->step_stage = 1;
				}
				else{
					// 重心移動のみ
#ifdef TEST
					fprintf(fp3, "[shift]zmp(%f:%f) => (%f:%f) count;%d\n", zmp->d[X], zmp->d[Y], mtp->zn->d[X], mtp->zn->d[Y], mtp->cnt);
#endif
					mtp->step_stage = 0;
					//mtp->fst_stage = 0;　　　//歩き始めの重心移動の間はfst_stageは1のままとするのでコメントアウト
				}
				mtp->scnt = mtp->cnt;
			}
			mtp->new_step = 1;
		}
		//「歩行」ならば歩行用パラメータの設定をする。
		if (mtp->new_step){
			if (mtp->step_stage){
#ifdef TEST
				fprintf(fp3, "w=%f d=%f  zn=%f z=%f step:%d sign:%d npivot:%d spivot:%d\n", mtp->wd[X], mtp->wd[Y], mtp->zn->d[X], zmp->d[X], mtp->nstep, zmp_pivot_sign(npivot + mtp->nstep + 1), npivot, spivot);
#endif
				mtp->wd[X] *= zmp_pivot_sign(npivot + mtp->nstep + 1);
				mtp->wd[Y] *= zmp_pivot_sign(npivot + mtp->nstep + 1);
				if (mtp->end_step){ // 最終歩はwd[X]を2倍にする
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
		rotate_2d_p(mtp->d_yaw, lbp->g);  //LBP_YAWの変化につれて回転するので外部で修正してあげる
		vct_copy(gt, lbp->o);      //lbp->o = gt
		if (mtp->step_stage){
			vct_set(param.f[mtp->_p] + mtp->fst_step_arange, param.f[mtp->_p + 2], 0, mtp->tz);
			rotate_2d_p(mtp->at, mtp->tz);
			vct_minus(mtp->tz);        //tz = -tz
			vct_add(zmp->d, mtp->tz);    //tz += zmp
			vct_sub(mtp->tz, lbp->o);  //lbp->o -=tz
		}
		else{
			if (!mtp->fst_stage){    //オンライン生成の累積誤差対策のため、fst_stage中はZMPとの差分を取らない。
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
// pivotがRIGHT(0)なら1、LEFT(1)なら-1を返す。
  if(pivot < 0) return 0;
  else return (pivot % 2) * -2 + 1;
}

int zmp_pivot(int sign)
{
  // signが1ならRIGHT(0)、-1ならLEFT(1)を返す。
  if(sign == 0) return -1;
  else return (sign * -1 + 1) / 2;
}

void zmp_get_robot_origin(Lbp *lbp, int pivot, FLOAT *o)
{
  o[X] = zmp_pivot_sign(pivot + 1) * lbp->p[LBP_WIDTH] * 0.5;
  o[Y] = lbp->p[LBP_DEPTH] == 0 ? 0 : zmp_pivot_sign(pivot + 1) * lbp->p[LBP_DEPTH] * 0.5;  // -0 を返さないように。
}

int free_leg(int pivot)
{
  return((pivot + 1) % 2);
}

void zmp_get_robot_gloval_position(FLOAT *zmp, Lbp *lbp, int pivot, FLOAT angle, FLOAT *gp)
     //zmp, lbp, pivot, angleから重心点のグローバル座標を求める。
{
  //zmpを原点とした時のロボット原点roの座標を求める。
  zmp_get_robot_origin(lbp, pivot, gp);
  gp[X] += lbp->o[X];
  gp[Y] += lbp->o[Y];
  //zmpのグローバル座標とロボットのグローバル座標に対する角度からロボット原点のグローバル座標を求める。
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

FLOAT walk_follow_back(int count)  //こちらを先に呼ぶこと
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
  直進と、カーブと旋回では、1歩を進めた後のロボットの回転角度の考え方が異なる。
  そのため、ZMP列からだけではその違いを抽出することは難しく、モーションを変えることが（簡単には）できない。

  ＃ZMP列がマクロでどういう方向に向かっているのかを分析すればできなくはないはずだが。。

  その辺りを考慮したパラメータの与え方を考えれば同一関数で処理することは可能だろう。
  ただし、ダンスステップなどに合わせた歩容生成に応用するには「ZMP列だけで歩容を生成、、」ということを
  実現する必要があるだろう。

 */
