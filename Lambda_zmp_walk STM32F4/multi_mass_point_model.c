//multi_mass_point_model.c

#include <stdlib.h>
#include "lambda.h"
#include "vector.h"
#include "multi_mass_point_model.h"
#include "lbposture.h"
//#include "motion.h"  // use DefaultData

#ifdef TEST
extern FILE *fp;
extern FILE *fp3;
extern FILE *fp5;
#endif

char s[100];
//#define RECORD
void tmcom_puts(char *send_data);


massPoint *mp_set(FLOAT weight,
            FLOAT gp0, FLOAT gp1, FLOAT gp2,
            FLOAT ct0, FLOAT ct1, FLOAT ct2,
            FLOAT ixx, FLOAT ixy, FLOAT iyy, FLOAT ixz, FLOAT iyz, FLOAT izz,
            int _jid,
            int axis,
            massPoint *mp)
{
  mp->weight = weight;
  vct_set(gp0, gp1, gp2, mp->gp);
  vct_set(ct0, ct1, ct2, mp->ct);
  mat_set(ixx, ixy, iyz, ixy, iyy, iyz, ixz, iyz, izz, mp->I);
  mp->jid = _jid;
  mp->axis = axis;
  return mp;
}

massPoint *mp_new(massPoint **mp)
{
	//親につなぐか兄弟につなぐかでリンク先がことなるため、リンク先を引数で受け取る。
  massPoint *_mp;
  _mp = (massPoint *)malloc(sizeof(massPoint));
  for (int i = 0; i < 2; i++){
	  for (int j = 0; j < 3; j++){
		  _mp->bgpos[i][j] = 0;
		  _mp->bbgpos[i][j] = 0;
		  for (int k = 0; k < 3; k++){
			  _mp->bgposture[i][j][k] = 0;
		  }
	  }
  }
  if (!_mp){
	  return NULL;
  }
  *mp = _mp;
  _mp->child = NULL;
  _mp->next = NULL;
  return _mp;
}

void mp_print(massPoint *mp)
{
#ifdef TEST
  printf("-w:%f gp:%f %f %f\n", mp->weight, mp->gp[0], mp->gp[1], mp->gp[2]);
  printf(" ct:%f %f %f\n", mp->ct1[0], mp->ct1[1], mp->ct1[2]);
#endif
}

Branch *branch_new(Branch **b)
{
  Branch *_b, *_bn;
  _b = (Branch *) malloc(sizeof(Branch));
  if(!_b)  {
	  return NULL;
  }
  if(!*b){
    *b = _b;
  }else{
    _bn = *b;
    while(_bn->next){
      _bn = _bn->next;
    }
    _bn->next = _b;
  }
  _b->mphead = NULL;
  _b->next = NULL;
  return _b;
}

void branch_print(Branch *b)
{
  //printf("connector %f %f %f\n", b->ct[0], b->ct[1], b->ct[2]);
}

void base_ini(Base *base)
{
  base->blist = NULL;
  base->ini_zmp[FORESEE] = 1;
  base->ini_zmp[RESERVE] = 1;
  base->ini_zmp[REMAKE] = 0;  //REMAKEで初期から開始することはないので常に0
}

#ifdef RECORD
FILE *fp6;
FILE *fp7;
#endif

void scan_mp1(massPoint *mp, void (*func)(massPoint*))
{
	(*func)(mp);
	if (mp->child != NULL){  //子供
		scan_mp1(mp->child, func);
	}
	if (mp->next != NULL){   //next(兄弟)
		scan_mp1(mp->next, func);
	}
}

void scan_mp(Base *base, void (*func)(massPoint*))
{
	Branch *b;
	b = base->blist;
	while (b){
		scan_mp1(b->mphead, func);
		b = b->next;
	}
}


void get_gp_scan_mp1(massPoint *mp, FLOAT angle, int axis, FLOAT *ct, FLOAT *j, int _from, int _to)
{
	if (ct[2] < -999999){
		if (mp->jid >= 0){  //angを確定する
            if(j){
            mp->ang = j[mp->jid];
            }else{
                mp->ang = conv_sv_f(&joint[mp->jid],joint[mp->jid].d[JOINT_POS]);
            }
	    }else if(mp->jid == -2){
	    	mp->ang = 0;
		}
		vct_copy(mp->gp, mp->gpos[_to]);
		vct_rotate(mp->ang, mp->axis, mp->gpos[_to]);  //自分のangで回転
#ifdef RECORD
		fprintf(fp6, "mp,%f,%f,%d,%f,%f,%f,%f,%f,%f\n", mp->weight, mp->ang, mp->axis, mp->ct[0], mp->ct[1], mp->ct[2], mp->gp[0], mp->gp[1], mp->gp[2]);
#endif
		vct_add(mp->ct, mp->gpos[_to]);  //コネクタを追加
	}
	else{
		vct_add(ct, mp->gpos[_to]);  //上位のコネクタを追加
	}
	vct_rotate(angle, axis, mp->gpos[_to]);  //自分を上位からの角度で回転
#ifdef RECORD
	if (ct[2] < -999999){
		fprintf(fp6, "mp1,%f,%f,%d,%f,%f,%f,%f,%f,%f\n", mp->weight, angle, axis, mp->ct[0], mp->ct[1], mp->ct[2], mp->gpos[0], mp->gpos[1], mp->gpos[2]);
	}
	else{
		fprintf(fp6, "mp1,%f,%f,%d,%f,%f,%f,%f,%f,%f\n", mp->weight, angle, axis, ct[0], ct[1], ct[2], mp->gpos[0], mp->gpos[1], mp->gpos[2]);
	}
#endif

	if (mp->child != NULL){  //子供
		if (ct[2] < -999999){
			get_gp_scan_mp1(mp->child, mp->ang, mp->axis, ct, j, _from, _to);
			get_gp_scan_mp1(mp->child, angle, axis, mp->ct, j, _from, _to);
		}
		else{
			get_gp_scan_mp1(mp->child, angle, axis, ct, j, _from, _to);
		}
	}
	if (mp->next != NULL){   //next(兄弟)
		get_gp_scan_mp1(mp->next, angle, axis, ct, j, _from, _to);
	}
}

void get_gp_scan_mp2(massPoint *mp, FLOAT *zmp, FLOAT *weight, int _from, int _to)
{
	FLOAT tmp[3];
	if (mp->child != NULL){  //childには自分の角度を渡す
		get_gp_scan_mp2(mp->child, zmp, weight, _from, _to);
	}
	if (mp->next != NULL){   //next(兄弟)には上位から来た角度を渡す
		get_gp_scan_mp2(mp->next, zmp, weight, _from, _to);
	}
	vct_copy(mp->gpos[_to], tmp);
	vct_mul(mp->weight, tmp);
	vct_add(tmp, zmp);
	*weight += mp->weight;
	if (_to == _from){
		//内部変数の初期化(ZMP用) 
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				mp->bgpos[i][j] = 0;
				mp->bbgpos[i][j] = 0;
#ifdef MOMENT
				for (int k = 0; k < 3; k++){
					mp->bgposture[i][j][k] = 0;
				}
#endif
			}
		}
	}
	else{
		for (int j = 0; j < 3; j++){
			mp->bgpos[_to][j] = mp->bgpos[_from][j];
			mp->bbgpos[_to][j] = mp->bbgpos[_from][j];
#ifdef MOMENT
			for (int k = 0; k < 3; k++){
				mp->bgposture[_to][j][k] = mp->bgposture[_from][j][k];
			}
#endif
		}
	}
#ifdef RECORD
	fprintf(fp6, "mp2,%f,%f,%f,%f\n", mp->weight, tmp[0], tmp[1], tmp[2]);
#endif
}

FLOAT ign[3] = { 0, 0, -1000000 };

void base_get_gp(Base *base, FLOAT *j, int _from, int _to)
{
  FLOAT weight = 0;
  Branch *b;
#ifdef RECORD
  if ((fp6 = fopen("C:\\Users\\kenji\\Documents\\gp_calc_log.csv", "w")) == NULL){
	  printf("file open error\n");
	  exit(1);
  }
#endif

  vct_set(0, 0, 0, base->zmp[_to]);
  b = base->blist;
  while(b){
	  //get_gp_scan_mp(b->mphead, j);
	  get_gp_scan_mp1(b->mphead, 0, AXIS_X, ign, j, _from, _to); // 角度、軸はダミー
	  get_gp_scan_mp2(b->mphead, base->zmp[_to], &weight, _from ,_to);
    b = b->next;
  }
  vct_mul(1 / weight, base->zmp[_to]);
#ifdef RECORD
  fclose(fp6);
#endif
}

void get_zmp_scan_mp1(Base *base, massPoint *mp, FLOAT angle, int axis, FLOAT *ct, FLOAT *j, FLOAT dt, FLOAT rdtt, int load, int i)
{
  if (ct[2] < -999999){
    if (mp->jid >= 0){  //angを確定する Cf.mp->jid == -1の場合は予めbiarticular_dk()にてmp->angを設定済とみなす
      if(j){
        mp->ang = j[mp->jid];
      }else{
        mp->ang = conv_sv_f(&joint[mp->jid],joint[mp->jid].d[JOINT_POS]);
      }
    }else if(mp->jid == -2){
    	mp->ang = 0;
    }
    //////////////////////////////////////////////////
    /*if(i == REMAKE){
    	sprintf(s, "%f,", mp->ang);
    	tmcom_puts(s);
    }*/
    //////////////////////////////////////////////////
    vct_copy(mp->gp, mp->gpos[i]);
    vct_rotate(mp->ang, mp->axis, mp->gpos[i]);  //自分のangで回転
    vct_rotate(angle, axis, mp->gpos[i]);  //自分を上位からの角度で回転
#ifdef RECORD
	//fprintf(fp7, "mp,%f,%f,%d,%f,%f,%f,%f,%f,%f\n", mp->weight, mp->ang, mp->axis, mp->ct[0], mp->ct[1], mp->ct[2], mp->gp[0], mp->gp[1], mp->gp[2]);
#endif
#ifdef MOMENT
    mat_set(1, 0, 0, 0, 1, 0, 0, 0, 1, mp->gposture);   // 角運動量計算用
    mat_rotate(mp->ang, mp->axis, mp->gposture);        // 角運動量計算用
#endif
    if(load){
      vct_set(0, 0, 0, mp->jgpos);
      vct_set_E(mp->axis, mp->axis_v);
      if(base->ini_zmp[i]){
        mp->bb_q = mp->b_q = mp->ang;
      }
      mp->qd = (mp->ang - mp->b_q) / dt;
      mp->qdd = (mp->bb_q - mp->b_q * 2 + angle) * 0.5 * rdtt;    // angle => mp->angじゃないのかな？20171029
      mp->bb_q = mp->b_q;
      mp->b_q = mp->ang;
    }

    vct_add(mp->ct, mp->gpos[i]);  //コネクタを追加
  }
  else{
    vct_add(ct, mp->gpos[i]);  //上位のコネクタを追加
  }
#ifdef RECORD
  if (ct[2] < -999999){
	 // fprintf(fp7, "mp1,%f,%f,%d,%f,%f,%f,%f,%f,%f\n", mp->weight, angle, axis, mp->ct[0], mp->ct[1], mp->ct[2], mp->gpos[0], mp->gpos[1], mp->gpos[2]);
}
  else{
	 // fprintf(fp7, "mp1,%f,%f,%d,%f,%f,%f,%f,%f,%f\n", mp->weight, angle, axis, ct[0], ct[1], ct[2], mp->gpos[0], mp->gpos[1], mp->gpos[2]);
  }
#endif
#ifdef MOMENT
  mat_rotate(angle, mp->axis, mp->gposture);         // 角運動量計算用
#endif
  if(load){
    vct_add(mp->ct, mp->jgpos);
    vct_rotate(angle, mp->axis, mp->jgpos);
    vct_rotate(angle, mp->axis, mp->axis_v);
  }
  
  if (mp->child != NULL){  //子供
    if (ct[2] < -999999){
		get_zmp_scan_mp1(base, mp->child, mp->ang, mp->axis, ct, j, dt, rdtt, load, i);
		get_zmp_scan_mp1(base, mp->child, angle, axis, mp->ct, j, dt, rdtt, load, i);
	}
	else{
		get_zmp_scan_mp1(base, mp->child, angle, axis, ct, j, dt, rdtt, load, i);
	}
  }
  if (mp->next != NULL){   //next(兄弟)
	  get_zmp_scan_mp1(base, mp->next, angle, axis, ct, j, dt, rdtt, load, i);
  }
}

#ifdef MOMENT
//角運動量
FLOAT rm[3][3]; // 質点の角速度
FLOAT Ip[3][3];  // 現在姿勢での慣性テンソル
FLOAT omega_p[3];  //角速度ωp
FLOAT Lp[3], dLp[3];
FLOAT thita, a;
FLOAT gPos[3];   // 角運動量計算用の質点の座標
#endif

void get_zmp_scan_mp2(Base *base, massPoint *mp, FLOAT *gppos, FLOAT angle, FLOAT *pxc, FLOAT *pyc, FLOAT *pp, FLOAT dt, FLOAT rdtt, int load, int i)
{

  if (mp->child != NULL){  //childには自分の角度を渡す
    get_zmp_scan_mp2(base, mp->child, gppos, angle, pxc, pyc, pp, dt, rdtt, load, i);   // 自分の角度を渡すなら　angle => mp->angじゃない？
  }
  if (mp->next != NULL){   //next(兄弟)には上位から来た角度を渡す
    get_zmp_scan_mp2(base, mp->next, gppos, angle, pxc, pyc, pp, dt, rdtt, load, i);
  }

  vct_rotate(angle, AXIS_Z, mp->gpos[i]);
  vct_add(gppos, mp->gpos[i]);
#ifdef RECORD
  fprintf(fp7, "%f,%f,%f,", mp->gpos[0], mp->gpos[1], mp->gpos[2]);
#endif
#ifdef MOMENT
  mat_rotate(angle, AXIS_Z, mp->gposture);         // 角運動量計算用
#endif
  if(base->ini_zmp[i]){
    vct_copy(mp->gpos[i], mp->bgpos[i]);
    vct_copy(mp->gpos[i], mp->bbgpos[i]);
#ifdef MOMENT
    mat_copy(mp->gposture, mp->bgposture);          // 角運動量計算用
    vct_set(0, 0, 0, mp->bLp);                      // 角運動量計算用
#endif
  }
  if(load){
    vct_rotate(angle, AXIS_Z, mp->jgpos);
    vct_add(gppos, mp->jgpos);
    vct_rotate(angle, AXIS_Z, mp->axis_v);
  }else{
    vct_copy(mp->bgpos[i], mp->pdd[i]);
    vct_mul(-2, mp->pdd[i]);
    vct_add(mp->gpos[i], mp->pdd[i]);
    vct_add(mp->bbgpos[i], mp->pdd[i]);
    vct_mul(0.5 * rdtt, mp->pdd[i]);   // a = (v2 - v1)/2*dt^2
  }
#ifdef MOMENT
  //角運動量計算
  mat_copy(mp->bgposture, rm);
  mat_trans(rm);                 // bgposture[][]T
  mat_mat(mp->gposture, rm);    // wmp[][] = gposture[][] * bgposture[][]T

  mat_copy(mp->gposture, Ip);
  mat_trans(Ip);                // gposture[][]T
  mat_mat(mp->I, Ip);          // I[][] * gposture[][]T
  mat_mat(mp->gposture, Ip);   // Ip[][] = gposture[][] * I[][] * gposture[][]T
  if(rm[0][0] + rm[1][1] + rm[2][2] > 3.0){
#ifdef TEST
    printf("over 3.0\n");
#endif
    thita = 0;
  }else{
    thita = acos((rm[0][0] + rm[1][1] + rm[2][2] - 1) / 2);
  }
  if(isnan(thita)){
    return 2;
  }
  a = (thita == 0) ? 0 : thita / (2 * sinf(thita));
  vct_set(a * (rm[2][1] - rm[1][2]) / dt, a * (rm[0][2] - rm[2][0]) / dt, a * (rm[1][0] - rm[0][1]) / dt, omega_p);  //回転行列Rから角速度ベクトルomega_pを求める
  //mat_vct(Ip, omega_p, Lp);  // Lp[] = Ip[][] * omega_p[]
  ///////////角運動量を、質点×質点の位置^2 ×ω　としたときの計算。
  ///////////質点の位置は、重心点の位置からとすべきだが、簡単にするため、原点からとしている。
  vct_copy(mp->gpos, gPos);
  gPos[X] = mp->weight * gPos[X] * gPos[X];
  gPos[Y] = mp->weight * gPos[Y] * gPos[Y];
  gPos[Z] = mp->weight * gPos[Z] * gPos[Z];       // gPos[] = mp->weight * (mp->gpos)^2
  vct_cross(gPos, omega_p, Lp);                   // Lp[] = gPos[] * omega_p[] (cross)
  ///////////ここまで
  vct_copy(mp->bLp, dLp);
  vct_sub(Lp, dLp);            // dLp[] = Lp[] - mp->bLp[]
  vct_copy(Lp, mp->bLp);
  mat_copy(mp->gposture, mp->bgposture);
#endif
  if (!load){
#ifdef MOMENT
    *pxc += mp->weight * ((mp->pdd[i][Z] + ga) * mp->bgpos[i][X] - (mp->bgpos[i][Z] - base->zmp[i][Z]) * mp->pdd[i][X]) - dLp[Y];
    *pyc += mp->weight * ((mp->pdd[i][Z] + ga) * mp->bgpos[i][Y] - (mp->bgpos[i][Z] - base->zmp[i][Z]) * mp->pdd[i][Y]) + dLp[X];
#else
    *pxc += mp->weight * ((mp->pdd[i][Z] + ga) * mp->bgpos[i][X] - (mp->bgpos[i][Z] - base->zmp[i][Z]) * mp->pdd[i][X]);
    *pyc += mp->weight * ((mp->pdd[i][Z] + ga) * mp->bgpos[i][Y] - (mp->bgpos[i][Z] - base->zmp[i][Z]) * mp->pdd[i][Y]);
#endif
    *pp += mp->weight * (mp->pdd[i][Z] + ga);
    vct_copy(mp->bgpos[i], mp->bbgpos[i]);
    vct_copy(mp->gpos[i], mp->bgpos[i]);
  }
}

int base_get_zmp(Base *base, FLOAT *gppos, FLOAT angle, FLOAT *j, FLOAT dt, int load, int idx)
{
  //gppos: ﾜｰﾙﾄﾞ座標上でのロボット原点の座標
  //angle: ﾜｰﾙﾄﾞ座標上でのロボットのZ軸角度
  //jang: 関節角度列
  //Branch *b;
  //FLOAT pxc, pyc, pp;
  FLOAT rdtt = 1 / (dt * dt);


  base->pxc[idx] = 0;
  base->pyc[idx] = 0;
  base->pp[idx] = 0;

#ifdef RECORD
  if ((fp7 = fopen("C:\\Users\\kenji\\Documents\\zmp_calc_log.csv", "a")) == NULL){
	  printf("file open error\n");
	  exit(1);
  }
#endif

  vct_set(0, 0, 0, base->zmp[idx]);
  base->b[idx] = base->blist;
  while(base->b[idx]){   // 質点枝のリストループ
    get_zmp_scan_mp1(base, base->b[idx]->mphead, 0, AXIS_X, ign, j, dt, rdtt, load, idx); // 角度、軸はダミー
      // 胴体のコネクタ位置とグローバル座標に対するロボットの姿勢を計算するループ
    get_zmp_scan_mp2(base, base->b[idx]->mphead, gppos, angle, &base->pxc[idx], &base->pyc[idx], &base->pp[idx], dt, rdtt, load, idx);
    base->b[idx] = base->b[idx]->next;
  }
  if(!load){
    base->zmp[idx][X] = base->pxc[idx] / base->pp[idx];
    base->zmp[idx][Y] = base->pyc[idx] / base->pp[idx];
  }

  if(base->ini_zmp[idx])
    base->ini_zmp[idx] = 0;
#ifdef RECORD
  fprintf(fp7, "\n");
  fclose(fp7);
#endif
  return 1;
}
/*
  base_get_zmp( )のメモ
  軸座標と軸姿勢、軸の速度、軸の加速度についてはload=1の時だけ計算する。
  逆にload=1の際にはzmpは計算しない。なぜならzmpは目標との差分を計算するためのものなので負荷計算時には必要ない。
  */

void base_get_load(Base *base)
{
	int i = 0;
  Branch *b, *b2;
  massPoint *mp;
  FLOAT v0_[3], w_[3], wd_[3], tmp[3], tmp2[3];
  FLOAT wwi[3][3], v0wi[3][3];
  //空間速度、空間加速度の算出
  FLOAT v0[3], w[3]; //胴体の空間速度
  FLOAT v0d[3], wd[3];  //胴体の空間加速度
  //fix肢の末端（接地側）から計算
  b = base->blist;
  while(b){
    if(b->df > 0){
      vct_set(0, 0, 0, b->v0);
      vct_set(0, 0, 0, b->w);
      vct_set(0, 0, 0, b->v0d);
      vct_set(0, 0, 0, b->wd);
      mp = b->mphead;
      while(mp){
        //if(mp->joint){
          // qd, qddは肢先端を末端とした角度扱いとなっているため符号を逆転させる。
          // 末端側のww(wのwedge), v0w(v0のwedge)が必要なので、ここで計算しておく。
          mat_wedge(b->w, wwi);
          mat_wedge(b->v0, v0wi);

          //空間速度を求める。
          vct_mul2(mp->qd * -1, mp->axis_v, w_);  //(6.27) w
          vct_cross(mp->jgpos, w_, v0_);  //(6.28) v0
          
          vct_add(v0_, b->v0);
          vct_copy(b->v0, mp->v0);
          vct_add(w_, b->w);
          vct_copy(b->w, mp->w);

          //空間加速度を求める。
          //(6.31)を展開した式
          // ξd_j = | v0d, wd |~T
          // ξd_j = ξd_i + sdd_j･qd_j + s_j･qdd_j
          // | ww_i･(p_j x w_j) + v0w_i･w_j | + | p_j x a_j･qdd_j |
          // |           ww_i･w_j           | + |    a_j･qdd_j    |
          vct_add(mat_vct(wwi, vct_cross(mp->jgpos, b->w, tmp), tmp2), b->v0d);
          vct_add(mat_vct(v0wi, b->w, tmp), b->v0d);
          vct_add(vct_cross(mp->jgpos, vct_mul2(mp->qdd * -1, mp->axis_v, wd_), tmp), b->v0d);
          vct_copy(b->v0d, mp->v0d);

          vct_add(mat_vct(wwi, b->w, tmp), b->wd);
          vct_add(wd_, b->wd);
          vct_copy(b->wd, mp->wd);
        //}
        mp = mp->next;
      }
      vct_copy(b->v0, v0);
      vct_copy(b->w, w);
      vct_copy(b->v0d, v0d);
      vct_copy(b->wd, wd);
    }
    b = b->next;
  }
  //free肢を根元（ﾌﾘｰ側）から計算
  b = base->blist;
  while(b){
    if(b->df == 0){
      vct_copy(v0, b->v0);
      vct_copy(w, b->w);
      vct_copy(v0d, b->v0d);
      vct_copy(wd, b->wd);
      //mp = b->mptail;  とりあえず
      while(mp){
          // qd, qddは肢先端を末端とした角度扱いとなっているため符号はそのまま。
          mat_wedge(b->w, wwi);
          mat_wedge(b->v0, v0wi);
          
          //空間速度を求める。
          vct_mul2(mp->qd, mp->axis_v, w_);  //(6.27) w
          vct_cross(mp->jgpos, w_, v0_);  //(6.28) v0
          
          vct_add(v0_, b->v0);
          vct_copy(b->v0, mp->v0);
          vct_add(w_, b->w);
          vct_copy(b->w, mp->w);
          
          vct_add(mat_vct(wwi, vct_cross(mp->jgpos, b->w, tmp), tmp2), b->v0d);
          vct_add(mat_vct(v0wi, b->w, tmp), b->v0d);
          vct_add(vct_cross(mp->jgpos, vct_mul2(mp->qdd, mp->axis_v, wd_), tmp), b->v0d);
          vct_copy(b->v0d, mp->v0d);

          vct_add(mat_vct(wwi, b->w, tmp), b->wd);
          vct_add(wd_, b->wd);
          vct_copy(b->wd, mp->wd);
        //mp = mp->prev;　　とりあえず
      }
    }
    b = b->next;
  }
  //質点にかかる力とモーメントの算出
  FLOAT f[3], t[3];
  FLOAT i_f[3], i_t[3];
  FLOAT mg[3],cdd[3];
  FLOAT b_weight, b_gpos[3];

  vct_set(0, 0, 0, f);
  vct_set(0, 0, 0, t);
  
  //free肢を末端から計算
  b = base->blist;
  while(b){
    vct_set(0, 0, 0, i_f);
    vct_set(0, 0, 0, i_t);
    if(b->df == 0){
      mp = b->mphead;
      while(mp){
        vct_set(0, 0, -mp->weight * ga, mg);
        //if(mp->joint){
          //(6.20)
          vct_copy(mp->v0d, cdd);
          vct_sub(vct_cross(mp->gpos[i], mp->wd, tmp), cdd);
          vct_copy(mp->v0, tmp);
          vct_add(vct_cross(mp->w, mp->gpos[i], tmp2), tmp);
          vct_add(vct_cross(mp->w, tmp, tmp2), cdd);
          vct_add(vct_mul2(mp->weight, cdd, tmp), i_f);
          // - f~E_j
          vct_sub(mg, i_f);

          // - t~E_j
          vct_sub(vct_cross(mp->gpos[i], mg, tmp), i_t);
          //(6.22)
          vct_add(vct_mul2(mp->weight, vct_cross(mp->gpos[i], cdd, tmp2), tmp), i_t);

          vct_cross(mp->jgpos, mp->axis_v, tmp);  // p_j x a_j
          mp->u = vct_vct(tmp, i_f) + vct_vct(mp->axis_v, i_t);  //(6.37)
        //}else{
        //  vct_sub(mg, i_f);
        //  vct_sub(vct_cross(mp->gpos, mg, tmp), i_t);
        //  mp->u = vct_vct(tmp, i_f) + vct_vct(mp->axis_v, i_t);
        //}
        mp = mp->next;
      }
    }
    vct_add(i_f, f);
    vct_add(i_t, t);
    
    b = b->next;
  }
  //fix肢を根元から計算
  b = base->blist;
  while(b){
    if(b->df > 0){
      // 負荷分配率でi_f,i_tを分配しなければならない。
      vct_copy(vct_mul2(b->df, f, tmp), i_f);   // i_f = f * b->df
      vct_copy(vct_mul2(b->df, t, tmp), i_t);   // i_t = t * b->df
      //mp = b->mptail;   とりあえず
      // 脚ではない質点（関節の無い質点、胴体など）による力とトルクをfix足の根元関節で計算する。
      // 脚ではない質点はmp.size = 1が前提。
      ////FLOAT u = 0;   // 脚ではない質点によるトルク（累計）
      b2 = base->blist;
      while(b2){
        if(b2->df < 0){
          vct_set(0, 0, -b2->mphead->weight * ga, mg);
          //if(mp->joint){
            vct_copy(mp->v0d, cdd);
            vct_sub(vct_cross(b2->mphead->gpos[i], mp->wd, tmp), cdd);
            vct_copy(mp->v0, tmp);
            vct_add(vct_cross(mp->w, b2->mphead->gpos[i], tmp2), tmp);
            vct_add(vct_cross(mp->w, tmp, tmp2), cdd);
            vct_add(vct_mul2(b2->mphead->weight, cdd, tmp), i_f);
            
            vct_sub(mg, i_f);
            
            vct_sub(vct_cross(b2->mphead->gpos[i], mg, tmp), i_t);
            vct_add(vct_mul2(b2->mphead->weight, vct_cross(b2->mphead->gpos[i], cdd, tmp2), tmp), i_t);

            ////vct_cross(b2->mphead->jgpos, b2->mphead->axis_v, tmp);
            ////u += vct_vct(tmp, i_f) + vct_vct(b2->mphead->axis_v, i_t);
          //}else{
          //  vct_sub(mg, i_f);
         //   vct_sub(vct_cross(b2->mphead->gpos, mg, tmp), i_t);
         //   mp->u = vct_vct(tmp, i_f) + vct_vct(b2->mphead->axis_v, i_t);
          //}
        }
        b2 = b2->next;
      }
      vct_cross(mp->jgpos, mp->axis_v, tmp);
      ////mp->u = u + vct_vct(tmp, i_f) + vct_vct(mp->axis_v, i_t);
      mp->u = vct_vct(tmp, i_f) + vct_vct(mp->axis_v, i_t);

      // 以降、前のリンクの質量と位置を使って計算する。
      b_weight = mp->weight;
      vct_copy(mp->gpos[i], b_gpos);
      //mp = mp->prev;  とりあえず
      while(mp){
        vct_set(0, 0, -b_weight * ga, mg);
        //if(mp->joint){
          vct_copy(mp->v0d, cdd);
          vct_sub(vct_cross(b_gpos, mp->wd, tmp), cdd);
          vct_copy(mp->v0, tmp);
          vct_add(vct_cross(mp->w, b_gpos, tmp2), tmp);
          vct_add(vct_cross(mp->w, tmp, tmp2), cdd);
          vct_add(vct_mul2(b_weight, cdd, tmp), i_f);

          vct_sub(mg, i_f);

          vct_sub(vct_cross(b_gpos, mg, tmp), i_t);
          vct_add(vct_mul2(b_weight, vct_cross(b_gpos, cdd, tmp2), tmp), i_t);
          vct_cross(mp->jgpos, mp->axis_v, tmp);
          mp->u = vct_vct(tmp, i_f) + vct_vct(mp->axis_v, i_t);
        //}else{
        //  vct_sub(mg, i_f);
        //  vct_sub(vct_cross(b_gpos, mg, tmp), i_t);
        //  mp->u = vct_vct(tmp, i_f) + vct_vct(mp->axis_v, i_t);
        //}
        b_weight = mp->weight;
        vct_copy(mp->gpos[i], b_gpos);
        //mp = mp->prev;   とりあえず
      }
    }
    b = b->next;
  }
}

void
base_make_bb_lbp(FLOAT gt[], FLOAT bzmp[][3], FLOAT bangle[], int bpivot[], FLOAT bgt[][3], Lbp blbp[])
{
  // FLOAT gt[2] : 引継ぎモーションの、第一フレームの重心点座標
  // FLOAT bzmp[3][2] : 前々・前・引継ぎﾌﾚｰﾑのZMP座標
  // FLOAT bangle[3]  : 前々・前・引継ぎﾌﾚｰﾑのロボットの向き（角度）
  // int bpivot[3]    : 前々・前・引継ぎﾌﾚｰﾑの支持脚
  // FLOAT bgt[2][3]  : 前々・前ﾌﾚｰﾑのロボット原点座標（返り値）
  // Lbp lbp[3]       : 前々・前・引継ぎﾌﾚｰﾑのロボット姿勢（返り値）
  //引継ぎモーションの座標原点は引継ぎフレームのZMP座標(bzmp[2][0-2])とロボット方位(bangle[2])なので、
  //引継ぎモーション第一フレームの重心点ｵﾌｾｯﾄは、gt[0-1]そのままとなる。
  //前回モーションの引継ぎフレームのロボット姿勢から引継ぎモーションの座標系での重心位置を求め、
  //引継ぎモーション第一フレームの重心点座標との差を求める。
  //引継ぎ前モーションの重心点座標及び引継ぎ前々モーションの重心点座標を引き継ぎモーション座標系で表現した上で、
  //先に求めた差を加えて、引継ぎモーションの前フレーム、前々フレームの重心点座標とする。
  FLOAT df[2], ro[2];
  
  //前回モーションの引継ぎフレームの重心点座標を求める。
  zmp_get_robot_origin(&blbp[2], bpivot[2], ro);  // blbp[2]の論理原点を求める:ro
  ro[X] += blbp[2].o[X];
  ro[Y] += blbp[2].o[Y];
  //引継ぎモーション第一フレームの重心点座標との差を求める。
  df[X] = gt[X] - ro[X];
  df[Y] = gt[Y] - ro[Y];
  //前々フレームのロボット姿勢構築
    //引継ぎモーション座標系に合わせて回転
  lbp_rot(bangle[2], &blbp[0]);
    //引継ぎモーションに合わせた重心オフセットを求める。
  blbp[0].o[X] += df[X];
  blbp[0].o[Y] += df[Y];
  //前々フレームの重心点座標を求める。
  zmp_get_robot_origin(&blbp[0], bpivot[0], bgt[0]);
    //bzmp[0]を引き継ぎモーション座標系に変換
  bzmp[0][X] -= bzmp[2][X];
  bzmp[0][Y] -= bzmp[2][Y];
  rotate_2d_p(-bangle[2], bzmp[0]);
    //bgt[0]を求める。
  bgt[0][X] += blbp[0].o[X] + bzmp[0][X];
  bgt[0][Y] += blbp[0].o[Y] + bzmp[0][Y];
  //前フレームのロボット姿勢構築
    //引継ぎモーション座標系に合わせて回転
  lbp_rot(bangle[2], &blbp[1]);
    //引継ぎモーションに合わせた重心オフセットを求める。
  blbp[1].o[X] += df[X];
  blbp[1].o[Y] += df[Y];
  //前フレームの重心点座標を求める。
  zmp_get_robot_origin(&blbp[1], bpivot[1], bgt[1]);
    //bzmp[1]を引き継ぎモーション座標系に変換
  bzmp[1][X] -= bzmp[2][X];
  bzmp[1][Y] -= bzmp[2][Y];
  rotate_2d_p(-bangle[2], bzmp[1]);
    //bgt[1]を求める。
  bgt[1][X] += blbp[1].o[X] + bzmp[1][X];
  bgt[1][Y] += blbp[1].o[Y] + bzmp[1][Y];
}

void
adj_mercury(Base *base)
{
  Branch *b;
  massPoint *mp;
  FLOAT j_u[16];
  int i;

  for(i = 0; i < 16; i++){
    j_u[i] = 0.0;
  }
  b = base->blist;
  while(b){
    mp = b->mphead;
    while(mp){
      if(mp->jid < 16 && mp->jid > -1){
        j_u[mp->jid] += mp->u;
      }
      mp = mp->next;
    }
    b = b->next;
  }
  j_u[3] += j_u[4];
  j_u[4] = j_u[3] * 0.5;
  j_u[3] = -j_u[4];
  j_u[6] += j_u[5];
  j_u[5] = j_u[6] * 0.5;
  j_u[6] = -j_u[5];

  j_u[11] += j_u[12];
  j_u[12] = j_u[11] * 0.5;
  j_u[11] = -j_u[12];
  j_u[14] += j_u[13];
  j_u[13] = j_u[14] * 0.5;
  j_u[14] = -j_u[13];

  j_u[1] += j_u[2];
  j_u[2] = -j_u[1] * 0.5;
  j_u[1] = j_u[2];
  j_u[9] += j_u[10];
  j_u[10] = -j_u[9] * 0.5;
  j_u[9] = j_u[10];

  b = base->blist;
  while(b){
    mp = b->mphead;
    while(mp){
      if(mp->jid < 16 && mp->jid > -1){
        mp->u = j_u[mp->jid];
      }
      mp = mp->next;
    }
    b = b->next;
  }

}

FLOAT adj_joint_angle(FLOAT *t, FLOAT *p1, FLOAT *p2)
{
  FLOAT a = *t * *p1;
  a *= a;
  a *= a * *p2;
//  if(a > 0.1) a = 0.1;
  a *= *t < 0 ? -1 : 1;
  return a;
}

FLOAT noise_reduction(Node14 *np, int id)
{
  //first
  Node14 *n = np;
  FLOAT max = -100, min = 100, sum = 0;
  int i = 0;
  while(n && i < 3){
    if(max < n->d[id]) max = n->d[id];
    if(min > n->d[id]) min = n->d[id];
    sum += n->d[id];
    n = n->next;
    i++;
  }
  return (sum - max - min);
}

FLOAT motor_speed(FLOAT t)
{
  static FLOAT w0 = 5.511566; // rad/s
  static FLOAT t0 = 3.998e9; // Nm / 10^9

  return ( -w0 * fabs(t) / t0 + w0);
}


