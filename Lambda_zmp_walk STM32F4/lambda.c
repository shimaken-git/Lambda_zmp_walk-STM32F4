/***********************************************/
/*   File        :lambda.c                     */
/*   Version     :v1.0.2                       */
/*   Date        :2017/02/07                   */
/*   Author      :Kenji Shimada                */
/*   動力学フィルターに対応                    */
/***********************************************/

#define _USE_MATH_DEFINES
#include <stdlib.h>
#include <math.h>

#include "lambda.h"
#include "vector.h"
#ifdef USE_ZMP_ONLINE
#include "multi_mass_point_model.h"
#else
#include "lbposture.h"
#endif

void tmcom_puts(char *send_data);
void tmcom_outval(long int val);
void tmcom_outfloat(float val);

//#define RECORD

#ifdef RECORD
FILE *fp9;
#endif


char *func_mess_name[] = {
  "",
  "servo_cntl()\r\n",
  "servo_ack_receive()\r\n",
  "servo_ack_receive_end()\r\n",
  "uart1_send()\r\n",
  "uart1_receive()\r\n",
  "uart1_receive_reset()\r\n",
  "uart1_receive_abort()\r\n",
  "TIM2_IRQHandler()\r\n",
  "TIM3_IRQHandler()\r\n",
  "TIM4_IRQHandler()\r\n",
  "servo_ack_receive_success\r\n",
  "servo_ack_receive_false\r\n",
};

char *param_name_i[] = {
		"_n_step",
		"_step",
		"_delay",
		"_innerfall_timing",
		"_innerfall_threshold",
		"_d_step",
		"_cp_space_threshold",
		"_gyro_select",
		"_lp_select",
		"_lp_mag",
		"_gyro_feedback_debug1",
		"_gyro_feedback_debug2",
		"_arm_swing_f_1",
		"_arm_swing_f_2",
		"_arm_swing_f_3",
		"_arm_swing_f_4",
		"_arm_swing_b_1",
		"_arm_swing_b_2",
		"_arm_swing_b_3",
		"_arm_swing_b_4",
		"_walk_follow_threshold_times",
		"_upboad_mode",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
		"",
};

char *param_name_f[] = {
		"_shift_zmp_rx",
		"_shift_zmp_lx",
		"_shift_zmp_ry",
		"_shift_zmo_ly",
		"_default_lbp_width",
		"_default_lbp_depth",
		"_default_lbp_height",
		"_default_lbp_hung",
		"_default_ubp_1",
		"_default_ubp_2",
		"_default_ubp_3",
		"_default_ubp_4",
		"_pace",
		"_hung",
		"_faststep_arange",
		"_gtz_adjust",
		"_gtz_adjust_reccati",
		"_height_adjust",
		"_innerfall_reaction",
		"_ankle_pitch_calibration_amount",
		"_ankle_roll_right_calibration_amount",
		"_ankle_roll_left_calibration_amount",
		"_com_calibration_amount",
		"_zmp_align_amount",
		"_gyro_roll_feedback_retio",
		"_gyro_pitch_feedback_retio",
		"_pos_pitch_calibration_retio",
		"_xzmp_right_roll_calibration_retio",
		"_xzmp_left_roll_calibration_retio",
		"_zmp_pitch_align_retio",
		"_pid_y_offset_rate_gyro",
		"_pid_y_offset_rate_zmp",
		"_gyro_angle_offset_roll",
		"_gyro_angle_offset_pitch",
		"_pid0_p_fact",
		"_pid0_i_fact",
		"_pid0_d_fact",
		"_pid0_u_max",
		"_pid1_p_fact",
		"_pid1_i_fact",
		"_pid1_d_fact",
		"_pid1_u_max",
		"_pid2_p_fact",
		"_pid2_i_fact",
		"_pid2_d_fact",
		"_pid2_u_max",
		"_pid3_p_fact",  //for pos calibration
		"_pid3_i_fact",
		"_pid3_d_fact",
		"_pid3_u_max",
		"_pid4_p_fact",  //for pos calibration
		"_pid4_i_fact",
		"_pid4_d_fact",
		"_pid4_u_max",
		"_pid5_p_fact",  //for com calibration
		"_pid5_i_fact",
		"_pid5_d_fact",
		"_pid5_u_max",
		"_pid6_p_fact",  //for zmp align calibration
		"_pid6_i_fact",
		"_pid6_d_fact",
		"_pid6_u_max",
		"_pid7_p_fact",  //for ankle right roll calibration
		"_pid7_i_fact",
		"_pid7_d_fact",
		"_pid7_u_max",
		"_pid8_p_fact",  //for ankle left roll calibration
		"_pid8_i_fact",
		"_pid8_d_fact",
		"_pid8_u_max",
		"_up_down_action_accel",
		"_walk_follow_max",
		"_walk_follow_threshold",
		"_criteria_of_standup",
		"_criteria_of_stable",
		"_criteria_of_motion_posture",
		"",
		"",
		"",
		"",
};

Joint joint[SV_VOL];                     // 関節=servo
int sv_rev_index[SVID_MAX + 1];          // servo id 逆引きインデックス

//sensor
int s_idx[3] = { SX, SY, SZ };
int s_sgn[3] = { SXS, SYS, SZS};

#ifdef WALK
WalkParaSet wps;
#endif

SensorUniqueData sud;

const FLOAT gbx = 62.5;  // ロボット原点を原点とした時の、J1関節原点の位置
const FLOAT pitch, roll, yaw;  // ロボットの姿勢　　⇒構造体にした方がいいかも。

const FLOAT H_PI = M_PI * 0.5;

// IK計算用定数
const FLOAT l1 = 120.0;  // 大腿、下腿の長さ
const FLOAT l3 = 37.5;  // 下腿リンク接続位置
const FLOAT lnx1 = -22.5;
const FLOAT lny1 = 60.0;
const FLOAT lnx2 = -60.0;
const FLOAT lny2 = -56.0;
// 20170401
const FLOAT nf = M_PI * 0.5 - 10.042 * M_PI / 180.0;  // 膝リンクのオフセット
// 20170401
//第一リンク定数
#define L11 30.0
#define L12 50.0
#define L13 40.0
#define L14 170.0
const FLOAT l11 = L11;  //
const FLOAT l12 = L12;  //
const FLOAT l13 = L13;  //
const FLOAT l14 = L14;  //
const FLOAT l11_2 = L11 * L11;
const FLOAT l12_2 = L12 * L12;
const FLOAT l14_2 = L14 * L14; //
#define A1 70.0
const FLOAT a1 = A1;  //
const FLOAT a1_2 = A1 * A1;
const FLOAT sx1 = 19.0;  //
const FLOAT sy1 = 8.5;  //
#ifdef GOPRO
const FLOAT a1o = 2.200320864;
#endif
//第二リンク定数
#define L21 37.0
#define L22 45.0
#define L23 35.0
#define L24 140.0

const FLOAT l21 = L21;  //
const FLOAT l22 = L22;  //
const FLOAT l23 = L23;  //
const FLOAT l24 = L24;  //
const FLOAT l21_2 = L21 * L21;
const FLOAT l22_2 = L22 * L22;
const FLOAT l23_2 = L23 * L23;
const FLOAT l24_2 = L24 * L24;
#define A2 70.0
const FLOAT a2 = A2;  //
const FLOAT a2_2 = A2 * A2;
const FLOAT sx2 = -19.0;  //
const FLOAT sy2 = 8.5;  //
const FLOAT an2 = -M_PI / 6.0;
#ifdef GOPRO
const FLOAT a2o = 0.999525491;
#endif

//股関節リンク定数
/*
FLOAT kl1 = 30.0;
FLOAT kl2 = 40.0;
FLOAT kl3 = 53.381;  // =SQRT(42.5^2+32.3^2)
FLOAT px = 22.5;
FLOAT py = 22.5;
FLOAT koffset = -58.07538295*M_PI/180;
FLOAT kroffset; // = atan2f(-42.5, 32.3);
*/

#define HCONST_KL1      0
#define HCONST_KL2      1
#define HCONST_KL3      2
#define HCONST_PX       3
#define HCONST_PY       4
#define HCONST_KOFFSET  5
#define HCONST_KROFFSET 6

FLOAT hconst[2][7] = { {18, 40, 53.381, 43.0, 0, 0, 0},       // 20170429 構造変更反映
                       {18, 40, 53.381, 43.0, 0, 0, 0}};
//FLOAT hconst[2][7] = { {18, 43, 29.1, 43.0, 0, 0, 0},       // 20170910 構造変更反映
//                       {18, 43, 29.1, 43.0, 0, 0, 0}};

//足全体での定数
const FLOAT lohx = 8.0;   // 股関節ヨー軸と股関節ロール軸のXオフセット
#define LOFX 15.9
const FLOAT lofx = LOFX;  // 股関節ロール軸と足首ロール軸のXオフセット
//FLOAT lohz = 80.5;  // 股関節ピッチ・ロールZ軸オフセット
//      ↑　lobzに統合  20170302
const FLOAT loz = 34.0;   // 足裏から足首までの高さ CAD上は32.8
const FLOAT la = 120.0;
const FLOAT ln = 120.0;
const FLOAT lofx_2 = LOFX * LOFX;

//ベースブロックの定数
const FLOAT loby = 0;  //ベースブロック原点から大腿関節までのオフセット           // 20170429 構造変更反映
//           ↑ﾏｲﾅｽだと思うが　20170302
const FLOAT lobz = -80.5;

//足部品の基準姿勢時のオフセット角度（傾き）
const FLOAT a_ele_o = -0.576447728;  //33.028deg
const FLOAT a_str_o = 2.153017009;   //123.2588deg
const FLOAT a_elnk_o = 0.364824062;  //20.9deg
const FLOAT a_slnk_o = 0.010949847;  //0.627deg

//FLOAT hw = 32.0;

const FLOAT f_sv_fact = 1697.652726;  // radianをｻｰﾎﾞ値に変換。180/π * 8000/270
//FLOAT sv_f_fact = 5.890486226e-4;
const FLOAT sv_f_fact = 0.0005890486226;
short sv_id_tbl[] =     {0, 1, 2, 3, 4, 5, 21, 22,                 6, 7, 8, 9, 10, 11, 23, 24,                13,14,15,16, 17,18,19,20, 25, 26, 27, 28, 29};
short sv_sign_tbl[] =   {1,-1,-1,-1, 1,-1,  1, -1,                 1,-1, 1, 1, -1, -1, -1,  1,                -1, 1, 1, 1,  1,-1, 1,-1,  1,  1,  1,  1,  1};
short sv_offset_tbl[] = {0, -1133, 0, 0, 889, 0, -840, -2666,         0, 1133, 0, 0,  -889,  0, 840, 2666,            0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0, 0};  //offsetはsignの影響を受ける
short sv_max_tbl[] =    {11500, 11500, 9120, 9080, 10240, 8354, 9125, 11500,  11500, 11500, 9340, 8500, 9823, 8341, 9219, 11500,  11500,11500,11500,11500, 11500,11500,11500,11500,  11500,11500,11500,11500,11500}; //物理角度（sign,offsetを考慮しない角度）
short sv_min_tbl[] =    {3500,   3500, 5970, 6800,  5700, 6304, 5263,  3500,   3500,  3500, 5980, 5970, 4651, 6239, 5497,  3500,   3500,3500,3500,3500,     3500,3500,3500,3500,      3500,3500,3500,3500,3500}; //物理角度
short sv_ics_tbl[] =     {1, 1, 0, 0, 1, 1, 1, 1,                  1, 1, 0, 0, 1, 1, 1, 1,                     1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1 ,1};

//short sv_offset_tbl[] 20170910版では股関節ROLLサーボの基準線を垂直下向きにしている。設定値の変更とともにサーボホーンの付け替えも必要。

void lambda_ik_ini(){
  // 20170429 構造変更反映　　　股関節剛性を見直した版
  hconst[IK_RIGHT][HCONST_KOFFSET] = -38.25120716*M_PI/180;
  hconst[IK_RIGHT][HCONST_KROFFSET] = atan2f(-42.5, 32.3);
  hconst[IK_LEFT][HCONST_KOFFSET] = -38.25120716*M_PI/180;
  hconst[IK_LEFT][HCONST_KROFFSET] = atan2f(-42.5, 32.3);

/*
  // 20170910 構造変更反映 	//サーボの基準線は垂直下向きだが、計算上の基準線は水平右側。なので、オフセット値は以下のようになる。
  hconst[IK_RIGHT][HCONST_KOFFSET] = -86.935*M_PI/180;
  hconst[IK_RIGHT][HCONST_KROFFSET] = atan2f(-29.0, 2.4);
  hconst[IK_LEFT][HCONST_KOFFSET] = -86.935*M_PI/180;
  hconst[IK_LEFT][HCONST_KROFFSET] = atan2f(-29.0, 2.4);
  //リンクの効率を考えて構造を変えてみたがあまり効果なく、がたつきが大きいため元に戻すことに
*/

}

/////////たわみ補償用////////////
//short adjust[2];

#ifdef WALK
//二重倒立振子関数を使った歩容生成関数
//FLOAT inverse pendulum

void calc_walk(WalkParaSet *w, FLOAT t, FLOAT *x, FLOAT *bx, FLOAT *y){

  FLOAT tg_x = sqrtf((w->x_leg_w * w->x_leg_z + w->x_body_w * w->x_body_z) / ((w->x_leg_w + w->x_body_w) * w->G));
  FLOAT t2_y = sqrtf(w->y_body_z / w->G);
  FLOAT tgg_y = sqrtf((w->y_leg_w * w->y_leg_z * w->y_leg_z + w->y_body_w * w->o_z * w->y_body_z) / ((w->y_leg_w * w->y_leg_z + w->y_body_w * w->o_z) * w->G));
  FLOAT c2_y = (2 * w->y_leg_w * w->y_leg_z + (w->y_body_w - w->y_leg_w) * w->o_z) * w->body_s / (w->y_body_w * w->o_z);
  FLOAT c1_y = w->y_leg_w * w->io * (w->o_z - w->y_leg_z) / (w->y_leg_w * w->y_leg_z * w->y_body_w * w->o_z);
  FLOAT a_y = (c1_y - w->io + w->y_body_w * w->o_z * c2_y / (w->y_leg_w * w->y_leg_z + w->y_body_w * w->o_z)) / (1 + exp(w->pace_t / tgg_y));
  FLOAT b_y = a_y * exp(w->pace_t / tgg_y);
  FLOAT emt_x = exp(-w->pace_t / tg_x);
  FLOAT et_x = exp(w->pace_t / tg_x);


  //sagittal plane 前後方向
  *y = ((w->end_y - w->start_y * emt_x) * exp(t / tg_x) + (- w->end_y + w->start_y * et_x) * exp( - t / tg_x)) / (et_x - emt_x);
  //coronal plane 左右方向
  *x = - (a_y * exp(t / tgg_y) + b_y * exp(- t / tgg_y) - c1_y - w->y_body_w * w->o_z * c2_y / (w->y_leg_w * w->y_leg_z * w->y_body_w * w->o_z));
  *bx = (- exp(t / t2_y) - exp(w->pace_t / t2_y) * exp(-t / t2_y) + exp(w->pace_t / t2_y) + 1) * c2_y / (1 + exp(w->pace_t / t2_y));
  
}

void print_walk(WalkParaSet *w){
#ifndef GOPRO
  tmcom_puts(    "x_leg_z  "); tmcom_outfloat(w->x_leg_z);
  tmcom_puts("\r\ny_leg_z  "); tmcom_outfloat(w->y_leg_z);
  tmcom_puts("\r\nx_body_z "); tmcom_outfloat(w->x_body_z);
  tmcom_puts("\r\ny_body_z "); tmcom_outfloat(w->y_body_z);
  tmcom_puts("\r\no_z      "); tmcom_outfloat(w->o_z);
  tmcom_puts("\r\nx_leg_w    "); tmcom_outfloat(w->x_leg_w);
  tmcom_puts("\r\ny_leg_w    "); tmcom_outfloat(w->y_leg_w);
  tmcom_puts("\r\nx_body_w   "); tmcom_outfloat(w->x_body_w);
  tmcom_puts("\r\ny_body_w   "); tmcom_outfloat(w->y_body_w);
  tmcom_puts("\r\n");
#endif
}

FLOAT calc_depth(WalkParaSet *w, int cycle){
  if(cycle < w->f_wait) return 0.0;
  else if(w->w_cycle - cycle < w->e_wait) return 1.0;
  else return (FLOAT)(cycle - w->f_wait) / (FLOAT)(w->w_cycle - w->f_wait - w->e_wait);
}
#endif

void joint_init(void)
{
  int sv;
  for(sv = 0; sv < SV_VOL; sv++){
    joint[sv].d[JOINT_STATUS] = SV_UNDEF;
    joint[sv].d[JOINT_CMD] = -1;
    joint[sv].d[JOINT_LPOS] = 0;
    joint[sv].d[JOINT_SVID] = sv_id_tbl[sv];
    joint[sv].d[JOINT_SIGN] = sv_sign_tbl[sv];
    joint[sv].d[JOINT_TRIM] = 0;
    joint[sv].d[JOINT_OFFSET] = sv_offset_tbl[sv];
    joint[sv].d[JOINT_MAX] = sv_max_tbl[sv];
    joint[sv].d[JOINT_MIN] = sv_min_tbl[sv];
    joint[sv].d[JOINT_ICS] = sv_ics_tbl[sv];
    joint[sv].d[JOINT_PRMCMD] = SV_PRM;
    joint[sv].d[JOINT_RTN] = -1;
    joint[sv].d[JOINT_FLG] = 0;
    joint[sv].d[JOINT_ERR] = 0;
    joint[sv].d[JOINT_CAPTURE] = 0;
  }


  //ik_init
#if 0
  l11_2 = l11 * l11;
  l12_2 = l12 * l12;
  l14_2 = l14 * l14;  // 
  a1_2 = a1 * a1;
  l21_2 = l21 * l21;
  l22_2 = l22 * l22;
  l23_2 = l23 * l23;
  l24_2 = l24 * l24;  // 
  a2_2 = a2 * a2;
  an2 = -M_PI / 6.0;
  koffset = -58.07538295 * M_PI/180;
  lohx_2 = lohx * lohx;
  kroffset = atan2f(-42.5, 32.3);
#endif

  //sonsor data init
  int i;
  for(i = 0; i < 8; i++){
    sud.lp_adc_max[i] = 3190;
    sud.lp_adc_min[i] = 1190;
    sud.gyro[i] = 0;
    sud.gravity[i] = 0;
  }

#ifdef WALK
  //walk para set init
  wps.x_leg_z = 160.1;
  wps.y_leg_z = 222.49;
  wps.x_body_z = 292.38;
  wps.y_body_z = 336.65;
  wps.x_leg_w = 274.11;
  wps.y_leg_w = 702.718;
  wps.x_body_w = 1669.0;
  wps.y_body_w = 1235.58;
  wps.o_z = 300;
  wps.G = 9800;

  wps.io = 60;
  wps.body_s = 0;
  wps.start_y = -40;
  wps.end_y = 40;
  wps.stride = 80;
  wps.pace_t = 0.4;
  wps.f_wait = 2;
  wps.e_wait = 2;
  wps.w_cycle = 40;
#endif
}


void
cnv_servo_pos(char h, char l, short *pos)
{
  *pos = (h & 0x7f) << 7;
  *pos = *pos | (l & 0x7f);
}

void
cnv_pos_servo(short pos, char *h, char *l)
{
  if(pos < 0) pos = 0;
  if(pos > 0x3fff) pos = 0x3fff;
  *l = pos & 0x007f;
  *h = pos >> 7;
}

void make_servo_target(int sv_id, int target, char *senddata)
{
  char h, l;
  cnv_pos_servo(target, &h, &l);
  senddata[0] = (sv_id & 0x1f) | 0x80;
  senddata[1] = h;
  senddata[2] = l;
}

void make_servo_setting(int sv_id, char sc, char data, char *senddata)
{
  senddata[0] = (sv_id & 0x1f) | 0xc0;
  senddata[1] = sc;
  senddata[2] = data;
}

void make_servo_read_para(int sv_id, char sc, char *senddata)
{
  senddata[0] = (sv_id & 0x1f) | 0xa0;
  senddata[1] = sc;
}

void cnv_rom_int2(char r1, char r2, int *data)
{
  *data = (r1 << 4) | (r2 & 0x0f);
}

void cnv_int_rom2(int data, char *r1, char *r2)
{
  data &= 0xff;
  *r2 = data & 0x0f;
  *r1 = data >> 4;
}

void cnv_rom_int4(char r1, char r2, char r3, char r4, int *data)
{
  *data = (r1 << 12) | (r2 << 8) | (r3 << 4) | (r4 & 0x0f);
}

void cnv_int_rom4(int data, char *r1, char *r2, char *r3, char *r4)
{
  data &= 0xffff;
  *r4 = data & 0x0f;
  *r3 = (data >> 4) & 0x0f;
  *r2 = (data >> 8) & 0x0f;
  *r1 = data >> 12;
}

char sum_data(int n, char *data)
{
  int i;
  char s = 0;
  for(i = 2; i < n; i++){
    s ^= data[i];
  }
  return s;
}

void w2b(int wd, char *cd)
{
  union word2byte c;
  c.wd = wd;
  cd[0] = c.cd[0];
  cd[1] = c.cd[1];
}

FLOAT hip_joint_ik(FLOAT hip_roll_angle, int side)
{
  FLOAT tmx1, tmy1, tml1, tma1;
  FLOAT tmx2, tmy2;
  //char s[100];
  //股関節roll (左足用）
  hip_roll_angle *= (side == IK_RIGHT ? -1 : 1);
  tmx1 = hconst[side][HCONST_KL3] * cosf(hip_roll_angle + hconst[side][HCONST_KROFFSET]) - hconst[side][HCONST_PX];
  tmy1 = hconst[side][HCONST_KL3] * sinf(hip_roll_angle + hconst[side][HCONST_KROFFSET]) - hconst[side][HCONST_PY];
//  sprintf(s, "hip_roll_angle:%f tmx1:%f tmy1:%f\r\n", hip_roll_angle,tmx1, tmy1);
//  tmcom_puts(s);
  if(crosspoint(tmx1, tmy1, hconst[side][HCONST_KL1], hconst[side][HCONST_KL2], &tmx2, &tmy2, &tml1, &tma1, 1)){    // modeを1にすればうまくいくが。。。20170503
    return -100; //交点なし=エラー
  }else{
//	  sprintf(s,"tmx2:%f tmy2:%f tml1:%f tma1:%f\r\n", tmx2, tmy2, tml1, tma1);
//	  tmcom_puts(s);
    return((atan2f(tmy2, tmx2) - hconst[side][HCONST_KOFFSET]) * (side == IK_RIGHT ? -1 : 1));
  }
}


FLOAT hip_joint_dk(FLOAT hip_servo_angle, int side)
{
  FLOAT tmx1, tmy1, tml1, tma1;
  FLOAT tmx2, tmy2;
  //股関節roll (左足用）
  //サーボリンク先端座標
  hip_servo_angle *= (side == IK_RIGHT ? -1 : 1);
  tmx1 = hconst[side][HCONST_KL1] * cosf(hip_servo_angle + hconst[side][HCONST_KOFFSET]) + hconst[side][HCONST_PX];
  tmy1 = hconst[side][HCONST_KL1] * sinf(hip_servo_angle + hconst[side][HCONST_KOFFSET]) + hconst[side][HCONST_PY];
  if(crosspoint(tmx1, tmy1, hconst[side][HCONST_KL3], hconst[side][HCONST_KL2], &tmx2, &tmy2, &tml1, &tma1, 2)){  //-yモード
    return -100;  //error
  }else{
  return((atan2f(tmy2, tmx2) - hconst[side][HCONST_KROFFSET]) * (side == IK_RIGHT ? -1 : 1));
  }
}

//#define IKDEBUG

unsigned char biarticular_ik(FLOAT *ik, FLOAT *j, int either)
{
  //股関節原点の足先座標を指定。

  //暫定処置として、
  //roll, pitch は胴体姿勢角度  ⇒　未実装
  //yawは足先の姿勢角度

  // j[0] : 股関節yaw
  // j[1] : 股関節roll
  // j[2] : 第一リンク
  // j[3] : 第二リンク
  // j[4] : 足首pitch
  // j[5] : 足首roll
  // j[6] : 膝
  // j[7] : 股関節pitch

  // ik[0] : x          論理原点からの、足先の左右寸法
  // ik[1] : y          論理原点からの、足先の前後寸法
  // ik[2] : z          論理原点からの、足先の上下寸法
  // ik[3] : roll       body_roll
  // ik[4] : pitch      body_pitch
  // ik[5] : yaw        foot_yaw

  // either = 0 => right
  // either = 1 => left

  unsigned char r = 0;
  FLOAT ankle[3];
  FLOAT x, y, z;
  FLOAT tml1, tma1, tma2, tml1_2, tml2;
  FLOAT tmx1, tmy1, tmx2, tmy2;
#ifndef GOPRO
  FLOAT tma3;
  FLOAT cs1, sn1;
#endif
  FLOAT link1, link2;
  FLOAT s_lohx, s_lofx;

  vct_set(ik[LEG_X], ik[LEG_Y], ik[LEG_Z] + loz, ankle);  // 足原点から足首関節の座標
  //胴体姿勢角度を足首座標側に移動⇒ベース部姿勢を正規
#if 0
  vct_rotate(-ik[LEG_ROLL], Y, ankle);
  vct_rotate(-ik[LEG_PITCH], X, ankle);
  vct_rotate(-ik[LEG_YAW], Z, ankle);
#endif

  vct_rotate(-ik[LEG_YAW], Z, ankle);
  vct_rotate(-ik[LEG_PITCH], X, ankle);
  vct_rotate(-ik[LEG_ROLL], Y, ankle);


  if(either == IK_LEFT){ // left
    s_lohx = lohx;
    s_lofx = -lofx;
  }else{ // right
    s_lohx = -lohx;
    s_lofx = lofx;
  }
  // j[5]の計算に使うのでここに移動    20170302
  // 上位座標からの差分表現に統一（符号を反転）  20170302
  
  //foot部姿勢を確定
  FLOAT sr = sinf(ik[LEG_ROLL]), cr = cosf(ik[LEG_ROLL]);
  FLOAT sp = sinf(ik[LEG_PITCH]), cp = cosf(ik[LEG_PITCH]);
  FLOAT sy = sinf(ik[LEG_YAW]), cy = cosf(ik[LEG_YAW]);
#if 0
  //pitch_f
  //s(pf) = s(r)s(y)+c(r)s(p)c(y)
  j[4] = -asin(sr*sy + cr*sp*cy);  //←現物合わせで符号逆転　要調査検討　-asin() の部分
  //roll_f
  //{-c(pf)s(rf)} / {c(pf)c(rf)} = -tan(rf) = {-s(r)c(y)+c(r)s(p)s(y)} / {c(r)c(p)}
  FLOAT roll_f = (FLOAT)atan2f(sr*cy - cr*sp*sy , cr*cp);
  //yaw_f
  //{-s(yf)c(pf)} / {c(yf)c(pf)} = -tan(yf) = {-c(r)s(y)+s(r)s(p)c(y)} / {-s(p)}
  j[0] = (FLOAT)atan2f(cr*sy - sr*sp*cy , cp*cy);
#endif
  //pitch_f
  //-s(pf) = s(y)s(r)-c(y)s(p)c(r)
  j[4] = asin(sy*sr - cy*sp*cr);
  //roll_f
  //[s(rf)c(pf)] / [c(rf)c(pf)] = [c(y)s(r)+s(y)s(p)c(r)] / [c(p)c(r)]
  FLOAT roll_f = atan2f(cy*sr + sy*sp*cr, cp*cr);
  //yay_f
  //c(pf)s(yf) / c(pf)c(yf) = tan(rf) = [s(y)c(r)+c(y)s(p)s(r)] / [c(y)c(p)]
  j[0] = atan2f(sy*cr + cy*sp*sr, cy*cp);

  //足首ROLLを求める
  j[5] = atan2f(ankle[X] - s_lofx - s_lohx, -ankle[Z]) - roll_f;  //←現物合わせで符号逆転　要調査検討　- roll_fの部分
  //                      ↑追加    20170302
  //                    ↑       ↑符号転換20170322
  // 股関節ROLL軸からの座標に変換
  x = ankle[X] - s_lohx;
  y = ankle[Y] - loby;
  z = ankle[Z];
  tma1 = atan2f(-x, -z);   // j1d
  tml2 = x * x + z * z;  // ldd_2
  if (tml2 - lofx_2 < 0) return(0x40);
  tml1 = sqrtf(tml2 - lofx_2);  // ld
  tml2 = tml1 + lobz;          // l = ld - lobz
  tma2 = atan2f( -s_lofx, tml1);  // jd = atan(-lofx / ld)
  j[1] = tma1 - tma2;  // = j1d - jd
  tml1 = sqrtf(tml2 * tml2 + y * y);                                   //①   L = tml1
  //20171218追加
  //第7サーボ（股関節PITCH角度）
  FLOAT b = asin(y / tml1);  // asin(y / L)
  FLOAT c = acos(tml1 / ( 2 * l1));
  j[6] = b + c;
  //第8サーボ（膝サーボ角度
  if((tma1 = yogen1(tml1, l1, l1)) != -100){
	  //j[7] = tma1 - M_PI + nf;
	  j[7] = tma1 - M_PI;
  }else{
	  j[7] = tma1; // error
  }

  //第一リンク、第二リンクの長さを求める。
  tma1 = atan2f(y, tml2) + acos(tml1 * tml1 / (2 * l1 * tml1));      //②③ A+B = tma1
  tmx1 = l1 * sinf(tma1);                                            //④   膝座標 = tm*1
  tmy1 = -l1 * cosf(tma1);
  tmx2 = (y - tmx1) * l3 / l1 + tmx1;                              //⑤   リンク交点 = tm*2
  tmy2 = (-tml2 - tmy1) * l3 / l1 + tmy1;
  tmx1 = tmx2 - lnx1;
  tmy1 = tmy2 - lny1;
  link1 = sqrtf(tmx1 * tmx1 + tmy1 * tmy1);                          //⑥
  tmx1 = tmx2 - lnx2;
  tmy1 = tmy2 - lny2;
  link2 = sqrtf(tmx1 * tmx1 + tmy1 * tmy1);                          //⑥


  //第一リンク
#ifdef GOPRO
  tml1_2 = link1 * link1;
  FLOAT t_ = (a1_2 + l14_2 - tml1_2) / (2 * a1 * l14);
  if (t_ > 1) return(0x80);
  else tma1 = acos(t_);                     //ang1
  j[2] = a1o - tma1;
#else
  //a1とlink1の交点(tmx1, tmy1)を求める
  //crosspoint(link1, 0, a1, l14, &tmx1, &tmy1, &tml1, &tma1, 0);
  tml1_2 = link1 * link1;
  tmx1 = (a1_2 - l14_2 + tml1_2) / (2 * link1);                     //①   リンク交点 = tm*1
  tmy1 = sqrtf(a1_2 - tmx1 * tmx1);
  //交点(tmx1, tmy1)のリンク延長先(tmx2, tmy2)を求める
  tmx2 = (tmx1 - link1) * l13 / l14 + tmx1;                         //②   リンク延長先 = tm*2
  tmy2 = tmy1 * l13 / l14 + tmy1;
  //link1とa1の角度tma1を求める
  tma1 = acos((a1_2 + tml1_2 - l14_2) / (2 * a1 * link1));            //③   ang1 = tma1
  //サーボ原点座標をtma1で回転して正規化し、リンク延長先との交点(tmx1,tmy1)を求める
  cs1 = cosf(tma1);
  sn1 = sinf(tma1);
  tmx2 = tmx2 - (sx1 * cs1 - sy1 * sn1);                            //④   リンク延長先-サーボ中心 = tm*2
  tmy2 = tmy2 - (sx1 * sn1 + sy1 * cs1);
  tml1_2 = tmx2 * tmx2 + tmy2 * tmy2;
  tml1 = sqrtf(tml1_2);                                              //⑤
  tma2 = atan2f(tmy2, tmx2);
  tmx2 = (l11_2 - l12_2 + tml1_2) / (2 * tml1);                     //⑥   サーボ中心を原点としたL11とL12の交点 = tm*2
  tmy2 = sqrtf(l11_2 - tmx2 * tmx2);
  cs1 = cosf(tma2);
  sn1 = sinf(tma2);
  tmx1 = tmx2 * cs1 - tmy2 * sn1;                                   //⑦   サーボ中心を原点としたリンク先端の絶対角度
  tmy1 = tmx2 * sn1 + tmy2 * cs1;
  //tmx2 -= sx1 * cs1 - sy1 * sn1;                            //④   リンク延長先-サーボ中心 = tm*2
  //tmy2 -= sx1 * sn1 + sy1 * cs1;
  //crosspoint(tmx2, tmy2, l11, l12, &tmx1, &tmy1, &tml1, &tma3, 0);
  tma2 = atan2f(tmy1, tmx1);                                         //⑧
  j[2] = -(tma2 - tma1 - H_PI);                                        //⑨   サーボ角度
#endif
  //第二リンク
#ifdef GOPRO
  tml1_2 = link2 * link2;
  t_ = (a2_2 + l24_2 - tml1_2) / (2 * a2 * l24);
  if (t_ > 1) return(0x80);
  else tma1 = acos(t_);                     //ang2
  j[3] = a2o - tma1;
#else
  tml1_2 = link2 * link2;
  tmx1 = (a2_2 - l24_2 + tml1_2) / (2 * link2);                     //①   リンク交点 = tm*1
  tmy1 = -sqrtf(a2_2 - tmx1 * tmx1);
  tmx2 = (tmx1 - link2) * l21 / l24;                                //②
  tmy2 = tmy1 * l21 / l24;
  cs1 = cosf(an2);
  sn1 = sinf(an2);
  tmx1 = tmx2 * cs1 - tmy2 * sn1 + tmx1;                            //③   リンク交点延長先 = tm*1
  tmy1 = tmx2 * sn1 + tmy2 * cs1 + tmy1;
  tma1 = acos((a2_2 + tml1_2 - l24_2) / (2 * a2 * link2));            //④
  tma3 = M_PI - tma1;
  cs1 = cosf(tma3);
  sn1 = sinf(tma3);
  tmx2 = sx2 * cs1 - sy2 * sn1;                                       //⑤   サーボ中心 = tm*2
  tmy2 = sx2 * sn1 + sy2 * cs1;
  tmx1 = tmx1 - tmx2;                                                 //⑤   リンク延長先-サーボ中心 = tm*1
  tmy1 = tmy1 - tmy2;
  tml1_2 = tmx1 * tmx1 + tmy1 * tmy1;
  tml1 = -sqrtf(tml1_2);                                                //⑥
  tma2 = atan2f(-tmy1, -tmx1);
  tmx1 = (l23_2 - l22_2 + tml1_2) / (2 * tml1);                     //⑦   サーボ中心を原点としたL22とL23の交点 = tm*2
  tmy1 = sqrtf(l23_2 - tmx1 * tmx1);
  cs1 = cosf(tma2);
  sn1 = sinf(tma2);
  tmx2 = tmx1 * cs1 - tmy1 * sn1;                                   //⑧   サーボ中心を原点としたリンク先端の絶対角度
  tmy2 = tmx1 * sn1 + tmy1 * cs1;
  tma2 = atan2f(tmy2, tmx2);                                         //⑨
  j[3] = -(tma2 + tma1 + H_PI);                                               //⑩   サーボ角度
#endif

  int i;
  int err_flg = 0x01;
  for(i = 0; i < LEG_SERVO_QT; i++){
    if(isnan(j[i]) || j[i] == -100){
      r = r | err_flg;
      j[i] = 0;
    }
    err_flg = err_flg << 1;
  }

  
  // j[1] 左右対応
#ifndef GOPRO
  if((j[1] = hip_joint_ik(j[1], either)) == -100){
    j[1] = 0;
    err_flg = 0x02;
  }
#endif

#ifdef RECORD
  fp9 = fopen("C:\\Users\\kenji\\Documents\\ik_result.csv", "a");
  fprintf(fp9, "ik,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", either, ik[0], ik[1], ik[2], ik[3], ik[4], ik[5], j[0], j[1], j[2], j[3], j[4], j[5]);
  fclose(fp9);
#endif
  return r;
  /*error code
  0-5: nam flag
  6  : sqrt() error
  7  : acos() error
  */
}

int biarticular_dk(FLOAT *j, FLOAT *dk, int either, massPoint *mp)
{
  // j[0] : 股関節yaw
  // j[1] : 股関節roll
  // j[2] : 第一リンク
  // j[3] : 第二リンク
  // j[4] : 足首pitch
  // j[5] : 足首roll

  // dk[0] : X
  // dk[1] : Y
  // dk[2] : Z
  // dk[3] : roll
  // dk[4] : pitch
  // dk[5] : yaw
  // 足質点グループの足原点に対する位置と姿勢を与える

  // either = 0 => right
  // either = 1 => left

  FLOAT x, y, z;
  FLOAT tml1, tma1, tml2;
  FLOAT tmx1, tmy1, tmx2, tmy2;
  FLOAT link1, link2, kroll;
#ifndef GOPRO
  FLOAT cs, sn;
#endif
  FLOAT s_lohx, s_lofx;

  //第一リンク長
#ifdef GOPRO
  link1 = yogen2(a1, l14, a1o - j[2]);
#else
  //サーボリンク先端座標(tmx1,tmy1)を求める
  sn = sinf(-j[2]);
  cs = cosf(-j[2]);
  tmx1 = -l11 * sn + (sx1 - a1);   
  tmy1 = l11 * cs + sy1;
  //L13,L12のリンクセンタ座標(tmx2,tmy2)を求める
  if(crosspoint(tmx1, tmy1, l13, l12, &tmx2, &tmy2, &tml1, &tma1, 0))
    return(1);
  tmx2 = -tmx2 * l14 / l13;
  tmy2 = -tmy2 * l14 / l13;
  tml1 = tmx2 + a1;
  link1 = sqrtf(tml1 * tml1 + tmy2 * tmy2);
    
#endif
  //第二リンク長
#ifdef GOPRO
  link2 = yogen2(a2, l24, a2o - j[3]);
#else
  sn = sinf(-j[3]);
  cs = cosf(-j[3]);
  tmx1 = -l23 * sn + (a2 + sx2);   //サーボリンク先端座標
  tmy1 = l23 * cs + sy2;
  if(crosspoint(tmx1, tmy1, l21, l22, &tmx2, &tmy2, &tml1, &tma1, 1))
    return(2);
  tmx2 = -tmx2 * l24 / l21;
  tmy2 = -tmy2 * l24 / l21;
  rotate_2d(tmx2, tmy2, &tmx1, &tmy1, -an2);
  tml1 = tmx1 - a2;
  link2 = sqrtf(tml1 * tml1 + tmy1 * tmy1);
#endif
  
  //股関節角度
  // j[1] 左右対応
#ifdef GOPRO
  if (either == IK_LEFT){ // left
    kroll = j[1];
  }
  else{ // right
    kroll = -j[1];
  }
#else
  kroll = hip_joint_dk(j[1], either);
#endif
  //足先座標
  if(crosspoint(lnx2 - lnx1, lny2 - lny1, link1, link2, &tmx1, &tmy1, &tml1, &tma1, 2))
    return(3);
  tmx1 += lnx1;  //リンク交点座標
  tmy1 += lny1;
  if(crosspoint(tmx1, tmy1, l1, l3, &tmx2, &tmy2, &tml1, &tma1, (tmx1 < 0) ? 1 : 0))  //膝座標
    return(4);
  y = (tmx1 - tmx2) * l1 / l3 + tmx2;
  z = (tmy1 - tmy2) * l1 / l3 + tmy2;
#ifdef USE_ZMP_ONLINE
  if (mp != NULL){
    //ベースブロックの原点に対する各オブジェクトの原点位置と基本姿勢からの角度を返す
    //質点姿勢【大腿】
    vct_set(0, loby, lobz, mp->ct); // x, y, z => 左右,前後,上下
    mp->ang = atan2f(tmx2, -tmy2);  // pitch, roll, yew
    mp = mp->next;
    //質点姿勢【下腿】
    vct_set(0, tmx2 + loby, tmy2 + lobz, mp->ct); // x, y, z => 左右,前後,上下
    mp->ang = atan2f(tmx1 - tmx2, -(tmy1 - tmy2));  // tm*1:リンク交点　tm*2:膝
    mp = mp->next;
    //質点姿勢【第一リンクベース】
    vct_set(0, lnx1 + loby, lny1 + lobz, mp->ct); // x, y, z => 左右,前後,上下
    FLOAT tmp_ang = atan2f(tmx1 - lnx1, lny1 - tmy1) - yogen1(l14, a1, link1);
    mp->ang = tmp_ang - a_ele_o;
    mp = mp->next;
    //質点姿勢【第一リンク】
    FLOAT tx = a1 * sinf(tmp_ang) + lnx1;
    FLOAT ty = -a1 * cosf(tmp_ang) + lny1;
    vct_set(0, tx + loby, ty + lobz, mp->ct); // x, y, z => 左右,前後,上下
    mp->ang = atan2f(tmx1 - tx, -(tmy1 - ty)) - a_elnk_o;
    mp = mp->next;
    //質点姿勢【第二リンクベース】
    vct_set(0, lnx2 + loby, lny2 + lobz, mp->ct); // x, y, z => 左右,前後,上下
    tmp_ang = yogen1(l24, a2, link2) + atan2f(tmx1 - lnx2, lny2 - tmy1);
    mp->ang = tmp_ang - a_str_o;
    mp = mp->next;
    //質点姿勢【第二リンク】
    tx = a2 * sinf(tmp_ang) + lnx2;
    ty = -a2 * cosf(tmp_ang) + lny2;
    vct_set(0, tx + loby, ty + lobz, mp->ct); // x, y, z => 左右,前後,上下
    mp->ang = atan2f(tmx1 - tx, -(tmy1 - ty)) - a_slnk_o;
    mp = mp->next;
    //足首ブロック
    vct_set(0, y + loby, z + lobz, mp->ct); // x, y, z => 左右,前後,上下
    mp->ang = j[4];
  }
#else
  //質点姿勢【大腿】
  vct_set (0, 0, 0, mp[1].pos); // x, y, z => 左右,前後,上下
  vct_set(atan2f(tmx2,-tmy2), 0, 0, mp[1].posture);  // pitch, roll, yew
  //質点姿勢【下腿】
  vct_set(0, tmx2, tmy2, mp[2].pos); // x, y, z => 左右,前後,上下
  vct_set(atan2f(tmy1-tmy2,-(tmx1-tmx2)), 0, 0, mp[2].posture);  // tm*1:リンク交点　tm*2:膝
  y = (tmx1 - tmx2) * l1 / l3 + tmx2;
  z = (tmy1 - tmy2) * l1 / l3 + tmy2;
  //質点姿勢【第一リンクベース】
  vct_set(0, lnx1, lny2, mp[3].pos); // x, y, z => 左右,前後,上下
  vct_set(atan2f(tmx1-lnx1, lny1-tmy1) - yogen1(a1, link1, l14), 0, 0, mp[3].posture);
  //質点姿勢【第二リンクベース】
  vct_set(0, lnx2, lny2, mp[4].pos); // x, y, z => 左右,前後,上下
  vct_set(yogen1(a2, link2, l24) + atan2f(tmx1-lnx2, lny1-tmy2) - M_PI, 0, 0, mp[4].posture);
  //質点姿勢【第一リンク】
  float ty = a1 * sinf(mp[3].posture[PI]) + lnx1;
  float tz = -a1 * cosf(mp[3].posture[PI]) + lny1;
  vct_set(0, ty, tz, mp[5].pos); // x, y, z => 左右,前後,上下
  vct_set(atan2f(tmy1 - tz, -(tmx1 - ty )), 0, 0, mp[5].posture);
  //質点姿勢【第二リンク】
  ty = -a2 * sinf(mp[4].posture[PI]) + lnx2;
  tz = a2 * cosf(mp[4].posture[PI]) + lny2;
  vct_set(0, ty, tz, mp[6].pos); // x, y, z => 左右,前後,上下
  vct_set(atan2f(tmy1 - tz, -(tmx1 - ty )), 0, 0, mp[6].posture);
#endif
  if (either == IK_LEFT){ // left
    s_lohx = lohx;
    s_lofx = -lofx;
  }
  else{ // right
    s_lohx = -lohx;
    s_lofx = lofx;
  }
  // 上位座標からの差分表現に統一（符号を反転）  20170302

  tml1 = z + lobz; //ld = l + lobz
  tml2 = sqrtf(tml1 * tml1 + lofx_2);//ldd = sqrt(ld^2 + lofx^2)
  tma1 = j[1] + atan2f(-s_lofx, -tml1); //jd = atan(-lofx / ld), j1d = j1+ jd
  x = -tml2 * sinf(tma1) + s_lohx; // x + lohx = ldd * cosf(j1d) -> x = ldd * cosf(j1d) + lohx

  dk[0] = x;
  dk[1] = y;
  dk[2] = -tml2 * cosf(tma1) - loz;
  dk[3] = 0;
  dk[4] = 0;
  dk[5] = 0;
  return(0);
}

void rotate_2d(FLOAT ix, FLOAT iy, FLOAT *ox, FLOAT *oy, FLOAT a)
{
  FLOAT sa = sinf(a);
  FLOAT ca = cosf(a);

  *ox = ix * ca - iy * sa;
  *oy = ix *sa + iy * ca;
}

void rotate_x(FLOAT *x, FLOAT *y, FLOAT *z, FLOAT a)
{
  FLOAT sa = sinf(a);
  FLOAT ca = cosf(a);

  FLOAT _y = *y * ca - *z * sa;
  FLOAT _z = *y *sa + *z * ca;

  *y = _y;
  *z = _z;
}

void rotate_y(FLOAT *x, FLOAT *y, FLOAT *z, FLOAT a)
{
  FLOAT sa = sinf(a);
  FLOAT ca = cosf(a);

  FLOAT _x = *x * ca + *z * sa;
  FLOAT _z = -*x *sa + *z * ca;

  *x = _x;
  *z = _z;
}

void rotate_z(FLOAT *x, FLOAT *y, FLOAT *z, FLOAT a)
{
  FLOAT sa = sinf(a);
  FLOAT ca = cosf(a);

  FLOAT _x = *x * ca - *y * sa;
  FLOAT _y = *x *sa + *y * ca;

  *x = _x;
  *y = _y;
}

FLOAT func_hung(FLOAT t, FLOAT *u, FLOAT *r)
{
  if(t < 0) t += 2;
  if(t > 2) t -= 2;
  *r = t;
  if(t < 1){
    *u = t;
    return (1 - cosf(2 * t * M_PI)) * 0.5;
  }else{
    *u = 0;
    return 0;
  }
}

FLOAT func_walk(FLOAT t)
{
  if(t < 0) t += 2;
  if(t > 2) t -= 2;
  if(t < 1){
    return(-cosf(t * M_PI));
  }else{
    return(1 - 2 * (t - 1));
  }
}

FLOAT yogen1(FLOAT a, FLOAT b, FLOAT c)  // 角度aaを求める
{
  FLOAT x = b * b + c * c - a * a;
  FLOAT r = 2 * b * c;
  if(x > r) return -100; //error;
  else return acos(x / r);
}

FLOAT yogen2(FLOAT b, FLOAT c, FLOAT aa)  // 対辺aを求める
{
  return(sqrtf(b * b + c * c - 2 * b * c * cosf(aa)));
}

int crosspoint(FLOAT x, FLOAT y, FLOAT r1, FLOAT r2, FLOAT *rx, FLOAT *ry, FLOAT *dist, FLOAT *ang, int mode)
{
  // ２円の交点を求める
  // x, y : 第二円の座標（第一円の中心を原点とした相対座標）
  // r1: 第一円の半径
  // r2: 第二円の半径
  // *rx: 交点x
  // *ry: 交点y
  // *dist, *ang: 第二円の極座標（距離と角度）
  // mode:反転をサポートするなら0x01　しないなら0x00
  //     :ty = -sqrt( )とするなら、0x02
  // 返り値が1ならエラー（交点がない）
  FLOAT d_2 = x * x + y * y;
  *dist = sqrtf(d_2);
  if(*dist > r1 + r2){
    *rx = *ry = *ang = 0;
    return 1;
  }else{
    if(x > 0 || mode & 1){
      *ang = atan2f(y, x);
    }else{
      *ang = atan2f(-y, -x);
      *dist = -*dist;
    }
    FLOAT r1_2 = r1 * r1;
    FLOAT tx = (r1_2 - r2 * r2 + d_2) / ( 2 * *dist);
    FLOAT ty = sqrtf(r1_2 - tx * tx);   //怪しいのはここかな？？？
    if(mode & 2){
      ty = -ty;
    }
    FLOAT sn = sinf(*ang);
    FLOAT cs = cosf(*ang);
    *rx = tx * cs - ty * sn;
    *ry = tx * sn + ty * cs;
    return 0;
  }
}

short conv_f_sv(FLOAT fpos)
{
  return (short)(fpos * f_sv_fact);
}

FLOAT conv_sv_f(Joint *j, short cap)
{
  return (FLOAT)((cap - j->d[JOINT_TRIM] - j->d[JOINT_OFFSET] - 7500) * j->d[JOINT_SIGN] * sv_f_fact );
}

FLOAT conv_sv_f2(short pos){
  return (FLOAT)(pos * sv_f_fact);
}

short conv_sv_f3(Joint *j, short cap)
{
  return (cap - j->d[JOINT_TRIM] - j->d[JOINT_OFFSET] - 7500) * j->d[JOINT_SIGN];
}

short conv_l_f(Joint *j, short pos)  // 論理位置を物理位置に変換
{
    return (pos * j->d[JOINT_SIGN] + 7500 + j->d[JOINT_OFFSET] + j->d[JOINT_TRIM]);

}


//////////////////////////////////////////////////////////////
// PID
//////////////////////////////////////////////////////////////

void clear_pid(PidSet *p)
{
  p->u = 0;
  p->ut = 0;
  p->bi = 0;
  p->integral = 0;
  p->release_counter = 0;
}

void calc_pid(PidSet *p, FLOAT inp, int dv)
{
  FLOAT diff;
  short u_this;
  if(inp * p->bi < 0){
    p->integral = 0; //積分値のリセット
  }else{
    p->integral += inp;
  }
  diff = inp - p->bi;
  u_this = (short)(inp * p->kp + p->integral * p->ki + diff * p->kd);
  if(dv){  //サーボ反応待ち
	    p->ut = -u_this - p->ut;  //前回データをキャンセルして今回に置き換え。　なんでu_thisをマイナスにしているのかわからない。
	    //解説：出力はu 前回utを足しこんでuになったわけだから、utを引くとキャンセルしたことになり、そのうえに今回のu_this	を適用することになる。
  }else{
	    p->ut = -u_this;
  }
  p->u += p->ut;
  p->bi = inp;
}

int release_pid(PidSet *p, short sh, short df)
{
  if(++p->release_counter > sh){
    if(p->u > 0){
      p->u -= df;
      if(p->u < 0){
        p->u = 0;
        p->release_counter = 0;
      }
    }else{
      p->u += df;
      if(p->u > 0){
        p->u = 0;
        p->release_counter = 0;
      }
    }
    return 1;
  }else{
    return 0;
  }
}

void clear_pid2(PidSet2 *p)
{
  p->u = 0;
  p->bu = 0;
  p->bi = 0;
  p->integral = 0;
  p->release_counter = 0;
}

void calc_pid2(PidSet2 *p, FLOAT *pm, FLOAT inp)
{
	FLOAT diff;

	  p->integral += inp;
	  diff = inp - p->bi;
	  p->bu = p->u;
	  p->u = inp * pm[0] + p->integral * pm[1] + diff * pm[2];
	  if(fabs(p->u) > pm[3]){
		  p->u = pm[3] * p->u / fabs(p->u);
	  }
	  p->bi = inp;
}

int release_pid2(PidSet2 *p, short sh, short df)
{
}
