/***********************************************/
/*   File        :lambda.h                     */
/*   Version     :v1.0.2                       */
/*   Date        :2017/02/07                   */
/*   Author      :Kenji Shimada                */
/*   動力学フィルターに対応                    */
/***********************************************/

#ifndef __LAMBDA_H__
#define __LAMBDA_H__

//#define WALK

//#define GOPRO
#define STBEEF4
//#define STBEEMINI
#define USE_ZMP_ONLINE
#define USE_ZMP_DF
#define ARM_SWING

#ifdef GOPRO
#define FLOAT double
#else
#define FLOAT float
#endif

#ifdef USE_ZMP_ONLINE
#include "multi_mass_point_model.h"
#endif

// *****************   STBee-MINI上でのペリフェラル割り当て
// USART1(TX:PA09,RX:PA10):コンソール通信用 DMA送信　フラグ受信
// USART3(TX:PB10,RX:PB11):サーボ通信用 DMA送信　DMA受信　送信終了割り込み：送信イネーブル処理
// TIM2:サーボインターバルタイマー　20msインターバルで割り込み発生
// GPIOD PIN4, PIN5 :LED L:enable
// GPIOB PIN12 :SPI CS L:enable
// SPI2((NSS:PB12)SCK:PB13,MOSO:PB14,MOSI:PB15)
// ADC00-ADC07(PA0-PA9) 足裏センサー
// ADC08(PB0)
// ADC09(PB1)

// バッテリー電圧校正値
// 12.3V 2636
// 10.0V 2381
//  8.0V 2100

#ifdef STBEE
#define LED01    GPIOD, GPIO_Pin_4
#define LED02    GPIOD, GPIO_Pin_5
#else
#ifdef STBEEMINI
#define LED01    GPIOA, GPIO_Pin_13
#define LED02    GPIOA, GPIO_Pin_15
#else
#ifdef STBEEF4
#define LED01    GPIOD, GPIO_Pin_2
#define LED02    GPIOD, GPIO_Pin_3   // but GPIOD_PIN is only PIN2
#endif
#endif
#endif

#ifdef STBEE
#define SPICS    GPIOA, GPIO_Pin_12
#else
#ifdef STBEEMINI
#define SPICS    GPIOA, GPIO_Pin_12
#else
#ifdef STBEEF4
#define SPICS    GPIOB, GPIO_Pin_8
#endif
#endif
#endif

#define X 0
#define Y 1
#define Z 2

#define PI 0
#define RO 1
#define YO 2

//ジャイロと加速度センサーの並び
#define SX 2
#define SY 0
#define SZ 1

//ジャイロと加速度センサーの符号
#define SXS -1
#define SYS -1
#define SZS 1


#define IK_RIGHT 0
#define IK_LEFT  1

#define LEGS_SERVO_QT 16
#define LEG_SERVO_QT 8
#define LEG_RIGHT 0
#define LEG_LEFT  8
#define LEG_DK_RIGHT 0
#define LEG_DK_LEFT  6

///////// Leg Plessure /////////////
#define ZMP_RX 0
#define ZMP_RY 1
#define ZMP_RP 2
#define ZMP_LX 3
#define ZMP_LY 4
#define ZMP_LP 5
///////// Leg Plessure /////////////

#define ADXL345  0xA6
#define ITG3200  0xD0
#define I2CREAD  0x00
#define I2CWRITE 0x01
#define I2C_Speed              100000
#define I2C_SLAVE_ADDRESS7     0x10

//#define QMC5883L 0x1A  // 0x0d


#define FLASH_JOINT 0x080FF000
#define FLASH_SENSOR 0x080FF800
#define FLASH_PARAM 0x080FFC00

#define IK_ERR_STAGE_1           1
#define IK_ERR_STAGE_2           2
#define IK_ERR_STAGE_3           3
#define IK_ERR_STAGE_4           4
#define IK_ERR_STAGE_5           5
#define IK_ERR_STAGE_6           6
#define IK_ERR_STAGE_7           7
#define IK_ERR_STAGE_8           8
#define IK_ERR_RATIO_OVER_RANGE  9
#define IK_ERR_OVER_RANGE       10

#define SV_ERR_ACK_RECEIVE_FALSE 0x0001
#define SV_ERR_TIMEOUT           0x0002
#define SV_ERR_MAX_OVER          0x0100
#define SV_ERR_MIN_OVER          0x0200
#define SV_ERR_RANGE_OVER        0x0300

#if 0
#define SERVO_CNTL func_mess[func_mess_num++] = FM_servo_cntl
#define SERVO_ACK_RECEIVE func_mess[func_mess_num++] = FM_servo_ack_receive
#define SERVO_ACK_RECEIVE_END func_mess[func_mess_num++] = FM_servo_ack_receive_end
#define SVCOM_SEND func_mess[func_mess_num++] = FM_uart1_send
#define SVCOM_RECEIVE func_mess[func_mess_num++] = FM_uart1_receive
#define SVCOM_RECEIVE_RESET func_mess[func_mess_num++] = FM_uart1_receive_reset
#define SVCOM_RECEIVE_ABORT func_mess[func_mess_num++] = FM_uart1_receive_abort
#define TIM2_IRQHANDELER func_mess[func_mess_num++] = FM_TIM2_IRQHandler
#define TIM3_IRQHANDELER func_mess[func_mess_num++] = FM_TIM3_IRQHandler
#define TIM4_IRQHANDELER func_mess[func_mess_num++] = FM_TIM4_IRQHandler
#else
#define SERVO_CNTL
#define SERVO_ACK_RECEIVE
#define SERVO_ACK_RECEIVE_END
#define SVCOM_SEND
#define SVCOM_RECEIVE
#define SVCOM_RECEIVE_RESET
#define SVCOM_RECEIVE_ABORT
#define TIM2_IRQHANDELER
#define TIM3_IRQHANDELER
#define TIM4_IRQHANDELER
#define SERVO_ACK_RECEIVE_SUCCESS func_mess[func_mess_num++] = FM_servo_ack_receive_success
#define SERVO_ACK_RECEIVE_FALSE func_mess[func_mess_num++] = FM_servo_ack_receive_false
#endif

//#define ZMPT     //zmpplanのアレンジにzmpplanを複製して行うコーディングの設定　メモリを大量に消費する。コメントアウトすればzmp_make_walk_motion()でアレンジをキャンセルする記述になる。　2017/10/20

enum func_mess_ {
  FM_servo_cntl = 1,
  FM_servo_ack_receive,
  FM_servo_ack_receive_end,
  FM_svcom_send,
  FM_svcom_receive,
  FM_svcom_receive_reset,
  FM_svcom_receive_abort,
  FM_TIM2_IRQHandler,
  FM_TIM3_IRQHandler,
  FM_TIM4_IRQHandler,
  FM_servo_ack_receive_success,
  FM_servo_ack_receive_false,
};

enum sv_cmd {
  SV_UNDEF = -2,
  SV_UNLIVE = -1,
  SV_OFF = 0,
  SV_ON,
  SV_POS,
  SV_POS_SPEED,
  SV_POS_STRETCH,
  SV_DISCONNECT,
};

enum sv_prm {
  SV_PRM = -1,
  SV_ROM = 0,
  SV_STRETCH,
  SV_SPEED,
  SV_CRNT,
  SV_TMP
};

enum sv_ics {
	SV_ICS_3_0,
	SV_ICS_3_5
};

enum sv_joint_name {
	joint_right_hip_yaw,
	joint_right_hip_roll,
	joint_right_elevator,
	joint_right_stride,
	joint_right_ankle_pitch,
	joint_right_ankle_roll,
	joint_right_hip_pitch,
	joint_right_knee,
	joint_left_hip_yaw,
	joint_left_hip_roll,
	joint_left_elevator,
	joint_left_stride,
	joint_left_ankle_pitch,
	joint_left_ankle_roll,
	joint_left_hip_pitch,
	joint_left_knee,
};

#define JOINT_D 32
enum joint_d {
  JOINT_SVID	,
  JOINT_SIGN	,   // 物理回転方向と論理回転方向の変換
  JOINT_OFFSET	,   // 物理角度と論理角度の差　POS = TARGET * SIGN + OFFSET
  JOINT_TRIM	, 
  JOINT_MAX	,   // 最大角度（単位：サーボの物理角度）
  JOINT_MIN	,   // 最小角度（単位：サーボの物理角度）
  JOINT_CMD     ,   // sv_cmd
  JOINT_TARGET	,   // 関節の論理目標角度
  JOINT_TIME	,   // 移動時間（指示）
  JOINT_PRMCMD  ,   // sv_prm
  JOINT_PRM     ,
  JOINT_POS	,   // サーボの物理ポジション
  JOINT_LPOS    ,   // 関節の論理指示角度
  JOINT_CAPTURE	,
  JOINT_STATUS	,   // -2:no connect -1:initial 0:torque off 1:torque on 3:move_now
  JOINT_STRETCH ,
  JOINT_SPEED   ,
  JOINT_REALCRNT,    //現在電流
  JOINT_REALTMP ,    //現在温度
  JOINT_CRNT    ,    //電流制限
  JOINT_TMP     ,    //温度制限
  JOINT_RTN     ,
  JOINT_FLG     ,   // リターンフラグ
  JOINT_ERR     ,    // エラーフラグ
  JOINT_ERRCNT  ,    // エラーカウント
  JOINT_ICS     ,
  JOINT_SET_STRETCH,
  JOINT_SET_SPEED,
  JOINT_SET_CRNT,
  JOINT_SET_TMP ,
  JOINT_DUMMY1  ,
  JOINT_DUMMY2  ,
  JOINT_DUMMY3
};

typedef struct _joint{
  short d[JOINT_D];
} Joint;

typedef struct _sensoruniquedata{
  short lp_adc_max[8];
  short lp_adc_min[8];
  short gyro[8];
  short gravity[8];
} SensorUniqueData;

union word2byte{
  short wd;
  char cd[2];
};

#ifndef USE_ZMP_ONLINE
typedef struct _base{
  int walkno;
  int phaseno;
  FLOAT ratio;
  FLOAT under;
  int damage_limit;
  int battery_limit;
} Base;
#endif

#ifdef WALK
typedef struct _walk_para_set{
  FLOAT x_leg_z;  //X軸周りの足の相当腕長さ
  FLOAT y_leg_z;  //Y軸周りの足の相当腕長さ
  FLOAT x_body_z;  //X軸周りの胴体の相当腕長さ
  FLOAT y_body_z;  //Y軸周りの胴体の相当腕長さ
  FLOAT o_z;    //股関節（原点）

  FLOAT x_leg_w;  //X軸周りのleg weight
  FLOAT y_leg_w;  //Y軸周りのleg weight
  FLOAT x_body_w; //X軸周りのbody weight
  FLOAT y_body_w; //Y軸周りのbody weight

  FLOAT start_y;
  FLOAT end_y;
  FLOAT body_s; //body swing
  FLOAT pace_t;

  FLOAT io;  //スタンス（ロボット原点（仮想股関節）から足先点の水平距離）
  FLOAT G; // 重力加速度

  FLOAT stride; //歩幅  (歩行用データ) 符号で進行方向を示す
  FLOAT hung;   //足上げ寸法
  int f_wait;   //両足支持期間（前半）
  int e_wait;   //両足支持期間（後半）
  int w_cycle;  //pace_t / 0.02
//  int direction; //進行方向
} WalkParaSet;
#endif

typedef struct _pid{
  short kp;
  short ki;
  short kd;
  short u;  //操作量
  short ut;  //前回の操作量
  FLOAT bi;  //前回の入力
  FLOAT integral;  //積分値
  short release_counter;
} PidSet;

typedef struct _pid_2{
  FLOAT u;  //操作量
  FLOAT bu;  //前回の操作量
  FLOAT bi;  //前回の入力
  FLOAT integral;  //積分値
  short release_counter;
} PidSet2;

#define WS_SIZE 3
#define WS_ZMPCHANGE_REJECT_COUNT 15

enum pivotLeg {plBoth, plRight, plLeft};
enum walkDir {wdFront, wdBack};
enum pivotLegStatus {plsNop, plsChange, plsTop};
enum freeLegStatus {flsNop, flsTakeoff, flsTop, flsTouch};

typedef struct _walk_status{
	enum pivotLeg leg;
	enum walkDir dir;
	enum pivotLegStatus pivot;
	enum freeLegStatus free;
	int pivot_count;
	int free_count;
	FLOAT dx;
	FLOAT ddx;
	FLOAT hdx;
} WalkStatus;

enum iParameters {
	_n_step,
	_step,
	_delay,
	_innerfall_timing,
	_innerfall_threshold,
	_d_step,
	_cp_space_threshold,
	_gyro_select,
	_lp_select,
	_lp_mag,
	_gyro_feedback_debug1,
	_gyro_feedback_debug2,
	_arm_swing_f_1,
	_arm_swing_f_2,
	_arm_swing_f_3,
	_arm_swing_f_4,
	_arm_swing_b_1,
	_arm_swing_b_2,
	_arm_swing_b_3,
	_arm_swing_b_4,
	_walk_follow_threshold_times,
	_upboard_mode,
	_param_i_end
};

enum fParameters {
	_shift_zmp_rx,
	_shift_zmp_lx,
	_shift_zmp_ry,
	_shift_zmo_ly,
	_default_lbp_width,
	_default_lbp_depth,
	_default_lbp_height,
	_default_lbp_hung,
	_default_ubp_1,
	_default_ubp_2,
	_default_ubp_3,
	_default_ubp_4,
	_pace,
	_hung,
	_faststep_arange,
	_gtz_adjust,
	_gtz_adjust_reccati,
	_height_adjust,
	_innerfall_reaction,
	_ankle_pitch_calibration_amount,
	_ankle_roll_right_calibration_amount,
	_ankle_roll_left_calibration_amount,
	_com_calibration_amount,
	_zmp_align_amount,
	_gyro_roll_feedback_retio,
	_gyro_pitch_feedback_retio,
	_pos_pitch_calibration_retio,
	_xzmp_right_roll_calibration_retio,
	_xzmp_left_roll_calibration_retio,
	_zmp_pitch_align_retio,
	_pid_y_offset_rate_gyro,
	_pid_y_offset_rate_zmp,
	_gyro_angle_offset_roll,
	_gyro_angle_offset_pitch,
	_pid0_p_fact,
	_pid0_i_fact,
	_pid0_d_fact,
	_pid0_u_max,
	_pid1_p_fact,
	_pid1_i_fact,
	_pid1_d_fact,
	_pid1_u_max,
	_pid2_p_fact,
	_pid2_i_fact,
	_pid2_d_fact,
	_pid2_u_max,
	_pid3_p_fact,
	_pid3_i_fact,
	_pid3_d_fact,
	_pid3_u_max,
	_pid4_p_fact,
	_pid4_i_fact,
	_pid4_d_fact,
	_pid4_u_max,
	_pid5_p_fact,
	_pid5_i_fact,
	_pid5_d_fact,
	_pid5_u_max,
	_pid6_p_fact,
	_pid6_i_fact,
	_pid6_d_fact,
	_pid6_u_max,
	_pid7_p_fact,
	_pid7_i_fact,
	_pid7_d_fact,
	_pid7_u_max,
	_pid8_p_fact,
	_pid8_i_fact,
	_pid8_d_fact,
	_pid8_u_max,
	_up_down_action_accel,
	_walk_follow_max,
	_walk_follow_threshold,
	_criteria_of_standup,
	_criteria_of_stable,
	_criteria_of_motion_posture,
	_param_f_end
};

#define PARAM_I_MAX 30
#define PARAM_F_MAX 80
#define PARAM_I_MAX_OLD 30
#define PARAM_F_MAX_OLD 80

typedef struct _parameters{
	int i_vol;
	int i_vol_old;
	int f_vol;
	int f_vol_old;
	int i[PARAM_I_MAX];
	FLOAT f[PARAM_F_MAX];
} Parameters;

#ifndef USE_ZMP_ONLINE
typedef struct _masspoint{
  FLOAT pos[3];
  FLOAT posture[3];
} massPoint;
#endif

void lambda_ik_ini();

#ifdef WALK
void calc_walk(WalkParaSet *w, FLOAT t, FLOAT *x, FLOAT *bx, FLOAT *y);
FLOAT calc_depth(WalkParaSet *w, int cycle);
void print_walk(WalkParaSet *w);
#endif

void joint_init(void);
FLOAT hip_joint_ik(FLOAT hip_roll_angle, int side);
FLOAT hip_joint_dk(FLOAT hip_servo_angle, int side);
unsigned char biarticular_ik(FLOAT *ik, FLOAT *j, int either);
int biarticular_dk(FLOAT *j, FLOAT *dk, int either, massPoint *mp);
char sum_data(int n, char *data);
void w2b(int, char*);

void rotate_2d(FLOAT ix, FLOAT iy, FLOAT *ox, FLOAT *oy, FLOAT a);
void rotate_x(FLOAT *x, FLOAT *y, FLOAT *z, FLOAT a);
void rotate_y(FLOAT *x, FLOAT *y, FLOAT *z, FLOAT a);
void rotate_z(FLOAT *x, FLOAT *y, FLOAT *z, FLOAT a);
FLOAT func_hung(FLOAT t, FLOAT *u, FLOAT *r);
FLOAT func_walk(FLOAT t);
FLOAT yogen1(FLOAT a, FLOAT b, FLOAT c);
FLOAT yogen2(FLOAT b, FLOAT c, FLOAT aa);
int crosspoint(FLOAT x, FLOAT y, FLOAT r1, FLOAT r2, FLOAT *rx, FLOAT *ry, FLOAT *dist, FLOAT *ang, int mode);
short conv_f_sv(FLOAT fpos);
FLOAT conv_sv_f(Joint *j, short cap);
FLOAT conv_sv_f2(short pos);
short conv_sv_f3(Joint *j, short cap);
short conv_l_f(Joint *j, short pos);  // 論理位置を物理位置に変換


void clear_pid(PidSet *p);
void calc_pid(PidSet *p, FLOAT inp, int dv);
int release_pid(PidSet *p, short sh, short df);

void clear_pid2(PidSet2 *p);
void calc_pid2(PidSet2 *p, FLOAT *pm, FLOAT inp);
int release_pid2(PidSet2 *p, short sh, short df);

void cnv_servo_pos(char h, char l, short *pos);
void cnv_pos_servo(short pos, char *h, char *l);
void make_servo_target(int sv_id, int target, char *senddata);
void make_servo_setting(int sv_id, char sc, char data, char *senddata);
void make_servo_read_para(int sv_id, char sc, char *senddata);
void cnv_rom_int2(char r1, char r2, int *data);
void cnv_int_rom2(int data, char *r1, char *r2);
void cnv_rom_int4(char r1, char r2, char r3, char r4, int *data);
void cnv_int_rom4(int data, char *r1, char *r2, char *r3, char *r4);


#define SV_VOL_OLD 12                     // サーボ数
#define SV_VOL 24                     // サーボ数
#define SVID_MAX 25                   // 逆引きインデックス用配列宣言数

extern Joint joint[];
extern int sv_rev_index[];
extern SensorUniqueData sud;
#ifdef WALK
extern WalkParaSet wps;
#endif

extern char *func_mess_name[];

extern int s_idx[];
extern int s_sgn[];

extern short adjust[];

extern char *param_name_i[];
extern char *param_name_f[];

extern Parameters param;

//extern WalkStatus walkstatus[];
//extern int walkstatus_index;

#endif
