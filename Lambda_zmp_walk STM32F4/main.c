//
// 二重四節リンク型ラムダ
// ZMP規範のオンライン生成方式with動力学フィルター
//
// Defines


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>  //sbrk対応で追加してみた。
#include "lambda.h"
#include "multi_mass_point_model.h"
#include "zmpwalk.h"
#include "lbposture.h"
#include "command.h"
#include "controler.h"
#include "motion.h"

/*
  PORT ASSIGN
  terminal communication(tmcom)    : USART1   TX:PA9 RX:PA10
  servo communication(svcom)       : USART3   TX:PB10 RX:PB11
  servo communication(svcom)       : USART4   TX:PC10 RX:PC11
  controler communication(cntlspi) : SPI2 => SPI1  MOSI:PB5 MISO:PB4 SCK:PB3
  SPI CS                           : GPIO     output : PB8
  gyro gravity sensor              : I2C1     SCL:PB6 SDA:PB7
  compas						   : I2C3     SCL:PA8 SDA:PA9
  pressure sensor                  : ANA1     PA0-7 PB0,1
  LED                              : GPIOD
*/

void RCC_Configuration( void);
void GPIO_Configuration( void);
void USART_Configuration( void);
void SPI_Configuration(void);
void TIM_Configuration( void);
void NVIC_Configuration( void);
void DMA_Configuration( void);
void ADC_Configuration(void);
void I2C_Configuration(void);
void tmcom_outval(long int val);
void tmcom_outvalf(int _order, char fill, long val);
void tmcom_outfloat(float val);
void tmcom_out2h(unsigned char val);
void tmcom_out4h(unsigned short val);
void tmcom_puts(char *send_data);
void tmcom_putc(char c);
void tmcom_send(short n);
void tmcom_send_end(void);
void svcom_send(int, char*);
void servo_ack_receive(int rcv_size);
void servo_ack_receive_end(void);
void tmcom_err_msg(int snd_n, char *errmsg);
void svcom_receive(int rcv_n);
void svcom_receive_reset(void);
void svcom_receive_abort(void);
uint32_t I2C1_BufferRead(uint8_t dev, uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint32_t I2C1_ByteWrite(uint8_t dev, uint8_t pBuffer, uint16_t WriteAddr);

uint32_t I2C3_BufferRead(uint8_t dev, uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint32_t I2C3_ByteWrite(uint8_t dev, uint8_t pBuffer, uint16_t WriteAddr);

int str_cpy(char *dst, char *src);
void str_cpyn(char *dst, char *src, int n);
int str_len(char *str);
void out1h(char *txt, unsigned char value);
int out2h(char *txt, unsigned char value);
int out4h(char *txt, unsigned short value);
int bin2hex(int bn, char *bin, char *hex);
int outval(char *txt, int val);
int outfloat(char *txt, float val);
void add_eol(char *txt);
int add_ptr(int* p);
void conv_endian(char *val);

void arm_data_set();
void arm_swing_action(FLOAT step, FLOAT depth, FLOAT *j, int sw);
void zmp_walk(void);
void zmp_walk_rebuild_pre(int md,FLOAT wd, FLOAT twd, int stp, FLOAT rd, FLOAT pc, int nstp);
void zmp_walk_rebuild(int);
void search_cp(void);

int walkStatusIndex(int wsi, int di);
void walkStatusInit();

void exec_motion_data(motionData *md);

void nmi_handler(void);
void hardfault_handler(void);


#define USART3BUFFSIZE 100
volatile static char svcom_buff[USART3BUFFSIZE], svcom_rcv_buff[USART3BUFFSIZE];

int bin2hex(int, char*, char*);
int servo_cntl(void);
void save_flash(void);
void load_flash(void);
void load_parameters(void);
void exec_motion_frame(motionFrame *mf);

void key_control(void);

void show_sw_status(int);

char* strtok_(char* str, char delim);

//FLOAT body_roll_adj(void);

volatile static char msg[200];
DMA_InitTypeDef DMA_InitStructure1;  //USART1送信DMA用
DMA_InitTypeDef DMA_InitStructure2;  //USART3受信DMA用
DMA_InitTypeDef DMA_InitStructure3;  //USART3送信DMA用
DMA_InitTypeDef DMA_InitStructure4;  //USART4送信DMA用
DMA_InitTypeDef DMA_InitStructure5;  //USART4受信DMA用
volatile static int n;
volatile static int svcom_rcv_size = 0;  // svcom受信フラグ

volatile int sv_prm;              // パラメータコマンドフラグ
volatile int jidx;                // グローバルなjoint index (joint[sv])
volatile int sv;
volatile short sv_cmd_real;       // 現在送信中のコマンド    0x20:パラメータ系　0x10:パラメータリード　0x00:位置コマンド

volatile static unsigned short spidata[8] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};

volatile static int servo_on;

volatile static float joy[4];
volatile static float joy_dir[2];
volatile static float joy_mag[2];

unsigned short res;

//ROBOT
Base robot;
Branch *br;
massPoint *m;

//flash
int flash_load;
#define FLASH_LOAD_WAIT 100
#define SERVO_START_WAIT 50
//servo interval timer
int tim2_start;

static int spi_disp;
static int sensor_disp = 0;

int func_mess[50];
int func_mess_num;

//文字列送信バッファ
#define SENDBUFFSIZE 300
volatile static char sendbuff[SENDBUFFSIZE];
volatile static int send_flg;  //tmcom送信中フラグ
volatile static int tmcom_wd;  //tmcom送信完了割り込み抜け監視
volatile static int tmcom_senderr;

//コマンドヒストリー
#define HISTRY_MAX 10
#define LINEINPUT_MAX 300
char comhistry[HISTRY_MAX][LINEINPUT_MAX];
int histry_pos, histry_read_pos, histry_most_old;
int esc_mode;

//コマンド入力
char inp_str[LINEINPUT_MAX];
char err_string[32];
char rchar;
int str_cur;
int cur;
int df, df2;
char dfstr[LINEINPUT_MAX];
int cmd_num;
char *inp_str_ptr;

//コマンドパラメータ
#define PRM_MAX 30
char prm_str[PRM_MAX][10];
int prm_num;

volatile static int command = CMD_NOP;
volatile static int command_req = CMD_NOP;
volatile static int shift_code = 0;
volatile static int command_code = 0;
volatile static int command_stack = CMD_NOP;
volatile static int d_command = 0;
volatile static int d_command_exor = 0;
volatile static int c_num = 0;

int syntax_err = -1;

volatile static int controler_connect = 0;

vu16 ADC1ConvertedValue[10];
volatile static char batt_check_timing;
volatile static char damage_on;
volatile static char laser_on;
volatile static char batt_check_on;
volatile static char damage_check_on;
volatile static char test_on, test_id, test_end;

#define PRM_QTY 20
short test_stretch, test_stretch2;
short test_prm[PRM_QTY];
float test_fprm[10];
short test_disp;

volatile static int b = 0;

///////// Leg Plessure /////////////
short lp_adc[10];
float zmp[6];
static int legpressure_row_flag = 0;
///////// Leg Plessure /////////////

///////// Gyro Accl Sensor Data ////////////
static uint8_t accld[6];
static uint8_t gyrod[6];
static int sensor_read_flag;
static long sensor_disp_frame = 0;
///////// Gyro Accl Sensor Data ////////////

#ifdef QMC5883L
////////// Compass ///////////////
uint8_t compass[6];
////////// Compass ///////////////
#endif

///////// Sensor Data Output ////////////
static int sensor_out_flag = 0;
static long sensor_out_frame = 0;
static int sensor_out_pace = 10;
///////// Sensor Data Output ////////////

///////// Servo Monitor //////////
int servo_mon_req = 0;
long servo_mon_disp = 0;
long servo_mon_disp_before = 0;
int servo_mon_title = 0;
int servo_mon_qt = 0;
int servo_mon_list[14];

int servo_setting_command_find = 0;
int servo_setting_command_id[SV_VOL];
short servo_setting_command_command[SV_VOL];

///////// Servo Monitor //////////

#ifdef WALK
///////// Walk ///////////////
static int walk_on = 0;        // 歩行中
static int walk_end = 0;       // 歩行終了中
static int walk_cycle = 0;     // 歩行サイクル
static int walk_step = 0;      // 歩数
static int walk_data = 0;      // データありフラグ
static float walk_x, walk_bx, walk_y;
static int sign_io, sign_stride;
///////// Walk ///////////////
#endif
FLOAT gp[3] = {0.0}, j[24], dk[12];

///////// PID ////////////
PidSet rollFollowRightSidePid;
PidSet rollFollowLeftSidePid;
PidSet rollFollowRightForePid;
PidSet rollFollowLeftForePid;
PidSet balanceControlSidePid;
PidSet balanceControlPitchPid;

PidSet2 gyroFeedbackPid0;
PidSet2 gyroFeedbackPid1;
PidSet2 gyroFeedbackPid2;
PidSet2 zmpFeedbackPid0;
PidSet2 zmpFeedbackPid1;
PidSet2 posCalibrationPid0;
PidSet2 posCalibrationPid1;
PidSet2 comCalibrationPid0;
PidSet2 zmpAlignPid0;
PidSet2 xzmpCalibrationPid0;
PidSet2 xzmpCalibrationPid1;

Lbp gf_lbp;

///////// PID ////////////

//zmp monitoring
ListS walklog;


//////// ZMP Walk ///////////
#define STEP 50
#define TURN_STEP 50
#define RAD  500

FLOAT interval = 0.02;

int s_pivot = BOTH;
int n_pivot = LEFT;

int mode = NOMODE;
FLOAT step = STEP;
FLOAT pace = 0.5;
int n_step = 3;
FLOAT radius = -100;
FLOAT turn_width = 120;

static List zmpplan;
static List zmpplan2;
List zmpplanc;
static List zmpplan_rmk;
static List dzmp;
static List dzmp_rmk;

static Node *zmpp, *zmpp_rmk, *bzmpp;
static FLOAT bzmp[3];  //remake時のZMP差分計算用、捨てるZMPnodeのデータを保管する。
FLOAT bzmpc[3];  //rebuildで使用するbzmp(bzmpはzmp_walk( )で更新される）

static Motion_para mtp;
static Motion_para mtp_fs;
static Motion_para mtp_rsv;
static Motion_para mtp_rmk;
static Prediction pred;
static Prediction pred_rsv;
static Prediction pred_rmk;
Prediction pred_df;

static Lbp lbp_fs, lbp_rsv, lbp_rmk;

Node *cp_node, *rmk_start_node, *cp_nodec;
int cp_pivot;
int cp_space;

Node *np;

FLOAT gangle;
FLOAT gt[3] = { 0, 0, 0 };
FLOAT zx = 0, zy = 0;
FLOAT width = 80;
static int res_pred;  //予見器リセット制御用
int cnt;
static int inres_rsv;

//腕データ [deg]
FLOAT arm_data[3][8] = {
	{ 69, 48, 80, 24, 69, -28, 147, -30 },
	{ -13, 73, 110, 30, 13, -73, 100, -30 },
	{ -69, 28, 147, 30, -69, -48, 80, -24 }
};

FLOAT ini[3] = {0.0, 0.0, 0.0};
static Node *dcp = NULL;
static int dcp_pivot = RIGHT;  //左右どっちでもいいから初期化
static Node *bzmpp_rmk;  // ここではstatic の必要は無いが、実機実装ではスレッド間引き渡しを要する変数となる。

int zmp_walk_on = 0;
int zmp_walk_data_ready = 0;

int zmp_walk_cont_on = 0;  //歩行継続フラグ&ステータス
int rebuild_count;

int cont_step;

int store_log_on = 0;

//歩行補正  body roll adjust は効果が無いのでとりあえずコメントアウト。　その後の判断でソースから削除とする。（20171021）
/*FLOAT body_roll_adj_max = 0.0;   //歩き出しに上体が遅れる事を補正するためにプリテンションをかける　最大角度(rad)
int body_roll_adj_space = 20;
int body_roll_adj_step = 10; //最大角度までのステップ数
int body_roll_adj_stay = 20;
int body_roll_adj_recover = 30; //プリテンションを解くステップ数
int body_roll_adj_count = 0; //カウンター*/

short height_adj_zmp = -1;  //flag

int walk_rest_frame = 0;

//////// ZMP Walk ///////////

//////// GYRO ///////////////
FLOAT gyro_angle[3];
FLOAT gyro_angle_bfr[3] = {0.0f, 0.0f, 0.0f};
//////// GYRO ///////////////

//////// ACC POSTURE /////////
FLOAT acc_angle[3];
//////// ACC POSTURE /////////

//////// WALK STATUS ////////
WalkStatus walkstatus[WS_SIZE];
Lbp bfr_lbp;
int walkstatus_index = 0;
int walkstatus_zmpchange_reject_counter;  //動作初期でddxが乱れるのを無視するカウンター
//////// WALK STATUS ////////

Parameters param;
Parameters *param_ro = (Parameters *)FLASH_PARAM;
//////// WALK STABILIZATION  //////////////////////
//足を上げた途端に体が内側に倒れる事を検知
FLOAT innerfall_mark;       //遊脚を上げるタイミングでの体の傾き
int gyro_feedback_on = 0;
int zmp_feedback_on = 0;
int pos_calibration_on = 0;
int com_calibration_on = 0;
int zmp_align_on = 0;
int xzmp_calibration_on = 0;
int feedback_verbose = 0;
FLOAT zmp_feedback_ankle = 0;
FLOAT zmp_align_amount = 0;
FLOAT xrzmp_calibration_amount = 0;
FLOAT xlzmp_calibration_amount = 0;
#define GYROFEEDBACK_TYPE1

int walk_follow_s = 0;
FLOAT walk_follow_ratio = 1;
int walk_follow = 0;
FLOAT walk_tilt = 0.0f;
//////// WALK STABILIZATION  /////////////////////

//////// motion /////////
motionData *md;
//////// motion /////////

//////// up board ///////
int upboard_no_massage = 0;
long int upm_bfr = 0;
int servo_torque_on;
int sv_rest_frame;
int up_ack = 1;
//////// up board ///////

//////// stability ////////
List stab;
FLOAT stability[3];
int stability_disp_on = 0;
int stability_update = 0;
//////// stability ////////

//////// servo err cmd ///////////
int servo_cmd_err_flag = 0;
int servo_cmd_err = 0;
int servo_cmd_err_id[25];
int servo_cmd_err_cmd[25];
int sv_rom_disp = -1;
char sv_rom_data[100];
//////// servo err cmd ///////////

long frame = 0;

int servo_enable = 0;

char s[200];

char mon_s[200];
int mon_s_flag = 0;

// 空ループでウェイトするルーチン
void Delay(volatile unsigned long delay)
{ 
  while(delay) delay--;
}

int main(void)
{
  int sv;
  int i, k;

  Delay(800000);
  
  // STM32の初期化 クロック設定
  SystemInit();
#if defined(STBEE) || defined(STBEEMINI)
  	  NVIC_SetVectorTable(0x3000, 0);
#else
#ifdef STBEEF4
      ;
#endif
#endif
    
  joint_init();
  for(i = 0; i < SVID_MAX; i++){
    sv_rev_index[i] = -1;
  }
  for(sv = 0; sv < SV_VOL; sv++){
    sv_rev_index[joint[sv].d[JOINT_SVID]] = sv;
  }
  
  lbp_ini(&lbp);
  lbp_ini(&lbp_fs);
  lbp_ini(&lbp_rsv);
  lbp_ini(&lbp_rmk);

  lbp_ini(&gf_lbp);

  lambda_ik_ini();

  //ROBOTの定義
  //質点データ
  base_ini(&robot);
  //右足
  br = branch_new(&robot.blist); //bにはblistの末端が返る。
  m = mp_set(116.106,   28.747, -12.226, -21.009, 45.5, 0, 0,  20155.138, -2211.329, 25100.17, 1938.219, -250.329, 26021.621, 0, AXIS_Z, mp_new(&br->mphead)); //yaw block　→　胴体へ   //20170501構造変更を反映
  m = mp_set(168.787,   13.868, -2.031, -65.606, -8, 0, 0,     195041.771, -1508.572, 186339.845, 3897.973, -44117.813, 116968.446, 1, AXIS_Y, mp_new(&m->child)); //base block　→　YAWブロックへ   //20170501構造変更を反映
  m = mp_set(236.263,   10.107, 0.314, -69.051,   0, 0, 0,     138169.367, -4661.285, 177992.189, -4482.949, -6105.217, 55667.796, -1, AXIS_X, mp_new(&m->child)); //大腿　→　ベースブロックへ
  m = mp_set(63.281,    14.495, -2.206, -2.206,   0, 0, 0,     107809.352, -7613.462, 120855.636, -1845.642, -5322.284, 61185.227, -1, AXIS_X, mp_new(&m->next)); //下腿　→　ベースブロックへ
  m = mp_set(133.156,   17.97, -23.708, -19.061,  0, 0, 0,     35609.089, -1.438, 50921.298, -4783.53, -5003.606, 38537.244, -1, AXIS_X, mp_new(&m->next)); //上下ブロック　→　ベースブロックへ
  m = mp_set(24.823,    53.234, -36.145, 94.813,  0, 0, 0,     82668.101, -350.084, 71471.121, 993.937, 27354.655, 11277.501, -1, AXIS_X, mp_new(&m->next)); //上下リンク　→　ベースブロックへ
  m = mp_set(134.179,   17.004, 19.862, 24.956,   0, 0, 0,     38740.102, 3850.854, 45150.846, -4213.149, -5845.73, 40864.717, -1, AXIS_X, mp_new(&m->next)); //前後ブロック　→　ベースブロックへ
  m = mp_set(22.85,     38.636, -2.752, 85.491,   0, 0, 0,     64398.472, 89.077, 63203.653, -631.644, 4286.28, 1283.62, -1, AXIS_X, mp_new(&m->next)); //前後リンク　→　ベースブロックへ
  m = mp_set(89.059,    1.013, 0.162, 9.479,      15.9, 0, -120, 26898.12, -2824.881, 25673.681, -742.926, -974.377, 25976.259, -1, AXIS_X, mp_new(&m->next)); //足首ブロック　→　ベースブロックへ
  m = mp_set(50.21,     0.01, 0.028, -24.55,      0, 0, 0,     35243.881, 0.013, 15719.492, 0.574, -0.787, 47073.93, 5, AXIS_Y, mp_new(&m->child)); //足ブロック　→　足首ブロックへ
  br->df = 0;
  br->base_leg = MMPM_RIGHTLEG;
  //左足
  br = branch_new(&robot.blist); //bにはblistの末端が返る。
  m = mp_set(116.106, -28.747, -12.226, -21.009, -45.5, 0, 0,   19536.675, 2211.045, 24482.09, -309.127, 1877.136, 26021.238, 8, AXIS_Z, mp_new(&br->mphead)); //yaw block　→　胴体へ   //20170501構造変更を反映
  m = mp_set(168.787, -13.868, -2.031, -65.606,  8, 0, 0,       198562.655, 3993.762, 188373.642, -3322.275, -41696.627, 120263.433, 9, AXIS_Y, mp_new(&m->child)); //base block　→　YAWブロックへ   //20170501構造変更を反映
  m = mp_set(236.263, -10.107,  0.314, -69.051,  0, 0, 0,       140639.429, 4903.564, 180270.389, 4259.39, -5299.021, 56083.351, -1, AXIS_X, mp_new(&m->child)); //大腿　→　ベースブロックへ
  m = mp_set(62.365,  -14.817, -2.29, -55.558,   0, 0, 0,       104790.402, 7889.836, 117307.814, 2407.782, -4684.646, 60716.57, -1, AXIS_X, mp_new(&m->next)); //下腿　→　ベースブロックへ
  m = mp_set(133.156, -17.964, -23.712, -19.059, 0, 0, 0,       35605.644, -1829.087, 46356.485, 5955.795, -7013.627, 43034.919, -1, AXIS_X, mp_new(&m->next)); //上下ブロック　→　ベースブロックへ
  m = mp_set(24.823,  -53.234, -36.15, 94.813,   0, 0, 0,       82668.101, 350.084, 71471.121, -993.937, 27354.655, 11277.501, -1, AXIS_X, mp_new(&m->next)); //上下リンク　→　ベースブロックへ
  m = mp_set(134.561, -17.068, 19.847, 24.91,    0, 0, 0,       38844.628, -2703.961, 40878.968, 2265.833, -3919.286, 45623.409, -1, AXIS_X, mp_new(&m->next)); //前後ブロック　→　ベースブロックへ
  m = mp_set(22.839,  -38.638, -2.753, 85.533,   0, 0, 0,       64316.344, -88.956, 63121.538, 628.139, 4284.479, 1283.292, -1, AXIS_X, mp_new(&m->next)); //前後リンク　→　ベースブロックへ
  m = mp_set(89.122,   2.767, 0.147, 9.47,      -15.9, 0, -120, 26935.402, 1552.973, 25094.306, 1418.525, -989.966, 25415.146, -1, AXIS_X, mp_new(&m->next)); //足首ブロック　→　ベースブロックへ
  m = mp_set(50.21,    0.01, 0.028, -24.55,      0, 0, 0,       35243.881, 0.013, 15719.492, 0.574, -0.787, 47073.93, 13, AXIS_Y, mp_new(&m->child)); //足ブロック　→　足首ブロックへ
  br->df = 0;
  br->base_leg = MMPM_RIGHTLEG;
  
  //胴体
  br = branch_new(&robot.blist); //bにはblistの末端が返る。
  //mp_set(607.657,      0.644, -14.562, 55.171,  0, 0, 0,    1139626.986, -2709.118, 1876449.383, 10033.331, -72788.166, 1620403.592, -1, AXIS_Z, mp_new(&br->mphead)); //胴体   //20170501構造変更を反映
  //mp_set(525.335,      0.655, -15.888, 55.624,  0, 0, 0,    1139626.986, -2709.118, 1876449.383, 10033.331, -72788.166, 1620403.592, -2, AXIS_Z, mp_new(&br->mphead)); //胴体   //20170501構造変更を反映   //バッテリーなし
  //mp_set(524.631,      0.512, -13.856, 58.043,  0, 0, 0,    1139626.986, -2709.118, 1876449.383, 10033.331, -72788.166, 1620403.592, -2, AXIS_Z, mp_new(&br->mphead)); //胴体   //20170501構造変更を反映   //バッテリーなし
  mp_set(1089.248,      0.103, -11.125, 76.971,  0, 0, 0,    1139626.986, -2709.118, 1876449.383, 10033.331, -72788.166, 1620403.592, -2, AXIS_Z, mp_new(&br->mphead)); //胴体   //20170501構造変更を反映   //バッテリーなし
  #if 1
  //右腕    //20170502データ修正
  br = branch_new(&robot.blist); //bにはblistの末端が返る。
  m = mp_set(19.433,   6.571, 0, -0.692,       66.5, 2, 81.75,  0, 0, 0, 0, 0, 0, 16, AXIS_X, mp_new(&br->mphead));
  m = mp_set(146.627,  32.857,-5.988, 0,       22.0, 0, 5,        0, 0, 0, 0, 0, 0, 17, AXIS_Y, mp_new(&m->child));
  m = mp_set(12.427,   9.976, 0, 0,            73.5, 0, 0,        0, 0, 0, 0, 0, 0, 18, AXIS_X, mp_new(&m->child));
  m = mp_set(110.581,   32.352, -0.437, -0.898,  30.0, 0, 0,        0, 0, 0, 0, 0, 0, 19, AXIS_Y, mp_new(&m->child));

  //左腕    //20170502データ修正
  br = branch_new(&robot.blist); //bにはblistの末端が返る。
  m = mp_set(19.433,   -6.571, 0, -0.692,      -66.5, 2, 81.75, 0, 0, 0, 0, 0, 0, 20, AXIS_X, mp_new(&br->mphead));
  m = mp_set(146.627,  -32.857, -5.988, 0,     -22, 0, -5,       0, 0, 0, 0, 0, 0, 21, AXIS_Y, mp_new(&m->child));
  m = mp_set(12.427,   -9.976, 0, 0,           -73.5, 0, 0,       0, 0, 0, 0, 0, 0, 22, AXIS_X, mp_new(&m->child));
  m = mp_set(110.581,   -32.352, -0.437, -0.8988, -30.0, 0, 0,       0, 0, 0, 0, 0, 0, 23, AXIS_Y, mp_new(&m->child));
#endif

  clear_pid(&rollFollowRightSidePid);
  clear_pid(&rollFollowLeftSidePid);
  clear_pid(&rollFollowRightForePid);
  clear_pid(&rollFollowLeftForePid);
  clear_pid(&balanceControlSidePid);
  clear_pid(&balanceControlPitchPid);

  clear_pid2(&gyroFeedbackPid0);
  clear_pid2(&gyroFeedbackPid1);
  clear_pid2(&gyroFeedbackPid2);
  clear_pid2(&zmpFeedbackPid0);
  clear_pid2(&zmpFeedbackPid1);
  clear_pid2(&posCalibrationPid0);
  clear_pid2(&posCalibrationPid1);
  clear_pid2(&comCalibrationPid0);
  clear_pid2(&zmpAlignPid0);
  clear_pid2(&xzmpCalibrationPid0);
  clear_pid2(&xzmpCalibrationPid1);

  /////// ZMP Walk 変数初期化 ////////

  fact_ini(ki, fi, 290.0);
  
  list_ini(&zmpplan);
  list_ini(&zmpplan2);
  list_ini(&zmpplanc);
  list_ini(&zmpplan_rmk);
  list_ini(&dzmp);
  list_ini(&dzmp_rmk);

  for (i = 0; i < 3; i++){
    for (int j = 0; j < 8; j++){
      arm_data[i][j] = arm_data[i][j] * M_PI / 180;
    }
  }

  listS_ini(&walklog);

  /////// ZMP Walk 変数初期化 ////////

  motion_info_ini(&motion_info);
  motion_ini(&motion);
  md = motion.head;

  /////// stability ///////
  list_ini(&stab);
  /////// stability ///////



  servo_on = 0;
  
  flash_load = 0;
  
  tim2_start = 0;
  
  joy_dir[0] = joy_dir[1] = 0;
  joy_mag[0] = joy_mag[1] = 0;
  
  damage_on = 0;
  laser_on = 0;
  batt_check_on = 0;
  damage_check_on = 0;

  test_on = 0;
  test_id = 0;
  test_disp = 0;

  test_stretch = 10;
  test_stretch2 = 120;
  
  func_mess_num = 0;

  //////////////////////////////  parameter initialize  ///////////////////////////////////
  param.i_vol = PARAM_I_MAX;
  param.i_vol_old = PARAM_I_MAX_OLD;
  param.f_vol = PARAM_F_MAX;
  param.f_vol_old = PARAM_F_MAX_OLD;

  param.i[_upboard_mode] = 0;
  param.i[_n_step] = 3;
  param.i[_step] = 1;
  param.i[_delay] = 4;
  param.i[_innerfall_timing] = 2;     //傾きをチェックするフレーム
  param.i[_innerfall_threshold] = 100;  //閾値
  param.f[_innerfall_reaction] = 5.0;

  param.f[_shift_zmp_rx] = 0.0;
  param.f[_shift_zmp_lx] = 0.0;
  param.f[_shift_zmp_ry] = 0.0;
  param.f[_shift_zmp_lx] = 0.0;

  param.f[_default_lbp_width] = 106.8;
  param.f[_default_lbp_depth] = 0.0;
  param.f[_default_lbp_height] = 320.0;
  param.f[_default_lbp_hung] = 0.0;

  param.f[_default_ubp_1] = -30.0;
  param.f[_default_ubp_2] = 70.0;
  param.f[_default_ubp_3] = 80.0;
  param.f[_default_ubp_4] = 60.0;


  param.f[_pace] = 0.5;
  param.f[_hung] = 20;

  param.f[_faststep_arange] = 5.0;
  param.f[_height_adjust] = 2.0;
  param.f[_gtz_adjust] = -80.0;
  param.f[_gtz_adjust_reccati] = 0.0;

  param.i[_d_step] = D_STEP;
  param.i[_cp_space_threshold] = CP_SPACE_THRESHOLD;
  param.i[_gyro_select] = 0;
  param.i[_lp_select] = 0;
  param.i[_lp_mag] = 1000;

  param.i[_gyro_feedback_debug1] = 0;
  param.i[_gyro_feedback_debug2] = 0;

  param.f[_pid0_p_fact] = 0.2f;
  param.f[_pid0_i_fact] = 0.05f;
  param.f[_pid0_d_fact] = 0.05f;
  param.f[_pid0_u_max] = 0.1f;

  param.f[_pid1_p_fact] = 0.3f;
  param.f[_pid1_i_fact] = 0.15f;
  param.f[_pid1_d_fact] = 0.1f;
  param.f[_pid1_u_max] = 0.2f;

  param.f[_pid2_p_fact] = 0.2f;
  param.f[_pid2_i_fact] = 0.05f;
  param.f[_pid2_d_fact] = 0.05f;
  param.f[_pid2_u_max] = 0.1f;

  param.f[_pid3_p_fact] = 0.05f;   // for zmp calibration
  param.f[_pid3_i_fact] = 0.01f;
  param.f[_pid3_d_fact] = 0.005f;
  param.f[_pid3_u_max] = 1.0f;

  param.f[_pid4_p_fact] = 0.05f;   // for zmp calibration
  param.f[_pid4_i_fact] = 0.01f;
  param.f[_pid4_d_fact] = 0.005f;
  param.f[_pid4_u_max] = 0.1f;

  param.f[_pid5_p_fact] = 0.05f;   // for com calibration
  param.f[_pid5_i_fact] = 0.01f;
  param.f[_pid5_d_fact] = 0.005f;
  param.f[_pid5_u_max] = 1.0f;

  param.f[_pid6_p_fact] = 0.05f;   // for zmp align calibration
  param.f[_pid6_i_fact] = 0.01f;
  param.f[_pid6_d_fact] = 0.005f;
  param.f[_pid6_u_max] = 0.1f;

  param.f[_pid7_p_fact] = 0.0f;   // for ankle right roll calibration
  param.f[_pid7_i_fact] = 0.0f;
  param.f[_pid7_d_fact] = 0.005f;
  param.f[_pid7_u_max] = 0.1f;

  param.f[_pid8_p_fact] = 0.0f;   // for ankle left roll calibration
  param.f[_pid8_i_fact] = 0.0f;
  param.f[_pid8_d_fact] = 0.005f;
  param.f[_pid8_u_max] = 0.1f;

  param.f[_gyro_roll_feedback_retio] = 1.0f;
  param.f[_gyro_pitch_feedback_retio] = 1.0f;
  param.f[_pos_pitch_calibration_retio] = 1.0f;
  param.f[_xzmp_right_roll_calibration_retio] = 1.0f;
  param.f[_xzmp_left_roll_calibration_retio] = 1.0f;
  param.f[_zmp_pitch_align_retio] = 1.0f;
  param.f[_pid_y_offset_rate_gyro] = 30.0f;
  param.f[_pid_y_offset_rate_zmp] = -30.0f;

  param.f[_gyro_angle_offset_roll] = -0.1f;
  param.f[_gyro_angle_offset_pitch] = 0.02f;

  param.f[_ankle_pitch_calibration_amount] = 0.0f;
  param.f[_ankle_roll_right_calibration_amount] = 0.0f;
  param.f[_ankle_roll_left_calibration_amount] = 0.0f;
  param.f[_com_calibration_amount] = 0.0f;
  param.f[_zmp_align_amount] = 0.0f;

  param.f[_up_down_action_accel] = 200.0f;

  param.i[_arm_swing_f_1] = (int)param.f[_default_ubp_1];
  param.i[_arm_swing_f_2] = (int)param.f[_default_ubp_2];
  param.i[_arm_swing_f_3] = (int)param.f[_default_ubp_3];
  param.i[_arm_swing_f_4] = (int)param.f[_default_ubp_4];
  param.i[_arm_swing_b_1] = (int)param.f[_default_ubp_1];
  param.i[_arm_swing_b_2] = (int)param.f[_default_ubp_2];
  param.i[_arm_swing_b_3] = (int)param.f[_default_ubp_3];
  param.i[_arm_swing_b_4] = (int)param.f[_default_ubp_4];

  param.f[_walk_follow_max] = 5.0f;
  param.f[_walk_follow_threshold] = 1.5f;
  param.i[_walk_follow_threshold_times] = 4;

  param.f[_criteria_of_standup] = 0.2f;
  param.f[_criteria_of_stable] = 0.000005f;
  param.f[_criteria_of_motion_posture] = 0.1f;

  RCC_Configuration();
  NVIC_Configuration();
  GPIO_Configuration();
  USART_Configuration();
  SPI_Configuration();
  DMA_Configuration();
  ADC_Configuration();  // <=サーキュラーモードを使っているのでDMA設定よりあと
  I2C_Configuration();


  I2C1_ByteWrite(ADXL345, 0x08, 0x31);  //full scale 2g
  I2C1_ByteWrite(ADXL345, 0x08, 0x2d);  //full resolution
  I2C1_ByteWrite(ADXL345, 0x00, 0x2e);  //INT disable
  I2C1_ByteWrite(ITG3200, 0x80, 0x3e);  //reset
  I2C1_ByteWrite(ITG3200, 0x00, 0x15);  //sample rate dev = 0
  I2C1_ByteWrite(ITG3200, 0x18, 0x16);  //full scale 2000deg/sec, LPF 256Hz internal clock 8kHz
  I2C1_ByteWrite(ITG3200, 0x05, 0x17);  //enable
  I2C1_ByteWrite(ITG3200, 0x00, 0x3e);  //reset release

#ifdef QMC5883L
  I2C3_ByteWrite(QMC5883L, 0x01, 0x0b);  //define set/reset period
  I2C3_ByteWrite(QMC5883L, 0x11, 0x09);  //define OSR=512, full scale range=8 gauss, ODR=200Hz, set continuous
#endif

  TIM_Configuration();

  GPIO_SetBits(SPICS);  // SPI CS DISABLE

  send_flg = 0;
  tmcom_wd = 0;
  tmcom_senderr = 0;
  
  histry_pos = histry_most_old = 0;
  histry_read_pos = -1; // -1:histryなし

  esc_mode = 0;
  str_cur = 0;
  cur = 0;

  if(!param_ro->i[_upboard_mode]){
	  sprintf(s, "\r\nlambda04 for STBeeF4mini Program start %s %s\r\n", __DATE__, __TIME__);
	  tmcom_puts(s);

	  RCC_ClocksTypeDef RCC_ClockFreq;
	  RCC_GetClocksFreq(&RCC_ClockFreq);

	  sprintf(s,"SYSCLK_Frequency = %ld\r\n", RCC_ClockFreq.SYSCLK_Frequency );
	  tmcom_puts(s);
	  sprintf(s,"HCLK_Frequency = %ld\r\n", RCC_ClockFreq.HCLK_Frequency );
	  tmcom_puts(s);
	  sprintf(s,"PCLK1_Frequency = %ld\r\n", RCC_ClockFreq.PCLK1_Frequency );
	  tmcom_puts(s);
	  sprintf(s,"PCLK2_Frequency = %ld\r\n", RCC_ClockFreq.PCLK2_Frequency );
	  tmcom_puts(s);
  }

  while(1){
	  if(!param_ro->i[_upboard_mode]){
			if(!tim2_start){
				TIM_Cmd(TIM2, ENABLE);
				tim2_start = 1;
				tmcom_puts("SERVO CONTROL LOOP START\r\n");
			}
		    if(!flash_load && frame > FLASH_LOAD_WAIT){
		      load_flash();
		      flash_load = 1;
		      tmcom_puts("FLASH LOAD\r\n");
		    }
		    if(frame > FLASH_LOAD_WAIT && flash_load && !servo_enable ){
		    	servo_enable = 1;
		        tmcom_puts("SERVO CONTROL START\r\n");
		    }
	  }else{
			if(!tim2_start){
				TIM_Cmd(TIM2, ENABLE);
				tim2_start = 1;
			}
		    if(!flash_load){
		      load_flash();
		      flash_load = 1;
		    }
		    if(frame > SERVO_START_WAIT && flash_load && !servo_enable ){
		  	  load_parameters();
		  	  motion_frame = motion_info_load_flash(&motion_info);
		  	  motion_info_load = 1;
		  	  servo_enable = 1;
		    }
	  }
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE)){
      rchar = USART_ReceiveData(USART1);
      if(esc_mode == 0 && rchar == 27)  //ESC
        esc_mode = 1;
      else if(esc_mode == 1){
        if(rchar == 91)  // [
          esc_mode = 2;
        else
          esc_mode = 0;
      }else if(esc_mode == 2){
        int hist_load = 0;
        if(rchar == 65){ // A ↑
          if(histry_read_pos != histry_most_old){ // (-)できる
            histry_read_pos = (histry_read_pos - 1 + HISTRY_MAX) % HISTRY_MAX;
            hist_load = 1;
          }
        }else if(rchar == 66){ // B ↓
          if(histry_read_pos != histry_pos){  //(+)できる
            histry_read_pos = (histry_read_pos + 1) % HISTRY_MAX;
            hist_load = 1;
          }
        }else if(rchar == 67){ // C →
          if(++cur > str_cur) cur = str_cur;
          else tmcom_puts("\x1b[C");
        }else if(rchar == 68){ // D ←
          if(--cur < 0) cur = 0;
          else tmcom_puts("\x1b[D");
        }
        if(hist_load){
          tmcom_puts("\x0d\x1b[0K");
          str_cur = strlen(comhistry[histry_read_pos]);
          cur = str_cur;
          strcpy(inp_str, comhistry[histry_read_pos]);
          tmcom_puts(comhistry[histry_read_pos]);
        }
        esc_mode = 0;
      }else if(rchar == 8){  //BS
        if(str_cur > 0){
          if(cur > 0){
            tmcom_putc('\x08');
            if(str_cur > cur){
              df = str_cur - cur;
              strncpy(dfstr, &inp_str[cur], df);
              dfstr[df] = 0;
              tmcom_puts(dfstr);
              tmcom_putc(' ');
              tmcom_puts("\x1b[");
              df++;
              if(df > 9){
                df2 = df / 10;
                tmcom_putc(df2 + 0x30);
              }else{
                df2 = 0;
              }
              df2 = df - df2 * 10;
              tmcom_putc(df2 + 0x30);
              tmcom_putc('D');
              memmove(&inp_str[cur - 1], &inp_str[cur], df);
            }else{
              tmcom_puts(" \x08");
            }
          }
          cur--;
          str_cur--;
        }
      }else if(rchar == 0x7f){  // DEL
        if(str_cur > 0 && str_cur > cur){
          df = str_cur - cur - 1;
          strncpy(dfstr, &inp_str[cur + 1], df);
          dfstr[df] = 0;
          tmcom_puts(dfstr);
          tmcom_putc(' ');
          tmcom_puts("\x1b[");
          df++;
          if(df > 9){
            df2 = df / 10;
            tmcom_putc(df2 + 0x30);
          }else{
            df2 = 0;
          }
          df2 = df - df2 * 10;
          tmcom_putc(df2 + 0x30);
          tmcom_putc('D');
          memmove(&inp_str[cur], &inp_str[cur + 1], df);
          str_cur--;
        }
      }else if(rchar == '\r'){
        if(flash_load == 0){
          flash_load = 1;
          strcpy(inp_str, cm_str[CM_UNLOADFLASH]);
          str_cur = strlen(cm_str[CM_UNLOADFLASH]);
        }
        if(servo_mon_req == 1 && str_cur == 0){
          servo_mon_req = 0;
          sprintf(s, "cleared servo_mon_req\r\n");
          tmcom_puts(s);
        }
        if(sensor_disp != 0 && str_cur == 0){
          sensor_disp = 0;
        }
        if(sensor_out_flag == 1 && str_cur == 0){
          sensor_out_flag = 0;
        }
        if(legpressure_row_flag == 1 && str_cur == 0){
          legpressure_row_flag = 0;
        }
        if(stability_disp_on != 0 && str_cur == 0){
          stability_disp_on = 0;
        }
#ifdef WALK
        if(walk_on == 1 && str_cur == 0){
          walk_on = 0;
          walk_end = 1;
        }
#endif
        if(test_on == 1 && str_cur == 0){
          test_end = 1;
        }
        inp_str[str_cur] = 0;
        if(str_cur != 0){
          strcpy(comhistry[histry_pos], inp_str);
          if(++histry_pos == HISTRY_MAX){
            histry_pos = 0;
            histry_most_old = 1;
          }else if(histry_most_old != 0){
            histry_most_old = (histry_most_old + 1) % HISTRY_MAX;
          }
          comhistry[histry_pos][0] = 0;
          histry_read_pos = histry_pos;
#if 0  // histry buffer モニター
          tmcom_outval(histry_top); tmcom_putc(' '); tmcom_outval(histry_read_pos); tmcom_putc(' '); tmcom_outval(histry_pos);
#endif
        }

        tmcom_puts("\r\n");
        str_cur = 0;
        cur = 0;
        cmd_num = 0;
        prm_num = 0;
        inp_str_ptr = strtok_(inp_str, ' ');
        if(inp_str_ptr != NULL){
          while(cmd_num < CM_END){
            if(strcmp(cm_str[cmd_num], inp_str_ptr) == 0){
              break;
            }else{
              cmd_num++;
            }
          }
        }else{
          cmd_num = CM_END;
          strcpy(err_string, inp_str_ptr);
        }
        while((inp_str_ptr = strtok_(NULL, ' ')) != NULL){
          strcpy(prm_str[prm_num], inp_str_ptr);
          if(++prm_num == PRM_MAX) break;
        }
        if(sw_status[SW_PRM_MON - SW_PRM_MON]){
          n = str_cpy((char*)msg, "cmd_num = "); n += outval((char*)&msg[n], cmd_num);
          n += str_cpy((char*)&msg[n], "cm_str[ ] = "); n += str_cpy((char*)&msg[n], cm_str[cmd_num]);
          n += str_cpy((char*)&msg[n], "prm_num = "); n += outval((char*)&msg[n], prm_num);
          add_eol((char*)&msg[n]);
          tmcom_puts((char*)msg);
          for(i = 0; i < prm_num; i++){
            tmcom_puts(prm_str[i]);
            tmcom_puts(" ");
          }
          tmcom_puts("\r\n");
        }
#if 1   /////////////入力コマンド処理 start
        if(cmd_num == CM_SERVOENABLE){
        	if(prm_num == 1){
        		servo_enable = atoi(prm_str[0]);
        	}
        	sprintf(s, "servo_enable = %d\r\n", servo_enable);
        	tmcom_puts(s);
        }else if(cmd_num == CM_SERVOON){
          command = CMD_SERVO_ON;
        }else if(cmd_num == CM_SERVOOFF){
          command = CMD_SERVO_OFF;
        }else if(cmd_num == CM_SERVOZERO){
          command = CMD_SERVO_ZERO;
        }else if(cmd_num == CM_SERVOGET){
          if(prm_num >= 2){
            sv = atoi(prm_str[0]);
            joint[sv].d[JOINT_RTN] = atoi(prm_str[1]);
          }
        }else if(cmd_num == CM_SVGET){
          if(prm_num >= 1){
            for(sv = 0; sv < SV_VOL; sv++){
              joint[sv].d[JOINT_RTN] = atoi(prm_str[0]);
            }
          }
        }else if(cmd_num == CM_SERVO){
          // prm_str[0] : joint id
          // prm_str[1] : on | off | servo position
          // prm_str[2] : time (unit: 20ms) (if need)
          if(prm_num >= 2){
            sv = atoi(prm_str[0]);
            if(strcmp("on", prm_str[1]) == 0){
              if(joint[sv].d[JOINT_STATUS] == SV_OFF){
                joint[sv].d[JOINT_CMD] = SV_ON;
              }
            }else if(strcmp("off", prm_str[1]) == 0){
              joint[sv].d[JOINT_CMD] = SV_OFF;
            }else if(strcmp("connect", prm_str[1]) == 0){
              joint[sv].d[JOINT_STATUS] = SV_OFF;
            }else if(strcmp("disconnect", prm_str[1]) == 0){
              joint[sv].d[JOINT_STATUS] = SV_DISCONNECT;
            }else{
              joint[sv].d[JOINT_TARGET] = atoi(prm_str[1]);
              if(prm_num >= 3){
                joint[sv].d[JOINT_TIME] = atoi(prm_str[2]);
              }
              joint[sv].d[JOINT_CMD] = SV_POS;
            }
          }
        }else if(cmd_num == CM_STRETCH){
            if(prm_num >= 2){
          	  int pn = prm_num;
          	  int np = 0;
          	  while(pn){
                    sv = atoi(prm_str[np++]);
                    if(--pn){
                        joint[sv].d[JOINT_PRM] = atoi(prm_str[np++]);
                        joint[sv].d[JOINT_PRMCMD] = SV_STRETCH;
                  	  pn--;
                    }
          	  }
            }
        }else if(cmd_num == CM_SPEED){
            if(prm_num >= 2){
          	  int pn = prm_num;
          	  int np = 0;
          	  while(pn){
                    sv = atoi(prm_str[np++]);
                    if(--pn){
                        joint[sv].d[JOINT_PRM] = atoi(prm_str[np++]);
                        joint[sv].d[JOINT_PRMCMD] = SV_SPEED;
                  	  pn--;
                    }
          	  }
            }
        }else if(cmd_num == CM_ROM){
          if(prm_num >= 1){
            sv = atoi(prm_str[0]);
            joint[sv].d[JOINT_PRMCMD] = SV_ROM | 0x10;
          }
        }else if(cmd_num == CM_SERVOMON_ON){
          if(prm_num > 0){
            servo_mon_req = 1;
            servo_mon_qt = prm_num;
            for(i = 0; i < servo_mon_qt && i < 14; i++){
            	servo_mon_list[i] = atoi(prm_str[i]);
            }
            servo_mon_title = 1;
          }
        }else if(cmd_num == CM_IK){
          FLOAT ik[6];
          unsigned char r;
          // prm_str[0] : X
          // prm_str[1] : Y
          // prm_str[2] : Z
          // prm_str[3] : RL 1:right 2:left
          // prm_str[4] : time (unit:20ms) timeを指定するとservo on
          if(prm_num >= 4){
            ik[0] = (FLOAT)atoi(prm_str[0]);
            ik[1] = (FLOAT)atoi(prm_str[1]);
            ik[2] = (FLOAT)atoi(prm_str[2]);
            ik[3] = ik[4] = ik[5] = 0;
            r = biarticular_ik(ik, j, atoi(prm_str[3]));
            for(i = 0; i < LEG_SERVO_QT; i++){
              tmcom_puts("j["); tmcom_outval(i); tmcom_puts("] = "); tmcom_outfloat(j[i]);
              tmcom_puts(" "); tmcom_outval(conv_f_sv(j[i])); tmcom_puts("\r\n");
            }
            tmcom_puts("result:"); tmcom_out2h(r); tmcom_puts("\r\n");
          }
          if(prm_num >= 5){
            if(atoi(prm_str[3]) == 1){
              k = LEG_RIGHT;
            }else{
              k = LEG_LEFT;
            }
            for(i = 0; i < LEG_SERVO_QT; i++){
              joint[i + k].d[JOINT_TARGET] = conv_f_sv(j[i]);
              joint[i + k].d[JOINT_TIME] = atoi(prm_str[4]);
              joint[i + k].d[JOINT_CMD] = SV_POS;
            }
          }
        }else if(cmd_num == CM_DK){
          for(i = 0; i < LEGS_SERVO_QT; i++){
            j[i] = conv_sv_f(&joint[i], joint[i].d[JOINT_CAPTURE]);
            tmcom_outfloat(j[i] * 180 / M_PI); tmcom_puts("\r\n");
          }
          m = robot.blist->mphead->child->child;
          biarticular_dk(j, dk, 1, m); // right
          m = robot.blist->next->mphead->child->child;
          biarticular_dk(&j[LEG_LEFT], &dk[LEG_DK_LEFT], 2, m); // left
          sprintf(s, "RIGHT Y:%f Z: %f\r\n", dk[1], dk[2]);
          tmcom_puts(s);
          sprintf(s, "LEFT Y:%f Z: %f\r\n", dk[7], dk[8]);
          tmcom_puts(s);
        }else if(cmd_num == CM_LBP){
          // prm_str[0] : WIDTH
          // prm_str[1] : DEPTH
          // prm_str[2] : HEIGHT
          // prm_str[3] : HUNG
          // prm_str[4] : 重心on/off
          // prm_str[5] : time (unit:20ms) timeを指定するとservo on
          // lbp off
          // lbp default 0 20
          int _gp = 0;
          int _time = 0;
          int _err = 0;
          if(prm_num){
            if(strcmp("default", prm_str[0]) == 0){
              lbp.p[LBP_WIDTH] = param.f[_default_lbp_width];
              lbp.p[LBP_DEPTH] = param.f[_default_lbp_depth];
              lbp.p[LBP_HEIGHT] = param.f[_default_lbp_height];
              lbp.p[LBP_HUNG] = param.f[_default_lbp_hung];
              if(prm_num >= 2){
                _gp = atoi(prm_str[1]);
              }
              if(prm_num >= 3){
                _time = atoi(prm_str[2]);
              }
            }else if(prm_num >= 4){
              lbp.p[LBP_WIDTH] = atof(prm_str[0]);
              lbp.p[LBP_DEPTH] = atof(prm_str[1]);
              lbp.p[LBP_HEIGHT] = atof(prm_str[2]);
              lbp.p[LBP_HUNG] = atof(prm_str[3]);
              if(prm_num >= 5){
                _gp = atoi(prm_str[4]);
              }
              if(prm_num >= 6){
                _time = atoi(prm_str[5]);
              }
            }
          }
          if(prm_num == 1){
            if(strcmp("off", prm_str[0]) == 0){
              _time = 0;
            }
          }
          if(lbp.p[LBP_WIDTH] < 60){
            _err = 1;
            sprintf(s, "width too narrow. [%f]\r\n", lbp.p[LBP_WIDTH]);
          }else if(lbp.p[LBP_WIDTH] > 200){
            _err = 1;
            sprintf(s, "width too wide. [%f]\r\n", lbp.p[LBP_WIDTH]);
          }
          if(lbp.p[LBP_HEIGHT] < 80){
            _err = 1;
            sprintf(s, "height too low. [%f]\r\n", lbp.p[LBP_HEIGHT]);
          }else if(lbp.p[LBP_WIDTH] > 330){
            _err = 1;
            sprintf(s, "width too heigh. [%f]\r\n", lbp.p[LBP_HEIGHT]);
          }
          if(_err){
            tmcom_puts(s);
          }else{
        	//reset gp and offset
            vct_set(0, 0, 0, lbp.g);
            vct_set(0, 0, 0, lbp.o);
            //convert lbp to servo position
            tmcom_puts("==IK==\r\n");
            res = lbp_cnv_l_sv(&lbp, gp, j);
            for(i = 0; i < LEGS_SERVO_QT; i++){
            	sprintf(s, "j[%d] = %f %d %d\r\n", i, j[i], conv_f_sv(j[i]), conv_l_f(&joint[i], conv_f_sv(j[i])));
                tmcom_puts(s);
            }
            tmcom_puts("result:"); tmcom_out4h(res); tmcom_puts("\r\n");
            //calcurate direct kinetic
            int r;
            tmcom_puts("==DK==\r\n");
            m = robot.blist->mphead->child->child;
            r = biarticular_dk(j, dk, 1, m); // right
            m = robot.blist->next->mphead->child->child;
            r = biarticular_dk(&j[LEG_LEFT], &dk[LEG_DK_LEFT], 2, m); // left
            sprintf(s, "RIGHT Y:%f Z: %f\r\n", dk[1], dk[2]);
            tmcom_puts(s);
            sprintf(s, "LEFT  Y:%f Z: %f\r\n", dk[7], dk[8]);
            tmcom_puts(s);
            if(_gp){
              //calcurate center of mass (com)
              tmcom_puts("==COM==\r\n");
              base_get_gp(&robot, j, NO_DYN_FILTER, NO_DYN_FILTER);
              sprintf(s, "COM %f, %f, %f\r\nCOM calibration %f\r\n", robot.zmp[0][X], robot.zmp[0][Y], robot.zmp[0][Z], param.f[_com_calibration_amount]);
              tmcom_puts(s);
              lbp.g[X] -= robot.zmp[0][X];
              lbp.g[Y] -= robot.zmp[0][Y];
              lbp.g[Z] = robot.zmp[0][Z];
              lbp.o[Y] += param.f[_com_calibration_amount];
              res = lbp_cnv_l_sv(&lbp, gp, j);
              lbp.o[Y] -= param.f[_com_calibration_amount];
              m = robot.blist->mphead->child->child;
              r = biarticular_dk(j, dk, 1, m); // right
              m = robot.blist->next->mphead->child->child;
              r = biarticular_dk(&j[LEG_LEFT], &dk[LEG_DK_LEFT], 2, m); // left
              base_get_gp(&robot, j, NO_DYN_FILTER, NO_DYN_FILTER);
              sprintf(s, "g[] %f, %f, %f\r\n", lbp.g[X], lbp.g[Y], lbp.g[Z]);
              tmcom_puts(s);
              sprintf(s, "COM %f, %f, %f\r\n", robot.zmp[0][X] + lbp.g[X], robot.zmp[0][Y] + lbp.g[Y], robot.zmp[0][Z]);
              tmcom_puts(s);
            }
            if(_time){
              j[joint_right_ankle_pitch] += param.f[_ankle_pitch_calibration_amount] + param.f[_zmp_align_amount];
              j[joint_left_ankle_pitch] += param.f[_ankle_pitch_calibration_amount];
              for(i = 0; i < LEGS_SERVO_QT; i++){
                joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
                joint[i].d[JOINT_TIME] = _time;
                joint[i].d[JOINT_CMD] = SV_POS;
              }
            }else{
              if(strcmp("off", prm_str[0]) == 0){
                for(i = 0; i < LEGS_SERVO_QT; i++){
                  joint[i].d[JOINT_CMD] = SV_OFF;
                }
              }
            }
          }
          lbp_print(&lbp);
        }else if(cmd_num == CM_RPY){
        	if(prm_num >= 3){
                lbp.p[LBP_ROLL] = (float)atof(prm_str[0]) * M_PI / 180;
                lbp.p[LBP_PITCH] = (float)atof(prm_str[1]) * M_PI / 180;
                lbp.p[LBP_YAW] = (float)atof(prm_str[2]) * M_PI / 180;
                lbp_cnv_l_sv(&lbp, gp, j);
                for(i = 0; i < LEGS_SERVO_QT; i++){
                    sprintf(s, "j[%d] = %f %d\r\n", i, j[i], conv_f_sv(j[i]));
                    tmcom_puts(s);
                }
            }else if(prm_num == 0){
          	  sprintf(s, "%f %f %f\r\n", lbp.p[LBP_ROLL], lbp.p[LBP_PITCH], lbp.p[LBP_YAW]);
          	  tmcom_puts(s);
        	}
            if(prm_num >= 4){
              for(i = 0; i < LEGS_SERVO_QT; i++){
                joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
                joint[i].d[JOINT_TIME] = atoi(prm_str[3]);
                joint[i].d[JOINT_CMD] = SV_POS;
              }
            }
            lbp_print(&lbp);
        }else if(cmd_num == CM_GETGP){
        	int r;
            res = lbp_cnv_l_sv(&lbp, gp, j);
            m = robot.blist->mphead->child->child;
            r = biarticular_dk(j, dk, 1, m); // right
            m = robot.blist->next->mphead->child->child;
            r = biarticular_dk(&j[LEG_LEFT], &dk[LEG_DK_LEFT], 2, m); // left
			base_get_gp(&robot, j, NO_DYN_FILTER, NO_DYN_FILTER);
            sprintf(s, "g[]   %f, %f\r\n", lbp.g[X], lbp.g[Y]);
            tmcom_puts(s);
            sprintf(s, "GP    %f, %f, %f\r\n", robot.zmp[0][X] + lbp.g[X], robot.zmp[0][Y] + lbp.g[Y], robot.zmp[0][Z]);
            tmcom_puts(s);
        }else if(cmd_num == CM_GPSET){
          // prm_str[0] : gp_x
          // prm_str[1] : gp_y
          // prm_str[2] : gp_z
          // prm_str[3] : time (unit:20ms) timeを指定するとservo on
          if(prm_num >= 3){
            lbp.g[X] = (float)atof(prm_str[0]);
            lbp.g[Y] = (float)atof(prm_str[1]);
            lbp.g[Z] = (float)atof(prm_str[2]);
            lbp_cnv_l_sv(&lbp, gp, j);
            for(i = 0; i < LEGS_SERVO_QT; i++){
                sprintf(s, "j[%d] = %f %d\r\n", i, j[i], conv_f_sv(j[i]));
                tmcom_puts(s);
            }
          }else if(prm_num == 0){
        	  sprintf(s, "%f %f %f\r\n", lbp.g[X], lbp.g[Y], lbp.g[Z]);
        	  tmcom_puts(s);
          }
          if(prm_num >= 4){
            for(i = 0; i < LEGS_SERVO_QT; i++){
              joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
              joint[i].d[JOINT_TIME] = atoi(prm_str[3]);
              joint[i].d[JOINT_CMD] = SV_POS;
            }
          }
          lbp_print(&lbp);
        }else if(cmd_num == CM_OFFSET){
          // prm_str[0] : offset_x
          // prm_str[1] : offset_y
          // prm_str[2] : offset_z
          // prm_str[3] : time (unit:20ms) timeを指定するとservo on
          if(prm_num >= 3){
            lbp.o[X] = (float)atof(prm_str[0]);
            lbp.o[Y] = (float)atof(prm_str[1]);
            lbp.o[Z] = (float)atof(prm_str[2]);
            res = lbp_cnv_l_sv(&lbp, gp, j);
            for(i = 0; i < LEGS_SERVO_QT; i++){
              sprintf(s, "j[%d] = %f %d\r\n", i, j[i], conv_f_sv(j[i]));
              tmcom_puts(s);
            }
          }else if(prm_num == 0){
        	  sprintf(s, "%f %f %f\r\n", lbp.o[X], lbp.o[Y], lbp.o[Z]);
        	  tmcom_puts(s);
          }
          tmcom_puts("result:"); tmcom_out4h(res); tmcom_puts("\r\n");
          if(prm_num >= 4){
            for(i = 0; i < LEGS_SERVO_QT; i++){
              joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
              joint[i].d[JOINT_TIME] = atoi(prm_str[3]);
              joint[i].d[JOINT_CMD] = SV_POS;
            }
          }
          lbp_print(&lbp);
        }else if(cmd_num == CM_UBP){
        	int _time = 0;
            if(prm_num){
              if(strcmp("default", prm_str[0]) == 0){
          		j[LEGS_SERVO_QT] = param.f[_default_ubp_1] * M_PI / 180.0;
          		j[LEGS_SERVO_QT +1] = param.f[_default_ubp_2] * M_PI / 180.0;
          		j[LEGS_SERVO_QT +2] = param.f[_default_ubp_3] * M_PI / 180.0;
          		j[LEGS_SERVO_QT +3] = param.f[_default_ubp_4] * M_PI / 180.0;
                if(prm_num >= 2){
                  _time = atoi(prm_str[1]);
                }
              }else if(prm_num >= 4){
          		j[LEGS_SERVO_QT] = atof(prm_str[0]) * M_PI / 180.0;
          		j[LEGS_SERVO_QT +1] = atof(prm_str[1]) * M_PI / 180.0;
          		j[LEGS_SERVO_QT +2] = atof(prm_str[2]) * M_PI / 180.0;
          		j[LEGS_SERVO_QT +3] = atof(prm_str[3]) * M_PI / 180.0;
                if(prm_num >= 5){
                  _time = atoi(prm_str[4]);
                }
              }
              if(_time){
  				j[LEGS_SERVO_QT + 4] = j[LEGS_SERVO_QT];
  				j[LEGS_SERVO_QT + 5] = -j[LEGS_SERVO_QT + 1];
  				j[LEGS_SERVO_QT + 6] = j[LEGS_SERVO_QT + 2];
  				j[LEGS_SERVO_QT + 7] = -j[LEGS_SERVO_QT + 3];
              }
            }
            if(prm_num == 1){
              if(strcmp("off", prm_str[0]) == 0){
                _time = 0;
              }
            }
        	if(_time){
				for(i = LEGS_SERVO_QT; i < LEGS_SERVO_QT + 8; i++){
				  joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
				  joint[i].d[JOINT_TIME] = _time;
				  joint[i].d[JOINT_CMD] = SV_POS;
				}
        	}else{
				for(i = LEGS_SERVO_QT; i < LEGS_SERVO_QT + 8; i++){
                  joint[i].d[JOINT_CMD] = SV_OFF;
				}
        	}
        	if(prm_num == 0){
        		for(i = LEGS_SERVO_QT; i < LEGS_SERVO_QT + 8; i++){
            		sprintf(s, "%f ", j[i] * 180 / M_PI);
            		tmcom_puts(s);
        		}
        		tmcom_puts("\r\n");
        	}
        }else if(cmd_num == CM_UBP_SET){
        	if(prm_num){
        		if(atoi(prm_str[0]) == 1){
                    for(i = 0; i < 4; i++){
                      param.f[_default_ubp_1 + i] = conv_sv_f(&joint[LEGS_SERVO_QT + i], joint[LEGS_SERVO_QT + i].d[JOINT_CAPTURE]) * 180 / M_PI;
                      sprintf(s, "%d : %f\r\n", i, param.f[_default_ubp_1 + i] );
                      tmcom_puts(s);
                    }
        		}else if(atoi(prm_str[0]) == 2){
                    for(i = 0; i < 4; i++){
                      param.i[_arm_swing_f_1 + i] = (int)(conv_sv_f(&joint[LEGS_SERVO_QT + i], joint[LEGS_SERVO_QT + i].d[JOINT_CAPTURE]) * 180 / M_PI);
                      sprintf(s, "%d : %d\r\n", i, param.i[_arm_swing_f_1 + i] );
                      tmcom_puts(s);
                    }
        		}else if(atoi(prm_str[0]) == 3){
                    for(i = 0; i < 4; i++){
                      param.i[_arm_swing_b_1 + i] = (int)(conv_sv_f(&joint[LEGS_SERVO_QT + i], joint[LEGS_SERVO_QT + i].d[JOINT_CAPTURE]) * 180 / M_PI);
                      sprintf(s, "%d : %d\r\n", i, param.i[_arm_swing_b_1 + i] );
                      tmcom_puts(s);
                    }
        		}else if(strcmp(prm_str[0], "loose") == 0){
        			for(i = 0; i < 8; i++){
        				joint[LEGS_SERVO_QT + i].d[JOINT_PRM] = 10;
                        joint[LEGS_SERVO_QT + i].d[JOINT_PRMCMD] = SV_STRETCH;
        			}
        		}else if(strcmp(prm_str[0], "stiff") == 0){
        			for(i = 0; i < 8; i++){
        				joint[LEGS_SERVO_QT + i].d[JOINT_PRM] = 60;
                        joint[LEGS_SERVO_QT + i].d[JOINT_PRMCMD] = SV_STRETCH;
        			}
        		}
        	}
        }else if(cmd_num == CM_TEST){
          //test [testID]
          // testID = 0 test off
          if(prm_num){
            test_id = atoi(prm_str[0]);
            if(test_id == 1){
              if(prm_num == 1){
                test_on = test_id;
                rollFollowRightSidePid.kp = test_prm[1];
                rollFollowRightSidePid.ki = test_prm[2];
                rollFollowRightSidePid.kd = test_prm[3];
                rollFollowLeftSidePid.kp = test_prm[4];
                rollFollowLeftSidePid.ki = test_prm[5];
                rollFollowLeftSidePid.kd = test_prm[6];
                balanceControlSidePid.kp = test_prm[7];
                balanceControlSidePid.ki = test_prm[8];
                balanceControlSidePid.kd = test_prm[9];
                balanceControlPitchPid.kp = 0;
                balanceControlPitchPid.ki = 0;
                balanceControlPitchPid.kd = 0;
                rollFollowRightForePid.kp = test_prm[10];
                rollFollowRightForePid.ki = test_prm[11];
                rollFollowRightForePid.kd = test_prm[12];
                rollFollowLeftForePid.kp = test_prm[13];
                rollFollowLeftForePid.ki = test_prm[14];
                rollFollowLeftForePid.kd = test_prm[15];
                tmcom_puts("*** TEST ON ***\r\n");
              }
            }else if(test_id == 2){
            }
          }else{
            test_id = 0;
          }
        }else if(cmd_num == CM_TESTPRM){
          // testprm prm0,prm1,prm2,prm3 . . . .
          if(prm_num){
            char *test_prm_p;
            i = 0;
            test_prm_p = strtok_(prm_str[0], ',');
            while(test_prm_p != NULL){
              if(strlen(test_prm_p) || i < PRM_QTY){
                test_prm[i] = atoi(test_prm_p);
              }
              i++;
              test_prm_p = strtok_(NULL, ',');
            }
          }
          for(i = 0; i < PRM_QTY; i++){
            tmcom_puts("test_prm["); tmcom_outval(i); tmcom_puts("]=");
            tmcom_outval(test_prm[i]); tmcom_puts("\r\n");
          }
#ifdef WALK
        }else if(cmd_num == CM_WPS){
          if(prm_num >= 7){
            wps.x_leg_z = (FLOAT)atoi(prm_str[0]);
            wps.y_leg_z = (FLOAT)atoi(prm_str[1]);
            wps.x_body_z = (FLOAT)atoi(prm_str[2]);
            wps.y_body_z = (FLOAT)atoi(prm_str[3]);
            wps.o_z = (FLOAT)atoi(prm_str[4]);
            wps.x_leg_w = (FLOAT)atoi(prm_str[5]);
            wps.x_body_w = (FLOAT)atoi(prm_str[6]);
            wps.y_leg_w = (FLOAT)atoi(prm_str[7]);
            wps.y_body_w = (FLOAT)atoi(prm_str[8]);
          }
          print_walk(&wps);
        }else if(cmd_num == CM_CALCWALK){
          // calcwalk stance body_swing start end time ex. walk 50 0 -50 50 20   mean: 20 * 20ms = 0.4s
          FLOAT x, bx, y;
          if(prm_num >= 5){
            wps.io = (FLOAT)atoi(prm_str[0]);
            wps.body_s = (FLOAT)atoi(prm_str[1]);
            wps.start_y = (FLOAT)atoi(prm_str[2]);
            wps.end_y = (FLOAT)atoi(prm_str[3]);
            wps.w_cycle = atoi(prm_str[4]);
            wps.pace_t = wps.w_cycle * 0.02;
            for(i = 0; i <= wps.w_cycle; i++){
              calc_walk(&wps, i * 0.02, &x, &bx, &y);
              tmcom_outval(i); tmcom_puts(",");
              tmcom_outfloat(i * 0.02); tmcom_puts(",");
              tmcom_outfloat(x); tmcom_puts(",");
              tmcom_outfloat(bx); tmcom_puts(",");
              tmcom_outfloat(y); tmcom_puts("\r\n");
            }
          }
        }else if(cmd_num == CM_WALK){
          // walk stride body_swing time ex. walk 50 10 0 20   mean: 20 * 20ms = 0.4s
          if(prm_num >= 3){
            wps.io = lbp.p[LBP_WIDTH] * 0.5;
            wps.stride  = (FLOAT)atoi(prm_str[0]);
            wps.hung = (FLOAT)atoi(prm_str[1]);
            wps.body_s = (FLOAT)atoi(prm_str[2]);
            wps.w_cycle = atoi(prm_str[3]);
            wps.pace_t = wps.w_cycle * 0.02;
            walk_on = 1;
            walk_cycle = 0;
            walk_step = 0;
          }
#endif
        }else if(cmd_num == CM_WALKLOG){
        	NodeS *wlogn;
        	int st, ed;

        	if(prm_num){
        		int n;
        		n = atoi(prm_str[0]);
        		if(n <= 5){
            		switch(n){
            		case 0:
            			store_log_on = (store_log_on + 1) % 2;
            			sprintf(s, "store_log_on = %d\r\n", store_log_on);
            			tmcom_puts(s);
            			break;
            		case 1:   //servo
            			st = 0;
            			ed = 14;
            			break;
            		case 2:   //capture
            			st = 14;
            			ed = 28;
            			break;
            		case 3:   //gyro
            			st = 28;
            			ed = 34;
            			break;
            		case 4:   //pressure
            			st = 34;
            			ed = 42;
            			break;
            		case 5:   //zmp
            			st = 42;
            			ed = 46;
            			break;
            		default:
            			st = ed = 0;
            		}
                	wlogn = walklog.head;
                	while(wlogn){
                		for(int i = st; i < ed; i++){
                			sprintf(s, "%d,", wlogn->d[i]);
                			tmcom_puts(s);
                		}
            			tmcom_puts("\r\n");
                		wlogn = wlogn->next;
                	}
        		}
        	}

/*        }else if(cmd_num == CM_BODY_ROLL_ADJ){
          // bodyrolladj max step recover
          if(prm_num >= 5){
            body_roll_adj_max = atof(prm_str[0]);
            body_roll_adj_space = atoi(prm_str[1]);
            body_roll_adj_step = atoi(prm_str[2]);
            body_roll_adj_stay = atoi(prm_str[3]);
            body_roll_adj_recover = atoi(prm_str[4]);
          }
          sprintf(s, "max = %f \r\nspace = %d\r\nstep = %d\r\nstay = %d\r\nrecover = %d\r\n", body_roll_adj_max, body_roll_adj_space, body_roll_adj_step, body_roll_adj_stay, body_roll_adj_recover);
          tmcom_puts(s);*/
        }else if(cmd_num == CM_PARAM_I){
          int si = 0;
          int ei = _param_i_end;
          if(prm_num >= 2){
        	  si = atoi(prm_str[0]);
        	  ei = si + prm_num - 1;
        	  for(i = si; i < ei; i++){
        		  param.i[i] = atoi(prm_str[i - si + 1]);
        	  }
          }
    	  for(i = si; i < ei; i++){
    		  sprintf(s, "%d:%s = %d\r\n", i, param_name_i[i], param.i[i]);
    		  tmcom_puts(s);
    	  }
        }else if(cmd_num == CM_PARAM_F){
            int si = 0;
            int ei = _param_f_end;
            if(prm_num >= 2){
          	  si = atoi(prm_str[0]);
          	  ei = si + prm_num - 1;
          	  for(i = si; i < ei; i++){
          		  param.f[i] = atof(prm_str[i - si + 1]);
          	  }
            }
      	  for(i = si; i < ei; i++){
      		  sprintf(s, "%d:%s = %f\r\n", i, param_name_f[i], param.f[i]);
      		  tmcom_puts(s);
      	  }
        }else if(cmd_num == CM_ZMP){
        	FLOAT j_prep[LEGS_SERVO_QT];  // zmp_walk_on = 0 && gyro_feedback_on == 0 の間はj[]が裏で使われるのでj_prop[]を使う。
        	if(prm_num >= 1 && !zmp_walk_on && !motion_play_on){
            	// zmp mode step pace n_step [radius]
               	// zmp mode=1(curve) step pace n_step rdius [left|right]
                // zmp mode=2(turn) step pace n_step [left|right]
          	  if(prm_num > 0){
          		  mode = atoi(prm_str[0]);
          	  }
          	  if(prm_num > 1){
          		  step = atoi(prm_str[1]);
          	  }else{
          		  step = param.i[_step];
          	  }
          	  if(prm_num > 2){
          		  pace = atof(prm_str[2]);
          	  }else{
          		  pace = param.f[_pace];
          	  }
          	  if(prm_num > 3){
          		  n_step = atoi(prm_str[3]);
          	  }else{
          		  n_step = param.i[_n_step];
          	  }
          	  if(prm_num > 4){
          		  if(mode == TURN){
          			  //実行時にn_pivotが現状の逆の足になることに注意
          			  if(strcmp(prm_str[4], "left") == 0){
          				  if(n_pivot == RIGHT){
          					  step = fabs(step);
          				  }else{
          					  step = -fabs(step);
          				  }
          			  }else if(strcmp(prm_str[4], "right") == 0){
          				  if(n_pivot == RIGHT){
          					  step = -fabs(step);
          				  }else{
          					  step = fabs(step);
          				  }
          			  }
          		  }else if(mode == CURVE){
              		  radius = atoi(prm_str[4]);
          		  }
          	  }else{
          		  if(mode == CURVE){
              		  radius = -100; //default
          		  }
          	  }
          	  if(prm_num > 5){
          		  if(mode == CURVE){
          			  if(strcmp(prm_str[5], "left") == 0){
          				  radius = fabs(radius);
          		  	  }else if(strcmp(prm_str[5], "right") == 0){
          				  radius = -fabs(radius);
          		  	  }
          		  }
          	  }
          	  arm_data_set();
          	  sprintf(s, "zmp foresee data start : %ld\r\n", frame);
          	  tmcom_puts(s);
          	  fact_ini(ki, fi, lbp.p[LBP_HEIGHT] + robot.zmp[0][Z]);
          	  sprintf(s, "fact_ini: height set : %f\r\n",lbp.p[LBP_HEIGHT] + robot.zmp[0][Z]);
          	  tmcom_puts(s);
          	  width = lbp.p[LBP_WIDTH];
              FLOAT zmp_start[3] = { 0.0, 0.0, 0.0 };
              motion_para_ini(&mtp);
              motion_para_ini(&mtp_fs);
              motion_para_ini(&mtp_rsv);
              motion_para_ini(&mtp_rmk);
              mtp.id = 1;
              mtp_fs.id = 2;
              mtp_rsv.id = 3;
              mtp_rmk.id = 4;
              prediction_ini(&pred);
              prediction_ini(&pred_rsv);
              prediction_ini(&pred_rmk);
              prediction_ini(&pred_df);
              list_clear(&zmpplan);
              list_clear(&dzmp);

              listS_clear(&walklog);

              n_pivot = (n_pivot + 1) % 2;
              mode = atoi(prm_str[0]);
              //目標ZMP列生成
              if (mode == STRATE){
                tmcom_puts("STRATE:\r\n");
                zmp_make_plan(s_pivot, n_pivot, width, step, pace, n_step, interval, zmp_start, lbp.o, &zmpplan);
              }
              else if (mode == CURVE){
                tmcom_puts("CURVE:\r\n");
                zmp_make_plan_curve(s_pivot, n_pivot, width, step, radius, pace, n_step, interval, zmp_start , lbp.o, &zmpplan);
              }
              else if (mode == TURN){
              	tmcom_puts("TURN:\r\n");
  				if(n_pivot == RIGHT){
  					if(step > 0){
  						tmcom_puts("right turn\r\n");
  					}else{
  						tmcom_puts("left turn\r\n");
  					}
  				}else if(n_pivot == LEFT){
  					if(step > 0){
  						tmcom_puts("left turn\r\n");
  					}else{
  						tmcom_puts("right turn\r\n");
  					}
  				}
  				zmp_make_plan_turn(s_pivot, n_pivot, width, turn_width, step, pace, n_step, interval, zmp_start, lbp.o, &zmpplan);
              }
              else if (mode == SIDE){
              	tmcom_puts("SIDE:\r\n");
                  zmp_make_plan_side(s_pivot, n_pivot, width, step, pace, n_step, interval, zmp_start, lbp.o, &zmpplan);
              }
              //初歩にアレンジ
              np = zmpplan.head;
              FLOAT sx = np->d[X];
              int stp = 0; //処理ステップ
              FLOAT fsa = 0;
              while(np && stp < 2){
              	if(np->d[X] != sx && !stp){
              		stp++;
              		sx = np->d[X];
              		fsa = param.f[_faststep_arange] * sx/fabs(sx);
              		np->d[X] += fsa;
              	}else if(sx == np->d[X] && stp == 1){
              		np->d[X] += fsa;
              	}else if(sx != np->d[X] && stp == 1){
              		stp++;
              	}
              	np = np->next;
              }

  #ifdef USE_ZMP_DF
              //動力学フィルター予見分計算
              Node *dzn, *_dcp;
              FLOAT gt_fs[3];
              int _dcp_pivot;
              zmpp = zmpplan.head;
              bzmpp = zmpplan.head;
              vct_copy(zmpplan.head->d, bzmp);
              lbp_copy(&lbp, &lbp_fs);
              lbp_copy(&lbp, &lbp_rsv);
              inres_rsv = 1; //予備予見器のリセット
              base_get_gp(&robot, j_prep, NO_DYN_FILTER, NO_DYN_FILTER); // 内部変数の初期化
              robot.ini_zmp[0] = 1;
              res_pred = 1; //初回の「zmp_walk_online」で0になる。（変数宣言文で初期化してはダメ）
              for (int i = 0; i < F_STEP; i++){
                zmp_walk_online(interval, zmpp, gt_fs, &zx, &zy, &pred, &res_pred, lbp.o);
                gt_fs[Z] = lbp_fs.p[LBP_HEIGHT] + lbp_fs.o[Z] + param.f[_gtz_adjust];
                //lbp決定
                cnt = zmp_make_walk_motion(gt_fs, zmpp, &lbp_fs, s_pivot, n_pivot, param.f[_hung], param.i[_delay], &gangle, &mtp_fs, &_dcp, &_dcp_pivot);
                //関節角度列j[]決定
  #ifdef ARM_SWING
                arm_swing_action(step, lbp_fs.p[LBP_DEPTH], j_prep, 0);
  #endif
                if((res = lbp_cnv_l_sv(&lbp_fs, gp, j_prep)) != 0){
              	  sprintf(s, "lbp error : %d (initial foresee section)\r\n", res);
              	  tmcom_puts(s);
                }
                //ZMP算出
                m = robot.blist->mphead->child->child;
                biarticular_dk(j_prep, dk, 1, m);
                m = robot.blist->next->mphead->child->child;
                biarticular_dk(&j_prep[LEG_LEFT], &dk[LEG_DK_LEFT], 2, m);
                base_get_zmp(&robot, gt_fs, gangle, j_prep, interval, NOLOAD, FORESEE);  //予見項は[0]を使用
                dzn = list_push_back(&dzmp);
                vct_copy(bzmpp->d, dzn->d);
                vct_sub(robot.zmp[FORESEE], dzn->d); //dzn->d - robot.zmp
                vct_sub(lbp_fs.g, dzn->d);     //dzn->d - lbp.g => Pref - P
                bzmpp = zmpp;
                zmpp = zmpp->next;
              }
  #endif
  //          body_roll_adj_count = 0;
              res_pred = 1;
              zmp_walk_on = 1;
              walk_rest_frame = list_size(&zmpplan);

              //sensor_out_flag = 1;   // verbose on
              lbp_copy(&lbp, &bfr_lbp);  //walkstatus用
              walkStatusInit();
              sprintf(s, "zmp foresee data end : %ld\r\n", frame);
              tmcom_puts(s);
              up_down_action_ini(interval, param.f[_pace]);
        	}
        }else if(cmd_num == CM_CONT || cmd_num == CM_C){
        	if(zmp_walk_on && !zmp_walk_cont_on){
        		zmp_walk_cont_on = 1;
        		if(prm_num > 0){
        			mode = atoi(prm_str[0]);
        		}
        		if(prm_num > 1){
        			step = atoi(prm_str[1]);
        		}else{
        			step = param.i[_step];
        		}
        		if(prm_num > 2){
        			pace = atof(prm_str[2]);
        		}else{
        			pace = param.f[_pace];
        		}
        		if(prm_num > 3){
        			n_step = atoi(prm_str[3]);
        		}else{
        			n_step = param.i[_n_step];
        		}
        		if(prm_num > 4){
				  	radius = atoi(prm_str[4]);
        		}else{
				  	radius = -100;
        		}
        		if(prm_num > 0){
        			mode = atoi(prm_str[0]);
        			if(mode < STRATE || mode > TURN){
        				mode = STRATE;
        			}
        		}
        	}
        }else if(cmd_num == CM_STOP || cmd_num == CM_S){
        	if(zmp_walk_on && !zmp_walk_cont_on){
        		zmp_walk_cont_on = 1;
        		n_step = 0;
        		if(prm_num > 0){
        			mode = atoi(prm_str[0]);
        			if(mode < STRATE || mode > TURN){
        				mode = STRATE;
        			}
        		}
        	}
        }else if(cmd_num == CM_JOINT){
          if(prm_num == 0){
            int joint_d_oder[] ={4,4,6,4,5,5,3,6,4,1,1,5,5,7,6,7,5,6,1,1,1,3,3,3,5,3};
            tmcom_puts("ID SVID SIGN OFFSET TRIM   MAX   MIN CMD TARGET TIME   POS  LPOS CAPTURE STATUS STRETCH SPEED CRRENT RTN FLG ERR ERCNT ICS\r\n");
            for(sv = 0; sv < SV_VOL; sv++){
              tmcom_outvalf(2, ' ', sv);
              tmcom_putc(' ');
              for(i = 0; i < (JOINT_D - 6); i++){  // dummyを除く
                if(i == 9 || i == 10 || i == 18 || i == 19 || i == 20) continue;
                tmcom_outvalf(joint_d_oder[i], ' ',joint[sv].d[i]);
                tmcom_putc(' ');
              }
              tmcom_puts("\r\n");
            }
          }else if(prm_num >= 4){
            int joint_d_oder[] ={4,4,6,3};
            int joint_d_list[] = {JOINT_SVID, JOINT_SIGN, JOINT_OFFSET, JOINT_ICS};
            sv = atoi(prm_str[0]);
            tmcom_puts("ID SVID SIGN OFFSET ICS\r\n");
            tmcom_outvalf(2, ' ', sv);
            tmcom_putc(' ');
            for(i = 0; i < 4; i++){
              joint[sv].d[joint_d_list[i]] = atoi(prm_str[i+1]);
              tmcom_outvalf(joint_d_oder[i], ' ',joint[sv].d[joint_d_list[i]]);
              tmcom_putc(' ');
            }
            tmcom_puts("\r\n");
          }else if(strcmp(prm_str[0], "help") == 0){
            tmcom_puts("joint id svid sign offset \"set joint[id] svid,sign,offset,ics\"\r\n");
          }
        }else if(cmd_num == CM_TRIM){
          short new_trim;
          if(prm_num){
        	  if(strcmp(prm_str[0], "all") == 0){
        		  //all mode
        		  if(prm_num > 1){
    				  sprintf(s, "prm_num =  %d \r\n", prm_num);
        			  tmcom_puts(s);
        			  for(i = 1; i < prm_num && i < (SV_VOL + 1); i++){
        				  sprintf(s, "%s %d %d\r\n", prm_str[i], atoi(prm_str[i]), joint[i - 1].d[JOINT_TRIM]);
            			  tmcom_puts(s);
        				  joint[i - 1].d[JOINT_TRIM] = atoi(prm_str[i]);
        			  }
        		  }
        		  for(i = 0; i < SV_VOL; i++){
        			  sprintf(s, "%d ", joint[i].d[JOINT_TRIM]);
        			  tmcom_puts(s);
        		  }
        		  tmcom_puts("\r\n");
        	  }else{
        		  sv = atoi(prm_str[0]);
                  if(prm_num >= 2) new_trim = atoi(prm_str[1]);
                  else new_trim = -1000;
                  if(sv > -1 && sv < SV_VOL){
                	  if(new_trim != -1000){
                		  joint[sv].d[JOINT_TRIM] = new_trim;
                	  }
                	  sprintf(s, "trim = %d\r\n", joint[sv].d[JOINT_TRIM]);
                	  tmcom_puts(s);
                  }else{
                	  tmcom_puts("servo id error.\r\n");
                  }
        	  }
          }
          else{
            tmcom_puts("trim [SVID | all] [new trim value...]\r\n");
            sv = -1000;
          }
        }else if(cmd_num == CM_LINK_TRIM_SET){
        	//リンク系サーボのトリムセットコマンド
        	if(prm_num){
        		int stp = atoi(prm_str[0]);
        		if(stp == 1){
        			//リンク系サーボのトリムをリセットし、ディスコネクトする。
        			joint[joint_right_hip_pitch].d[JOINT_TRIM] = 0;
        			joint[joint_right_knee].d[JOINT_TRIM] = 0;
        			joint[joint_left_hip_pitch].d[JOINT_TRIM] = 0;
        			joint[joint_left_knee].d[JOINT_TRIM] = 0;
        			joint[joint_right_hip_pitch].d[JOINT_STATUS] = SV_DISCONNECT;
        			joint[joint_right_knee].d[JOINT_STATUS] = SV_DISCONNECT;
        			joint[joint_left_hip_pitch].d[JOINT_STATUS] = SV_DISCONNECT;
        			joint[joint_left_knee].d[JOINT_STATUS] = SV_DISCONNECT;
        			sprintf(s, "Excute \"joint\".\r\nCheck trim=0, status=5(disconnect) for joint[%d,%d,%d,%d].\r\n",joint_right_hip_pitch,joint_right_knee,joint_left_hip_pitch,joint_left_knee);
        			tmcom_puts(s);
        			sprintf(s, "Excute \"rpy 0 0 0\" \"lbp 106.8 0 320 0 0 20\".\r\nAjust trim all servo. And excute \"linktrimset 2\".\r\n");
        			tmcom_puts(s);
        		}else if(stp == 2){
        			int j_list[] = {joint_right_hip_pitch,joint_right_knee,joint_left_hip_pitch,joint_left_knee};
                    lbp.p[LBP_ROLL] = 0;
                    lbp.p[LBP_PITCH] = 0;
                    lbp.p[LBP_YAW] = 0;
                    lbp.p[LBP_WIDTH] = 106.8;
                    lbp.p[LBP_DEPTH] = 0;
                    lbp.p[LBP_HEIGHT] = 320;
                    lbp.p[LBP_HUNG] = 0;
                    vct_set(0, 0, 0, lbp.g);
                    vct_set(0, 0, 0, lbp.o);
                    res = lbp_cnv_l_sv(&lbp, gp, j);
                    for(i = 0; i < 4; i++){
                    	joint[j_list[i]].d[JOINT_TRIM] = joint[j_list[i]].d[JOINT_CAPTURE] - conv_l_f(&joint[j_list[i]], conv_f_sv(j[j_list[i]]));
                    	sprintf(s, "j[%d] : calc:%d capture:%d trim:%d\r\n", j_list[i], conv_l_f(&joint[j_list[i]], conv_f_sv(j[j_list[i]])), joint[j_list[i]].d[JOINT_CAPTURE], joint[j_list[i]].d[JOINT_TRIM]);
                        tmcom_puts(s);
                        tmcom_puts("Excute \"linktrimset 3\", connect link servos.\r\n");
                    }
        		}else if(stp == 3){
        			joint[joint_right_hip_pitch].d[JOINT_STATUS] = SV_OFF;
        			joint[joint_right_knee].d[JOINT_STATUS] = SV_OFF;
        			joint[joint_left_hip_pitch].d[JOINT_STATUS] = SV_OFF;
        			joint[joint_left_knee].d[JOINT_STATUS] = SV_OFF;
        			tmcom_puts("link servos status is CONNECT and OFF.\r\nOK. End of link trim setting.\r\nRemember excute \"saveflash\"");
        		}
        	}
        }else if(cmd_num == CM_LEG_PRESSURE){
          // legpressure
          // センサー値表示
          // 足裏センサー、ZMP、センサーのユニーク値（足裏センサー補正用データ）
          // （ジャイロセンサー、加速度センサーの補正値（未使用））
          // 表示フォーマット
          // Leg Pressure_ = [RLF] [RRF] [RLR] [RRR] [LLF] [LRF] [LLR] [LRR]
          // ZMP = [RX] [RY] [RP] [LX] [LY] [LP]
          // Sensor unique data
          // max : [RLF] [RRF] [RLR] [RRR] [LLF] [LRF] [LLR] [LRR]
          // min : [RLF] [RRF] [RLR] [RRR] [LLF] [LRF] [LLR] [LRR]
          // gyro : 0 0 0 0 0 0 0 0
          // gravity : 0 0 0 0 0 0 0 0

          tmcom_puts("Leg Pressure_ = ");
          for(i = 0; i < 8; i++){
            tmcom_outval(lp_adc[i]);
            tmcom_putc(' ');
          }
          tmcom_puts("\r\n");
          tmcom_puts("ZMP = ");
          for(i = 0; i < 6; i++){
            tmcom_outfloat(zmp[i]);
            tmcom_putc(' ');
          }
          tmcom_puts("\r\n");
          tmcom_puts("Sensor unique data\r\nmax : ");
          for(i = 0; i < 8; i++){
            tmcom_outval(sud.lp_adc_max[i]);
            tmcom_putc(' ');
          }
          tmcom_puts("\r\nmin : ");
          for(i = 0; i < 8; i++){
            tmcom_outval(sud.lp_adc_min[i]);
            tmcom_putc(' ');
          }
          tmcom_puts("\r\n");
          tmcom_puts("gyro : ");
          for(i = 0; i < 8; i++){
            tmcom_outval(sud.gyro[i]);
            tmcom_putc(' ');
          }
          tmcom_puts("\r\ngravity : ");
          for(i = 0; i < 8; i++){
            tmcom_outval(sud.gravity[i]);
            tmcom_putc(' ');
          }
          tmcom_puts("\r\n");
          
        }else if(cmd_num == CM_LEG_PRESSURE_ROW){
          // legpressurerow
          // 足裏センサーの生データ表示
          // センサーのキャリブレーションも行う。
          // キャリブレーション結果の表示は legpressure
          if(!legpressure_row_flag){
            legpressure_row_flag = 1;
            for(i = 0; i < 8; i++){
              sud.lp_adc_max[i] = 0;
              sud.lp_adc_min[i] = 0x7fff;
            }
          }else{
            legpressure_row_flag = 0;
          }
        }else if(cmd_num == CM_GYRO_READ){
          int i, err;
          
          uint8_t accld[6] = { 0,0,0,0,0,0 }, gyrod[6] = { 0,0,0,0,0,0 };
          err = I2C1_BufferRead(ADXL345, accld, 0x32, 6);
          // 0x32:DATAX0 0x33:DATAX1 0x34:DATAY0 0x35:DATAY1 0x36:DATAZ0 0x37:DATAZ1 
          if(err){
            //SensorData[]にエラー用データを入れる
            accld[0] = 255;
            err = 0;
          }
          err = I2C1_BufferRead(ITG3200, gyrod, 0x1d, 6);
          // 1D:GYRO_XOUT_H 1E:GYRO_XOUT_L 1F:GYRO_YOUT_H 20:GYRO_YOUT_L 21:GYRO_ZOUT_H 22:GYRO_ZOUT_L
          if(err){
            //SensorData[]にエラー用データを入れる
            gyrod[0] = 255;
            err = 0;
          }

#ifdef QMC5883L
          char ch;
          err = I2C3_BufferRead(QMC5883L, &ch, 0x06, 1);
          sprintf(s, "ch = %02x\r\n", ch);
          tmcom_puts(s);
          if(ch & 0x01 == 0x01){
              err = I2C3_BufferRead(QMC5883L, compass, 0x00, 6);
          }
#endif
          tmcom_puts("ACCL:");
          for(i = 0; i < 6; i += 2){
            tmcom_out2h(accld[i]);
            tmcom_out2h(accld[i+1]);
          }
          tmcom_puts("\r\n");
          tmcom_puts("GYRO:");
          for(i = 0; i < 6; i += 2){
            tmcom_out2h(gyrod[i]);
            tmcom_out2h(gyrod[i+1]);
          }
          tmcom_puts("\r\n");
#ifdef QMC5883L
          tmcom_puts("COMPASS:");
          for(i = 0; i < 6; i += 2){
            tmcom_out2h(compass[i]);
            tmcom_out2h(compass[i+1]);
          }
          tmcom_puts("\r\n");
          short sd;
          uint8_t *cpnt;
          cpnt = (uint8_t *)(&sd);
          cpnt[0] = compass[0];
          cpnt[1] = compass[1];
          FLOAT x = sd;
          cpnt[0] = compass[2];
          cpnt[1] = compass[3];
          FLOAT y = sd;
          cpnt[0] = compass[4];
          cpnt[1] = compass[5];
          FLOAT z = sd;
          FLOAT l = sqrtf(x * x + y * y);
          FLOAT a = atan2f(y, x);
          FLOAT m = sqrtf(l * l + z * z);
          FLOAT b = asinf(z / m);
          sprintf(s, "X:%f Y:%f Z:%f a:%f b:%f\r\n", x, y, z, a * 180/M_PI, b * 180 / M_PI);
          tmcom_puts(s);
#endif
        }else if(cmd_num == CM_SENSOR_DISP){
          if(prm_num){
            sensor_disp = atoi(prm_str[0]);
          }else{
            sensor_disp = 0;
          }
        }else if(cmd_num == CM_VERBOSE_SENSOR || cmd_num == CM_V_S){
          if(!sensor_out_flag){
        	sensor_out_flag = 1;
          }else{
            sensor_out_flag = 0;
          }
        }else if(cmd_num == CM_SENSOR_PACE || cmd_num == CM_S_P){
          if(prm_num > 0){
            sensor_out_pace = atoi(prm_str[0]);
          }
        }else if(cmd_num == CM_GYRO_FEEDBACK){
        	if(!gyro_feedback_on){
        		clear_pid2(&gyroFeedbackPid0);
        		clear_pid2(&gyroFeedbackPid1);
        		clear_pid2(&gyroFeedbackPid2);
        		gyro_feedback_on = 1;
        		if(prm_num){
        			feedback_verbose = atoi(prm_str[0]);
        		}
        		tmcom_puts("gyro feedback ON\r\n");
        	}else{
        		gyro_feedback_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("gyro feedback OFF\r\n");
        	}
        }else if(cmd_num == CM_ZMP_FEEDBACK){
        	if(!zmp_feedback_on){
        		clear_pid2(&zmpFeedbackPid0); //zmp[LY]
        		clear_pid2(&zmpFeedbackPid1); //pitch
        		zmp_feedback_on = 1;
        		if(prm_num){
        			feedback_verbose = atoi(prm_str[0]);
        		}
        		tmcom_puts("zmp feedback ON\r\n");
        	}else{
        		zmp_feedback_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("zmp feedback OFF\r\n");
        	}
        }else if(cmd_num == CM_CALIBRATION){
        	if(prm_num){
        		if(strcmp(prm_str[0], "com") == 0){  //com calibration
                	if(!com_calibration_on){
                		clear_pid2(&comCalibrationPid0); //zmp[LY]
                		com_calibration_on = 1;
                		tmcom_puts("com calibration ON\r\n");
                	}
        		}else if(strcmp(prm_str[0], "pos") == 0){ //posture calibration
                	if(!pos_calibration_on){
                		clear_pid2(&posCalibrationPid0); //zmp[LY]
                		clear_pid2(&posCalibrationPid1); //pitch
                		pos_calibration_on = 1;
                		tmcom_puts("pos calibration ON\r\n");
                	}
        		}else if(strcmp(prm_str[0], "yzmp") == 0){ //zmp y align
                	if(!zmp_align_on){
                		clear_pid2(&zmpAlignPid0);
                		zmp_align_on = 1;
                		tmcom_puts("zmp align ON\r\n");
                	}
        		}else if(strcmp(prm_str[0], "xzmp") == 0){ //zmp x calibration
                	if(!xzmp_calibration_on){
                		clear_pid2(&xzmpCalibrationPid0); //zmp[RX]
                		clear_pid2(&xzmpCalibrationPid1); //zmp[LX]
                		xzmp_calibration_on = 1;
                		tmcom_puts("xzmp calibration ON\r\n");
                	}
        		}
        		if(prm_num >= 2){
        			feedback_verbose = atoi(prm_str[1]);
        		}
        	}
        }else if(cmd_num == CM_CALIBRATION_END){
        	if(com_calibration_on){
        		com_calibration_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("com calibration OFF\r\n");
        		param.f[_com_calibration_amount] += comCalibrationPid0.u * param.f[_pid_y_offset_rate_zmp];
        		sprintf(s, "com calibration amount : %f pid5(zmp[LY]).umax=%f\r\n", param.f[_com_calibration_amount], param.f[_pid5_u_max]);
        		tmcom_puts(s);
        	}else if(pos_calibration_on){
        		pos_calibration_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("pos calibration OFF\r\n");
        		param.f[_com_calibration_amount] += posCalibrationPid0.u * param.f[_pid_y_offset_rate_zmp];
        		param.f[_ankle_pitch_calibration_amount] += posCalibrationPid1.u;
        		sprintf(s, "com calibration amount : %f pid3(zmp[*Y]).umax=%f\r\n", param.f[_com_calibration_amount], param.f[_pid3_u_max]);
        		tmcom_puts(s);
        		sprintf(s, "anke pitch calibration amount : %f pid4(pitch).umax=%f\r\n", param.f[_ankle_pitch_calibration_amount], param.f[_pid4_u_max]);
        		tmcom_puts(s);
        	}else if(zmp_align_on){
        		zmp_align_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("zmp align OFF\r\n");
        		param.f[_zmp_align_amount] += zmpAlignPid0.u;
        		sprintf(s, "zmp align amount : %f pid6(right anle pitch).umax=%f\r\n", param.f[_zmp_align_amount], param.f[_pid6_u_max]);
        		tmcom_puts(s);
        	}else if(xzmp_calibration_on){
        		xzmp_calibration_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("xzmp calibration OFF\r\n");
        		param.f[_ankle_roll_right_calibration_amount] += xzmpCalibrationPid0.u;
        		param.f[_ankle_roll_left_calibration_amount] += xzmpCalibrationPid1.u;
        		sprintf(s, "xzmp calibration amount : %f pid7(zmp[RX]).umax=%f\r\n", param.f[_ankle_roll_right_calibration_amount], param.f[_pid7_u_max]);
        		tmcom_puts(s);
        		sprintf(s, "xzmp calibration amount : %f pid8(zmp[LX]).umax=%f\r\n", param.f[_ankle_roll_left_calibration_amount], param.f[_pid8_u_max]);
        		tmcom_puts(s);
        	}
/*        }else if(cmd_num == CM_POS_CALIBRATION){
        	if(!pos_calibration_on){
        		clear_pid2(&posCalibrationPid0); //zmp[LY]
        		clear_pid2(&posCalibrationPid1); //pitch
        		pos_calibration_on = 1;
        		if(prm_num){
        			feedback_verbose = atoi(prm_str[0]);
        		}
        		tmcom_puts("pos calibration ON\r\n");
        	}else{
        		pos_calibration_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("pos calibration OFF\r\n");
        		param.f[_ankle_pitch_calibration_amount] += posCalibrationPid1.u;
        		sprintf(s, "anke pitch calibration amount : %f pid3(zmp[LY]).umax=%f, pid4(pitch).umax=%f\r\n", param.f[_ankle_pitch_calibration_amount], param.f[_pid3_u_max], param.f[_pid4_u_max]);
        		tmcom_puts(s);
        		sprintf(s, "offset[Y] ajust amount : %f\r\n", posCalibrationPid0.u * param.f[_pid_y_offset_rate_zmp]);
        		tmcom_puts(s);
        	}
        }else if(cmd_num == CM_COM_CALIBRATION){
        	if(!com_calibration_on){
            	if(gyro_feedback_on || zmp_feedback_on){
            		sprintf(s, "ERROR! gyro feedback(%d) or zmp feedback(%d) is ON\n\r", gyro_feedback_on, zmp_feedback_on);
            		tmcom_puts(s);
            	}else{
            		clear_pid2(&comCalibrationPid0); //zmp[LY]
            		com_calibration_on = 1;
            		if(prm_num){
            			feedback_verbose = atoi(prm_str[0]);
            		}
            		tmcom_puts("com calibration ON\r\n");
            	}
        	}else{
        		com_calibration_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("com calibration OFF\r\n");
        		param.f[_com_calibration_amount] += comCalibrationPid0.u * param.f[_pid_y_offset_rate_zmp];
        		sprintf(s, "com calibration amount : %f pid5(zmp[LY]).umax=%f\r\n", param.f[_com_calibration_amount], param.f[_pid5_u_max]);
        		tmcom_puts(s);
        	}
        }else if(cmd_num == CM_ZMP_ALIGN){
        	if(!zmp_align_on){
            	if(gyro_feedback_on || zmp_feedback_on){
            		sprintf(s, "ERROR! gyro feedback(%d) or zmp feedback(%d) is ON\n\r", gyro_feedback_on, zmp_feedback_on);
            		tmcom_puts(s);
            	}else{
            		clear_pid2(&zmpAlignPid0);
            		zmp_align_on = 1;
            		if(prm_num){
            			feedback_verbose = atoi(prm_str[0]);
            		}
            		tmcom_puts("zmp align ON\r\n");
            	}
        	}else{
        		zmp_align_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("zmp align OFF\r\n");
        		param.f[_zmp_align_amount] += zmpAlignPid0.u;
        		sprintf(s, "zmp align amount : %f pid6(right anle pitch).umax=%f\r\n", param.f[_zmp_align_amount], param.f[_pid6_u_max]);
        		tmcom_puts(s);
        	}
        }else if(cmd_num == CM_XZMP_CALIBRATION){
        	if(!xzmp_calibration_on){
            	if(gyro_feedback_on || zmp_feedback_on){
            		sprintf(s, "ERROR! gyro feedback(%d) or zmp feedback(%d) is ON\n\r", gyro_feedback_on, zmp_feedback_on);
            		tmcom_puts(s);
            	}else{
            		clear_pid2(&xzmpCalibrationPid0); //zmp[RX]
            		clear_pid2(&xzmpCalibrationPid1); //zmp[LX]
            		xzmp_calibration_on = 1;
            		if(prm_num){
            			feedback_verbose = atoi(prm_str[0]);
            		}
            		tmcom_puts("xzmp calibration ON\r\n");
            	}
        	}else{
        		xzmp_calibration_on = 0;
        		feedback_verbose = 0;
        		tmcom_puts("xzmp calibration OFF\r\n");
        		param.f[_ankle_roll_right_calibration_amount] += xzmpCalibrationPid0.u;
        		param.f[_ankle_roll_left_calibration_amount] += xzmpCalibrationPid1.u;
        		sprintf(s, "xzmp calibration amount : %f pid7(zmp[RX]).umax=%f\r\n", param.f[_ankle_roll_right_calibration_amount], param.f[_pid7_u_max]);
        		tmcom_puts(s);
        		sprintf(s, "xzmp calibration amount : %f pid7(zmp[LX]).umax=%f\r\n", param.f[_ankle_roll_left_calibration_amount], param.f[_pid8_u_max]);
        		tmcom_puts(s);
        	}*/
        }else if(cmd_num == CM_WALK_FOLLOW_FRONT){
        	walk_follow = 1;
        	walk_follow_s = 1;
        }else if(cmd_num == CM_WALK_FOLLOW_BACK){
        	walk_follow = -1;
        	walk_follow_s = 1;
        }else if(cmd_num == CM_MOTION || cmd_num == CM_M){
        	if(prm_num){
        		if(strcmp(prm_str[0], "add") == 0 || strcmp(prm_str[0], "a") == 0){
        			if(motion.head == NULL){
        				md = motion_push_back(&motion);
        			}else{
        				md = motion_add(&motion, md);
        			}
            		for(int i = 0; i < SV_VOL; i++){
            			md->frame.dt[i] = 0;
            		}
            		for(int i = 0; i < 2; i++){
            			md->frame.angle[i] = 0;
            		}
            		for(int i = 0; i < 5; i++){
            			md->frame.sw[i] = 0;
            		}
        			md->frame.time = 0;
        		}else if(strcmp(prm_str[0], "del") == 0){
        			if(md){
        				md = motion_del(&motion, md);
        			}
        		}else if(strcmp(prm_str[0], "cap") == 0 || strcmp(prm_str[0], "c") == 0){
        			if(md != NULL){
        				for(int i = 0; i < SV_VOL; i++){
            				md->frame.dt[i] = conv_sv_f3(&joint[i], joint[i].d[JOINT_CAPTURE]);
        				}
    					md->frame.sw[0] = SV_DISCONNECT;
        				md->frame.sw[1] = 0; //all servo
        				for(int i = 2; i < 5; i++){
        					md->frame.sw[i] = 0;
        				}
        				for(int i = 0; i < 2; i++){
            				md->frame.angle[i] = gyro_angle[i];
        				}
        			}else{
        				tmcom_puts("motion error!! please before exec \"motion add\"\r\n");
        			}
        			if(prm_num > 1){
        				int exec = 0;
        				for(int i = 1; i < prm_num; i++){
            				if(strcmp(prm_str[i], "p") == 0){  // cap pause ...
            					exec = 1;
            				}else if(strcmp(prm_str[i], "s") == 0){  // cap sw ...
            					md->frame.sw[0] = SV_OFF;
            				}else{  // cap time
                				md->frame.time = atoi(prm_str[i]);
            				}
        				}
        				if(exec){
                			exec_motion_data(md);
        				}
        			}
        		}else if(strcmp(prm_str[0], "time") == 0 || strcmp(prm_str[0], "t") == 0){
        			if(md != NULL){
            			if(prm_num > 1){
            				md->frame.time = atoi(prm_str[1]);
            			}
        			}
        		}else if(strcmp(prm_str[0], "sw") == 0){
        			if(prm_num >= 2 && md->frame.time != 0){
        				if(strcmp(prm_str[1], "connect") == 0 || strcmp(prm_str[1], "c") == 0){
        					md->frame.sw[0] = SV_OFF;
        				}else if(strcmp(prm_str[1], "disconnect") == 0 || strcmp(prm_str[1], "d") == 0){
        					md->frame.sw[0] = SV_DISCONNECT;
        				}else if(strcmp(prm_str[1], "lbp") == 0){
        					md->frame.sw[2] = 1; //lbp sw
        				}else if(strcmp(prm_str[1], "stab") == 0){
        					md->frame.sw[3] = 1; //stability check cansel sw
        				}else if(strcmp(prm_str[1], "off") == 0){
        					md->frame.sw[4] = 1; //servooff sw
        				}
        			}
        		}else if(strcmp(prm_str[0], "name") == 0 || strcmp(prm_str[0], "n") == 0){
        			if(md != NULL){
            			if(prm_num > 1){
            				strcpy(motion.id.name, prm_str[1]);
            			}
        			}
        		}else if(strcmp(prm_str[0], "pause") == 0 || strcmp(prm_str[0], "p") == 0){
        			exec_motion_data(md);
        		}else if(strcmp(prm_str[0], "next") == 0 || strcmp(prm_str[0], "x") == 0){
        			if(md->next){
            			md = md->next;
            			if(prm_num == 1){
            				exec_motion_data(md);
            			}
        			}
        		}else if(strcmp(prm_str[0], "prev") == 0 || strcmp(prm_str[0], "v") == 0){
        			if(md->prev){
            			md = md->prev;
            			if(prm_num == 1){
            				exec_motion_data(md);
            			}
        			}
        		}else if(strcmp(prm_str[0], "top") == 0){
        			md = motion.head;
        			if(m){
            			if(prm_num == 1){
            				exec_motion_data(md);
            			}
        			}
        		}else if(strcmp(prm_str[0], "bottom") == 0){
        			md = motion.tail;
        			if(m){
            			if(prm_num == 1){
            				exec_motion_data(md);
            			}
        			}
        		}else if(strcmp(prm_str[0], "play") == 0){
        			if(prm_num > 1 && motion_info_load){
        				motion_play_id = atoi(prm_str[1]);
        				motion_play_time = 0;
        				motion_play_on = -1;
        			}else{
        				if(motion.head){
            				motion_play_id = -1;
            				motion_play_time = 0;
            				motion_play_on = -1;
        				}
        			}
        		}else if(strcmp(prm_str[0], "off") == 0 || strcmp(prm_str[0], "o") == 0){
        			if(prm_num > 1){
        				if(strcmp(prm_str[1], "rl") == 0){
        	        		for(int i = 0; i < LEG_LEFT; i++){
        	        			joint[i].d[JOINT_CMD] = SV_OFF;
        	        		}
        				}else if(strcmp(prm_str[1], "ll") == 0){
        	        		for(int i = LEG_LEFT; i < LEGS_SERVO_QT; i++){
        	        			joint[i].d[JOINT_CMD] = SV_OFF;
        	        		}
        				}else if(strcmp(prm_str[1], "ra") == 0){
        	        		for(int i = LEGS_SERVO_QT; i < LEGS_SERVO_QT+4; i++){
        	        			joint[i].d[JOINT_CMD] = SV_OFF;
        	        		}
        				}else if(strcmp(prm_str[1], "la") == 0){
        	        		for(int i = LEGS_SERVO_QT+4; i < SV_VOL; i++){
        	        			joint[i].d[JOINT_CMD] = SV_OFF;
        	        		}
        				}
        			}
        		}else if(strcmp(prm_str[0], "mirror") == 0 || strcmp(prm_str[0], "m") == 0){
        			if(prm_num > 1){
        				if(strcmp(prm_str[1], "leg") == 0){
        				}else if(strcmp(prm_str[1], "arm") == 0){
        				}
        			}
        		}else if(strcmp(prm_str[0], "save") == 0){
        			if(motion.head){
        				if(motion_info_load){
                			motion_frame = motion_save_flash(&motion_info, &motion);
        				}else{
        					tmcom_puts("motion error!! motion no load. please exec \"motion load\"\r\n");
        				}
        			}else{
        				tmcom_puts("motion empty no save.\r\n");
        			}
        		}else if(strcmp(prm_str[0], "load") == 0){
        			motion_frame = motion_info_load_flash(&motion_info);
        			motion_info_load = 1;
        			sprintf(s, "motion_frame=%lx\r\n", motion_frame);
        			tmcom_puts(s);
        			tmcom_puts("name frames\r\n");
        			for(int i = 0; i < motion_info.motions; i++){
        				sprintf(s, "%d : [%s] %dframes\r\n", i, motion_info.table[i].name, motion_info.table[i].steps);
        				tmcom_puts(s);
        			}
        		}else if(strcmp(prm_str[0], "list") == 0){
        			int id;
        			if(prm_num > 1 && motion_info_load){
        				id = atoi(prm_str[1]);
        				if(id < motion_info.motions){
            				sprintf(s, "%d : [%s] %dframes\r\n", id, motion_info.table[id].name, motion_info.table[id].steps);
            				tmcom_puts(s);
            				for(int i = 0; i < motion_info.table[id].steps; i++){
            					sprintf(s, "%2d:", i);
            					tmcom_puts(s);
            					for(int j = 0; j < SV_VOL; j++){
            						sprintf(s,"%-4d ", motion_frame[motion_info.table[id].start + i].dt[j]);
            						tmcom_puts(s);
            					}
            					for(int j = 0; j < 2; j++){
            						sprintf(s,"%f ", motion_frame[motion_info.table[id].start + i].angle[j]);
            						tmcom_puts(s);
            					}
            					for(int j = 0; j < 5; j++){
            						sprintf(s,"%d ", motion_frame[motion_info.table[id].start + i].sw[j]);
            						tmcom_puts(s);
            					}
            					sprintf(s, "%d\r\n", motion_frame[motion_info.table[id].start + i].time);
            					tmcom_puts(s);
            				}
        				}
        			}else{
        				sprintf(s, "motion : [%s] %dframes\r\n", motion.id.name, motion_size(&motion));
        				tmcom_puts(s);
        				motionData *m = motion.head;
        				int i = 0;
        				while(m){
        					sprintf(s, "[%6xl]:", m);
        					tmcom_puts(s);
        					for(int j = 0; j < SV_VOL; j++){
        						sprintf(s,"%-4d ", m->frame.dt[j]);
        						tmcom_puts(s);
        					}
        					for(int j = 0; j < 5; j++){
        						sprintf(s,"%d ", m->frame.sw[j]);
        						tmcom_puts(s);
        					}
        					for(int j = 0; j < 2; j++){
        						sprintf(s,"%f ", m->frame.angle[j]);
        						tmcom_puts(s);
        					}
        					sprintf(s, "%d\r\n", m->frame.time);
        					tmcom_puts(s);
        					m = m->next;
        				}
        			}
        		}else if(strcmp(prm_str[0], "edit") == 0){
        			if(prm_num > 1 || motion_info_load){
        				motion_edit_id = atoi(prm_str[1]);
        				if(motion_edit_id < motion_info.motions){
        					motion_clear(&motion);
            				sprintf(s, "%d : [%s] %dframes\r\n", motion_edit_id, motion_info.table[motion_edit_id].name, motion_info.table[motion_edit_id].steps);
            				tmcom_puts(s);
            				for(int i = 0; i < motion_info.table[motion_edit_id].steps; i++){
            					motion_push_back(&motion);
            					memcpy(&motion.tail->frame, &motion_frame[motion_info.table[motion_edit_id].start + i], sizeof(motionFrame));
            					sprintf(s, "%d / %d [%xl]\r\n", i, motion_info.table[motion_edit_id].steps, motion.tail);
            					tmcom_puts(s);
            				}
            				md = motion.head;
            				strcpy(motion.id.name, motion_info.table[motion_edit_id].name);
        				}else{
        					tmcom_puts("motion id error.\r\n");
        				}
        			}
        		}else if(strcmp(prm_str[0], "table") == 0){
        			if(motion_info_load){
        				sprintf(s, "version:%d  %d motions\r\n", motion_info.version, motion_info.motions);
        				tmcom_puts(s);
        				for(int i = 0; i < motion_info.motions; i++){
        					sprintf(s, "%d : [%s] %d => %dframes\r\n", i, motion_info.table[i].name, motion_info.table[i].start, motion_info.table[i].steps);
            				tmcom_puts(s);
        				}
        			}else{
        				tmcom_puts("motion error!! motion un load.\r\n");
        			}
        		}else if(strcmp(prm_str[0], "new") == 0){
        			motion_edit_id = motion_info.motions;
        			motion_clear(&motion);
        			md = NULL;
        			sprintf(s, "new motion id : %d\r\n", motion_edit_id);
        		}else if(strcmp(prm_str[0], "disp") == 0){
        		}else{
        			tmcom_puts("motion command syntax error.\r\n");
        		}
        		tmcom_puts("==motion info==\r\n");
        		if(motion_info_load){
            		sprintf(s,"version:%d\r\nmotions:%d\r\n\r\n", motion_info.version, motion_info.motions);
            		tmcom_puts(s);
        		}else{
        			tmcom_puts("motion NO LOAD\r\n");
        		}
        		sprintf(s,"edit:%d\r\n", motion_edit_id);
        		tmcom_puts(s);
        		if(md){
            		sprintf(s,"motion name : %s\r\n", motion.id.name);
            		tmcom_puts(s);
            		motionData *n = motion.head;
            		int nm = 1;
            		while(n && n != md){
            			n = n->next;
            			nm++;
            		}
            		sprintf(s, "frame %d / %d\r\n", nm, motion_size(&motion));
            		tmcom_puts(s);
            		sprintf(s, "md : %08lx\r\n", md);
            		tmcom_puts(s);
            		tmcom_puts("dt[]\r\n");
            		tmcom_puts("right : ");
            		for(int i = 0; i < LEG_LEFT; i++){
            			sprintf(s, "%d ", md->frame.dt[i]);
            			tmcom_puts(s);
            		}
            		tmcom_puts("\r\nleft : ");
            		for(int i = LEG_LEFT; i < LEGS_SERVO_QT; i++){
            			sprintf(s, "%d ", md->frame.dt[i]);
            			tmcom_puts(s);
            		}
            		tmcom_puts("\r\narm : ");
            		for(int i = LEGS_SERVO_QT; i < SV_VOL; i++){
            			sprintf(s, "%d ", md->frame.dt[i]);
            			tmcom_puts(s);
            		}
            		tmcom_puts("\r\nangle : ");
            		for(int i = 0; i < 2; i++){
            			sprintf(s, "%f ", md->frame.angle[i]);
            			tmcom_puts(s);
            		}
            		tmcom_puts("\r\nsw : ");
            		for(int i = 0; i < 5; i++){
            			sprintf(s, "%d ", md->frame.sw[i]);
            			tmcom_puts(s);
            		}
        			sprintf(s, "\r\ntime : %d ", md->frame.time);
        			tmcom_puts(s);
        			tmcom_puts("\r\n");
        		}else{
        			tmcom_puts("motion empty.\r\n");
        		}
        	}else{
        		tmcom_puts("== motion help ==\r\n");
        		tmcom_puts(" add | a => add frame\r\n");
        		tmcom_puts(" del => delete frame\r\n");
        		tmcom_puts(" cap | c  [p(pause) |& s(sw on) |& {time} ]=> capture frame\r\n");
        		tmcom_puts(" time | t\r\n name | n\r\n sw => sw flag on\r\n");
        		tmcom_puts(" pause | p\r\n next | x [0]\r\n prev | v [0]\r\n");
        		tmcom_puts(" off | o  rl|ll|ra|la\r\n sw =>sw flag on\r\n ");
        		tmcom_puts(" save\r\n load\r\n edit\r\n new\r\n");
        	}
        }else if(cmd_num == CM_MOTION_STOP){
        	motion_emergency_stop = 1;
        	motion_rest_frame = 0;
        }else if(cmd_num == CM_UP){
        	if(!upboard_no_massage){
            	upboard_no_massage = 1;
        	}else{
            	upboard_no_massage = 0;
        	}
        }else if(cmd_num == CM_UP_ACK){
        	up_ack = 1;
#if 0
        }else if(cmd_num == CM_BATTERY){
          n = 0;
          n += str_cpy((char*)&msg[n], "BATT = ");
          n += outval((char*)&msg[n], ADC1ConvertedValue[7]);
          msg[n++] = ' ';
          n += str_cpy((char*)&msg[n], "limit voltage:");
          n += outval((char*)&msg[n], base.battery_limit);
          n += str_cpy((char*)&msg[n], "\r\n");
          add_eol((char*)&msg[n]);
          tmcom_puts((char*)msg);
        }else if(cmd_num == CM_BATTERY_LIMIT){
          if(prm_num > 0){
            base.battery_limit = atoi(prm_str[0]);
          }
          tmcom_puts("battery limit : ");
          n = outval((char*)msg, base.battery_limit);
          add_eol((char*)&msg[n]);
          tmcom_puts((char*)msg);
#endif
        }else if(cmd_num == CM_STABILITY){
        	if(!stability_disp_on){
        		stability_disp_on = 1;
        	}else{
        		stability_disp_on = 0;
        	}
        }else if(cmd_num == CM_HELP){
          for(i = 0; i < CM_END; i++){
            tmcom_puts(cm_str[i]);
            tmcom_puts("\r\n");
          }
        }else if(cmd_num == CM_SAVEFLASH){
          save_flash();
        }else if(cmd_num == CM_LOADFLASH){
          load_flash();
        }else if(cmd_num == CM_LOADPARAM){
          load_parameters();
        }else if(cmd_num == CM_UNLOADFLASH){
          tmcom_puts("FLASH UNLOAD\r\n");
        }else if(cmd_num == CM_FLASHDISP){
          int i, j;
          uint32_t flashaddr;
          flashaddr = 0x080FF000;
          for(i = 0; i < 0x10; i++){
            n = 0;
            for(j = 0; j < 0x10; j += 2){
              n += out4h((char*)&msg[n], *(__IO uint16_t*)flashaddr);
              flashaddr += 2;
            }
            add_eol((char*)&msg[n]);
            tmcom_puts((char*)msg);
            Delay(2000);
          }
        }else if(cmd_num == CM_INFORMATION){
          tmcom_puts(">> switch status : \r\n");
          for(i = 0; i < SW_NUM; i++){
            show_sw_status(i + SW_PRM_MON);
            //tmcom_puts(cm_str[i + SW_PRM_MON]); tmcom_puts(" : "); tmcom_outval(sw_status[i]); tmcom_putc('\n');
          }
          tmcom_puts("\r\n");
        }else if(cmd_num >= SW_PRM_MON && cmd_num < CM_END){
          int sw_name = cmd_num - SW_PRM_MON;
          sw_status[sw_name] = ++sw_status[sw_name] % 2;
          show_sw_status(cmd_num);
        }else{
          sprintf(s, "not command [%s]\r\n", err_string);
          tmcom_puts(s);
        }
        if(syntax_err > -1){
        	sprintf(s, "usage : %s\r\n", cm_help[syntax_err]);
        	tmcom_puts(s);
        	syntax_err = -1;
        }
#endif
          /////////////入力コマンド処理 end
      }else{
        if(str_cur < LINEINPUT_MAX){
          if(str_cur == cur){
            inp_str[str_cur] = rchar;
            if(!param_ro->i[_upboard_mode] || param_ro->i[_upboard_mode] && upboard_no_massage){
                tmcom_putc(inp_str[str_cur]);
            }
          }else{
            df = str_cur - cur;
            strncpy(dfstr, &inp_str[cur], df);
            dfstr[df] = 0;
            tmcom_putc(rchar);
            tmcom_puts(dfstr);
            tmcom_puts("\x1b[");
            if(df > 9){
              df2 = df / 10;
              tmcom_putc(df2 + 0x30);
            }else{
              df2 = 0;
            }
            df2 = df - df2 * 10;
            tmcom_putc(df2 + 0x30);
            tmcom_putc('D');
            memmove(&inp_str[cur + 1], &inp_str[cur], df);
            inp_str[cur] = rchar;
          }
          str_cur++;
          cur++;
        }
      }
    }else{
      if(func_mess_num > 0){
        int i;
        for(i = 0; i < func_mess_num; i++){
          tmcom_puts(func_mess_name[func_mess[i]]);
        }
        func_mess_num = 0;
      }
      if(mon_s_flag){
    	  tmcom_puts(mon_s);
    	  mon_s_flag = 0;
      }
      if(param_ro->i[_upboard_mode] && servo_enable){
    	  //information_send
    	  if(!upboard_no_massage && upm_bfr != frame){
        	  int stable;
    		  int servo_live = 1;
    		  servo_torque_on = 0;
    		  sv_rest_frame = 0;
    		  for(int i = 0; i < SV_VOL; i++){
    			  if(joint[i].d[JOINT_STATUS] > SV_OFF && joint[i].d[JOINT_STATUS] < SV_DISCONNECT){
    				  servo_torque_on++;
    			  }
    			  if(joint[i].d[JOINT_STATUS] == SV_ON && joint[i].d[JOINT_TIME] > sv_rest_frame){
    				  sv_rest_frame = joint[i].d[JOINT_TIME];
    			  }
    			  if(joint[i].d[JOINT_STATUS] < SV_OFF){
    				  servo_live = 0;
    			  }
    		  }
    		  if(stability[X] > param.f[_criteria_of_stable] || stability[Y] > param.f[_criteria_of_stable]){
    			  stable = 0;
    		  }else{
    			  stable = 1;
    		  }
    		  if(servo_live){
            	  sprintf(s,"%s,%ld,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", "lambda", frame, gyro_angle[X], gyro_angle[Y], gyro_angle[Z], servo_torque_on, zmp_walk_on, motion_play_on, motion_play_id, sv_rest_frame, walk_rest_frame, motion_rest_frame, stable, motion_emergency_stop );
            	  tmcom_puts(s);
    		  }else{
    			  tmcom_puts("servos still sleep..\r\n");
    		  }
        	  upm_bfr = frame;
    	  }
      }
      if(test_disp && frame % sensor_out_pace == 0){
        int i;
        int n = 0;
        if(frame != sensor_out_frame){
          sensor_out_frame = frame;
          msg[n++] = '<';
          n += outval((char*)&msg[n], zmp[ZMP_RX] * 1000); msg[n++] = ' ';
          n += outval((char*)&msg[n], zmp[ZMP_RY] * 1000); msg[n++] = ' ';
          n += outval((char*)&msg[n], zmp[ZMP_LX] * 1000); msg[n++] = ' ';
          n += outval((char*)&msg[n], zmp[ZMP_LY] * 1000); msg[n++] = ' ';
          n += outval((char*)&msg[n], rollFollowRightSidePid.u * 10); msg[n++] = ' ';
          n += outval((char*)&msg[n], rollFollowLeftSidePid.u * 10); msg[n++] = ' ';
          n += outval((char*)&msg[n], (zmp[ZMP_RP] - zmp[ZMP_LP]) * 1000); msg[n++] = ' ';
          n += outval((char*)&msg[n], balanceControlSidePid.u * 10); msg[n++] = ' ';
          for(i = 0; i < 3; i++){
            n += outval((char*)&msg[n], *(short*)&accld[i*2]);
            msg[n++] = ' ';
          }
          for(i = 0; i < 3; i++){
            n += outval((char*)&msg[n], *(short*)&gyrod[i*2]);
            msg[n++] = ' ';
          }
          n += outval((char*)&msg[n], frame);
          msg[n++] = '>';
          add_eol((char*)&msg[n]);
          tmcom_puts((char*)msg);
          test_disp =0;
        }
      }
#ifdef WALK
      if(walk_data){
        tmcom_outval(walk_cycle); tmcom_putc(',');
        tmcom_outval(walk_step); tmcom_putc(',');
        tmcom_outfloat(walk_x); tmcom_putc(',');
        tmcom_outfloat(walk_bx); tmcom_putc(',');
        tmcom_outfloat(walk_y); tmcom_putc(',');
        tmcom_outfloat(lbp.p[LBP_DEPTH]); tmcom_putc(',');
        tmcom_outfloat(lbp.o[X]); tmcom_putc(',');
        tmcom_outfloat(lbp.o[Y]); tmcom_putc(',');
        tmcom_outval(sign_io); tmcom_putc(',');
        tmcom_outval(sign_stride); tmcom_putc(',');
        tmcom_puts("\r\n");
        walk_data =0;
      }
#endif
      if(tmcom_senderr){
        tmcom_puts("TMCOMSENDERR\r\n");
        tmcom_senderr = 0;
      }
      if(zmp_walk_on){
          if(servo_mon_disp != servo_mon_disp_before){
        	  if(servo_mon_title == 1){
                  for(i = 0; i < servo_mon_qt && i < 14; i++){
                  	sprintf(s,"T%d, C%d, ", servo_mon_list[i], servo_mon_list[i]);
                  	tmcom_puts(s);
                  }
                  tmcom_puts("\r\n");
                  servo_mon_title = 0;
        	  }
        	  for( i = 0; i < servo_mon_qt; i++){
        		  sprintf(s, "%f,%f,", conv_sv_f2(joint[servo_mon_list[i]].d[JOINT_TARGET]),conv_sv_f(&joint[servo_mon_list[i]],joint[servo_mon_list[i]].d[JOINT_CAPTURE]));
        		  tmcom_puts(s);
        	  }
        	  tmcom_puts("\r\n");
        	  servo_mon_disp_before = servo_mon_disp;
          }
      }
      if(sensor_disp){
        int i;
        if(frame != sensor_disp_frame && !sensor_read_flag){
          sensor_disp_frame = frame;
          if(sensor_disp == 1){
            tmcom_puts("ACCL:");
            for(i = 0; i < 3; i++){
              tmcom_outvalf(4, ' ', *(short*)&accld[s_idx[i] * 2] * s_sgn[i]);
            }
            tmcom_puts(" GYRO:");
            for(i = 0; i < 3; i++){
              tmcom_outvalf(5, ' ', (*(short*)&gyrod[s_idx[i] * 2] - (short)param.f[4 + s_idx[X]]) * s_sgn[i]);
            }
            tmcom_puts("\r\n");
          }else if(sensor_disp == 2){
            tmcom_puts("LR:");
            short total_lp = 0;
            for(i = 0; i < 8; i++){
              total_lp += lp_adc[i];
            }
            short left_lp = 0, right_lp = 0;
            for(i = 0; i < 4; i++){
              right_lp += lp_adc[i];
              left_lp += lp_adc[i+4];
            }
            short front_lp = 0, back_lp = 0;
            for(i = 0; i < 2; i++){
              front_lp += lp_adc[i] + lp_adc[i+4];
              back_lp += lp_adc[i+2] + lp_adc[i+6];
            }
            short unit_lp = total_lp / 10;
            short point_lp = 0;
            int n = 0;
            while(point_lp < right_lp){
              msg[n++] = '-';
              point_lp += unit_lp;
            }
            msg[n++] = '|';
            while(point_lp < total_lp){
              msg[n++] = '-';
              point_lp += unit_lp;
            }
            n += str_cpy((char*)&msg[n], " FB:");
            point_lp = 0;
            while(point_lp < back_lp){
              msg[n++] = '-';
              point_lp += unit_lp;
            }
            msg[n++] = '|';
            while(point_lp < total_lp){
              msg[n++] = '-';
              point_lp += unit_lp;
            }
            add_eol((char*)&msg[n]);
            tmcom_puts((char*)msg);
          }
        }
      }
      if((sensor_out_flag || legpressure_row_flag) && frame % sensor_out_pace == 0){
        int i;
        int n = 0;
        if(frame != sensor_out_frame && !sensor_read_flag){
          sensor_out_frame = frame;
          msg[n++] = '<';
          for(i = 0; i < 8; i++){
        	  switch(param.i[_lp_select]){
        	  case 0:
        		  n += outval((char*)&msg[n], lp_adc[i]);
        		  break;
        	  case 1:
        		  if(i < 6){
        			  if(i != 2 && i != 5 ){
        				  n += outval((char*)&msg[n], (int)(zmp[i] * param.i[_lp_mag]));
        			  }else{
        				  n += outval((char*)&msg[n], zmp[i]);
        			  }
        		  }else{
        			  n += outval((char*)&msg[n],0);
        		  }
        		  break;
        	  case 2:
        		  if(i < 6){
        			  if(i == 0 || i == 3 ){ // x only
        				  n += outval((char*)&msg[n], (int)(zmp[i] * param.i[_lp_mag]));
        			  }else{
        				  n += outval((char*)&msg[n], 0);
        			  }
        		  }else{
        			  n += outval((char*)&msg[n],0);
        		  }
        		  break;
        	  case 3:
        		  if(i < 6){
        			  if(i == 1 || i == 4 ){ // y only
        				  n += outval((char*)&msg[n], (int)(zmp[i] * param.i[_lp_mag]));
        			  }else{
        				  n += outval((char*)&msg[n], 0);
        			  }
        		  }else{
        			  n += outval((char*)&msg[n],0);
        		  }
        		  break;
        	  }
            msg[n++] = ' ';
          }
          for(i = 0; i < 3; i++){
          	  switch(param.i[_gyro_select]){
          	  case 0:
                    n += outval((char*)&msg[n], *(short*)&accld[s_idx[i]*2] * s_sgn[i]);
                    break;
          	  case 1:
                    n += outval((char*)&msg[n], *(short*)&accld[i*2]);
                    break;
          	  case 2:
          		  n += outval((char*)&msg[n], (int)(acc_angle[i] * 1800 / M_PI));
                    break;
          	  default:
                    n += outval((char*)&msg[n], *(short*)&accld[s_idx[i]*2] * s_sgn[i]);
          		  break;
          	  }
            msg[n++] = ' ';
          }
          for(i = 0; i < 3; i++){
        	  switch(param.i[_gyro_select]){
        	  case 0:
                  n += outval((char*)&msg[n], *(short*)&gyrod[s_idx[i]*2] * s_sgn[i]);
                  break;
        	  case 1:
                  n += outval((char*)&msg[n], *(short*)&gyrod[i*2]);
                  break;
        	  case 2:
        		  n += outval((char*)&msg[n], (int)(gyro_angle[i] * 1800 / M_PI));
                  break;
        	  default:
                  n += outval((char*)&msg[n], *(short*)&gyrod[s_idx[i]*2] * s_sgn[i]);
        		  break;
        	  }
            msg[n++] = ' ';
          }
          n += outval((char*)&msg[n], frame);
          msg[n++] = '>';
          add_eol((char*)&msg[n]);
          tmcom_puts((char*)msg);
        }
      }
      if(spi_disp){
        //key_control();
        if(sw_status[SW_SPI_MON - SW_PRM_MON]){
          n = 0;
          for(i = 0; i < 8; i++){
            n += out4h((char*)&msg[n], spidata[i]);
          }
          add_eol((char*)&msg[n]);
          tmcom_puts((char*)msg);
        }
        if(sw_status[SW_CONTROLER_MON - SW_PRM_MON] && controler_connect){
          n = out4h((char*)msg, ~spidata[controler_reg_list[0]]);
          n += out4h((char*)&msg[n], ~spidata[controler_reg_list[1]]);
          n += out4h((char*)&msg[n], ~spidata[controler_reg_list[2]]);
          msg[n++] = ' ';
          n += outval((char*)&msg[n], shift_code);
          msg[n++] = ' ';
          n += outval((char*)&msg[n], command_code);
          msg[n++] = ' ';
          n += outval((char*)&msg[n], c_num);
          msg[n++] = ' ';
          n += outval((char*)&msg[n], d_command);
          msg[n++] = ' ';
          n += outval((char*)&msg[n], d_command_exor);
          for(i = 0; i < 4; i++){
            msg[n++] = ' ';
            n += outfloat((char*)&msg[n], joy[i]);
          }
          add_eol((char*)&msg[n]);
          tmcom_puts((char*)msg);
        }
        spi_disp = 0;
      }
      if(sv_rom_disp > -1){
    	  int rom_size;
    	  int sp = 0;
    	  if(joint[jidx].d[JOINT_ICS] == SV_ICS_3_0){
    		  rom_size = 60;
    	  }else{
    		  rom_size = 66;
    	  }
		  sprintf(s, "temp limit: %d\r\n", (sv_rom_data[30] << 4) | (sv_rom_data[31] & 0x0f));
		  tmcom_puts(s);
		  sprintf(s, "crnt limit: %d\r\n", (sv_rom_data[32] << 4) | (sv_rom_data[33] & 0x0f));
		  tmcom_puts(s);
		  sprintf(s, "   ");
		  sp = 3;
		  for(i = 1; i < 11; i++){
			  sprintf(&s[sp], "%02d ", i);
			  sp += 3;
		  }
		  sprintf(&s[sp], "\r\n");
		  tmcom_puts(s);
		  sp = 0;
    	  for(i = 0; i < rom_size; i++){
    		  if(sp == 0){
    			  sprintf(s, "%02d ", (i / 10)*10);
    			  sp = 3;
    		  }
    		  sprintf(&s[sp], "%02x ", sv_rom_data[i]);
    		  sp += 3;
    		  if(i % 10 == 9){
    			  sprintf(&s[sp], "\r\n");
        		  tmcom_puts(s);
        		  sp = 0;
    		  }
    	  }
    	  if(sp != 0){
			  sprintf(&s[sp], "\r\n");
    		  tmcom_puts(s);
    	  }
    	  sv_rom_disp  = -1;
      }
      if(stability_disp_on && stability_update){
    	  sprintf(s, "stab, %f ,%f, %f\r\n", stability[X], stability[Y], stability[Z]);
    	  tmcom_puts(s);
      }
#if 0
      if(sw_status[SW_STATUS_MON - SW_PRM_MON] && batt_check_on || ADC1ConvertedValue[7] < base.battery_limit && batt_check_on){
        n = 0;
        n += str_cpy((char*)&msg[n], "BATT = ");
        n += outval((char*)&msg[n], ADC1ConvertedValue[7]);
        msg[n++] = ' ';
        if(ADC1ConvertedValue[7] < base.battery_limit){
          n += str_cpy((char*)&msg[n], " ***LOW BATTERY***");
        }
        add_eol((char*)&msg[n]);
        tmcom_puts((char*)msg);
        batt_check_on = 0;
      }
#endif
      if(sw_status[SW_STATUS_MON - SW_PRM_MON] && damage_check_on){
        n = 0;
        n += str_cpy((char*)&msg[n], "DAMAGE = ");
        n += outval((char*)&msg[n], ADC1ConvertedValue[8]);
        n += str_cpy((char*)&msg[n], " ");
        n += outval((char*)&msg[n], ADC1ConvertedValue[9]);
        add_eol((char*)&msg[n]);
        tmcom_puts((char*)msg);
        damage_check_on = 0;
      }
      if(sw_status[SW_HIDAN - SW_PRM_MON] && damage_on && command == CMD_NOP){
        n = 0;
        n += str_cpy((char*)&msg[n], " ***DAMAGE FULL***");
        add_eol((char*)&msg[n]);
        tmcom_puts((char*)msg);
        command = CMD_SERVO_OFF;
        //GPIO_ResetBits(EXTLED1);
        sw_status[SW_HIDAN - SW_PRM_MON] = 0;
      }
      if(zmp_walk_on && !zmp_walk_data_ready){
          zmp_walk();
      }
      if (zmp_walk_cont_on == 2){ //継続歩行の場合
    	  zmp_walk_rebuild_pre(mode, width, turn_width, step, radius, pace, n_step);
		  rebuild_count = 0;
		  //予見データ再構築実施
		  zmp_walk_cont_on = 3;
		  sprintf(s, "rebuild data make start :%ld\r\n", frame);
		  tmcom_puts(s);
      }
      if( zmp_walk_cont_on == 1){
      	//継続指示があったが、継続可能かどうかの判断
  		//zmpplan copy
  		list_copy(&zmpplan, &zmpplanc);
  		//bzmp copy zmpplanとzmpplancで差がないことを確認してからコピーすること
  		while(list_size(&zmpplan) != list_size(&zmpplanc) && zmpplanc.head != NULL){
  			list_pop_front(&zmpplanc);
  		}
  		vct_copy(bzmp, bzmpc);
  		//①質点モデル内部データのコピー（予備⇒変更用）
  		//②姿勢・予見器・モーション生成パラメータのデータコピー　→　変更スレッドで行いたいところだが、コピー元で変更するものなのでここでコピーが必須
  		//③切り替え開始フラグを立てる
  		prediction_copy(&pred_rsv, &pred_rmk);  //予見器データをコピー（予備データで現時点から未来を作る）
  		base_get_gp(&robot, j, RESERVE, REMAKE);  //質点モデル内データのコピー（予備→変更用）
  		motion_para_copy(&mtp_rsv, &mtp_rmk);  //mtp構造体コピー
  		lbp_copy(&lbp_rsv, &lbp_rmk);  //lbpコピー
      	search_cp(); // zmpplanc, bzmpcを使う。ただし、cp_nodeはzmpplanから探す必要がある。
          /*
          cp_space の対応が必要かどうか。
          さらには通常の歩行継続で必要か、最終歩で必要かを見き分ける必要がある。
          */
      	if(cp_node){
          	sprintf(s, "cp_node:%0x cp_pivot:%d %f %f \r\n", cp_node, cp_pivot, cp_node->d[X], cp_node->d[Y]);
      		tmcom_puts(s);
      	}else{
      		tmcom_puts("cp_node=NULL\r\n");
      	}
      	if (cp_node && cp_space > param.i[_cp_space_threshold]){  //継続フラグが立っても、継続機会を過ぎていたら無効
      		zmp_walk_cont_on = 2;
      	}else if(!cp_node){
      		zmp_walk_cont_on = 0;
      		sprintf(s, "cp_node is NULL! rebuild process abort.\r\n");
      		tmcom_puts(s);
      	}else if(!(cp_space > param.i[_cp_space_threshold])){
      		zmp_walk_cont_on = 0;
      		sprintf(s, "cp_space too nallow. rebuild process abort. : %d\r\n", cp_space);
      		tmcom_puts(s);
      	}
      	//継続実施決定。　継続データを作成する準備を実施
      }
      if(zmp_walk_cont_on == 3){
    	  if(rebuild_count < F_STEP + param.i[_d_step]){
    		  zmp_walk_rebuild(rebuild_count++);
    	  }else{
    		  zmp_walk_cont_on = 4; //　実際に差し替えを実施しない場合は　=0  とする。
    		  sprintf(s, "rebuild data make finish : %ld", frame);
    		  tmcom_puts(s);
    	  }
      }
      if(servo_cmd_err_flag){
    	  for(i = 0; i < servo_cmd_err; i++){
    		  sprintf(s, "sce %d:%d\r\n", servo_cmd_err_id[i], servo_cmd_err_cmd[i]);
    		  tmcom_puts(s);
    	  }
    	  servo_cmd_err_flag = 0;
    	  servo_cmd_err = 0;
      }
      if(servo_setting_command_find != 0){
    	  sprintf(s, "frame:%d\r\n", frame);
    	  tmcom_puts(s);
    	  for(i = 0; i < servo_setting_command_find; i++){
    		  sprintf(s, "SERVO SETTING COMMAND FIND joint[%d].d[JOINT_PRMCMD] = %d\r\n", servo_setting_command_id[i], servo_setting_command_command[i]);
    		  tmcom_puts(s);
    	  }
    	  servo_setting_command_find = 0;
      }

      //コマンドリクエストの処理
      if(command_req){
        if(!command){
          command = command_req;
          command_req = CMD_NOP;
        }
      }
    }

    ///////////コマンド実行 start
    if(command == CMD_SERVO_ON){
      if(!damage_on){
        tmcom_puts("servo on\r\n");
        for(sv = 0; sv < SV_VOL; sv++){
          joint[sv].d[JOINT_CMD] = 1;
        }
        servo_on = 1;
      }
    }else if(command == CMD_SERVO_OFF){
      tmcom_puts("servo off\r\n");
      for(sv = 0; sv < SV_VOL; sv++){
        joint[sv].d[JOINT_CMD] = 0;
      }
      servo_on = 0;
    }else if(command == CMD_SERVO_ZERO){
      tmcom_puts("servo zero\r\n");
      for(sv = 0; sv < SV_VOL; sv++){
        joint[sv].d[JOINT_TARGET] = 0;
        joint[sv].d[JOINT_TIME] = 100;
        joint[sv].d[JOINT_CMD] = 2;
      }
    }else if(command == CMD_SENSOR_SWITCH){
      sw_status[SW_HIDAN - SW_PRM_MON] = (sw_status[SW_HIDAN - SW_PRM_MON] + 1) % 2;
      show_sw_status(SW_HIDAN);
    }else if(command == CMD_LASER_SWITCH){
      laser_on = (laser_on + 1) % 2;
      if(laser_on){
        //GPIO_ResetBits(LASER);
      }else{
        //GPIO_SetBits(LASER);
      }
    }
    command = CMD_NOP;
    ///////////コマンド実行 end

  }
}

void servo_data_set(void)
{
  int i;
  short follow = 0;
  unsigned short res;
  static int pitch_follow_right = 0, pitch_follow_left = 0;
  static int touch_right = 0, touch_left = 0;

  int test_phase;
  
  if(test_on == 1){
    ////////////////////////////////////
    // 足裏センサーによる足首ROLL軸制御
    ////////////////////////////////////

    // 足が浮いている場合は制御しない（元に戻す？）
    // サーボの反応時間分を待つ？

    if(test_end){
      // 制御OFFのタイミングで制御値リセット
      rollFollowRightSidePid.u = 0;
      rollFollowLeftSidePid.u = 0;
      rollFollowRightForePid.u = 0;
      rollFollowLeftForePid.u = 0;
      balanceControlSidePid.u = 0;
      balanceControlPitchPid.u = 0;
      test_end = 0;
      test_on = 0;
    }else{
      // 足首ROLL制御
      short def_r = abs(joint[5].d[JOINT_POS] - joint[5].d[JOINT_CAPTURE]);
      short def_l = abs(joint[11].d[JOINT_POS] - joint[11].d[JOINT_CAPTURE]);
      if(zmp[ZMP_RP] > 0.05 &&  zmp[ZMP_LP] > 0.05){
        //右足
        if(((zmp[ZMP_RX] < -0.1 && !(joint[5].d[JOINT_ERR] & SV_ERR_MIN_OVER)) || (zmp[ZMP_RX] > 0.1 && !(joint[5].d[JOINT_ERR] & SV_ERR_MAX_OVER)))){
          calc_pid(&rollFollowRightSidePid, zmp[ZMP_RX], def_r > test_prm[0]);
          follow++;
        }else{
          rollFollowRightSidePid.integral = 0;
        }
        //左足
        if(((zmp[ZMP_LX] < -0.1 && !(joint[11].d[JOINT_ERR] & SV_ERR_MIN_OVER)) || (zmp[ZMP_LX] > 0.1 && !(joint[11].d[JOINT_ERR] & SV_ERR_MAX_OVER)))){
          calc_pid(&rollFollowLeftSidePid, zmp[ZMP_LX], def_l > test_prm[0]);
          follow++;
        }else{
          rollFollowLeftSidePid.integral = 0;
        }
      }
      // 足首PITCH制御
      short def_rp = abs(joint[4].d[JOINT_POS] - joint[4].d[JOINT_CAPTURE]);
      short def_lp = abs(joint[10].d[JOINT_POS] - joint[10].d[JOINT_CAPTURE]);
      if(zmp[ZMP_RP] > 0.05 && !touch_right){
        touch_right = 1;
        if(!pitch_follow_right){
          pitch_follow_right = 1;
        }
      }else{
        if(zmp[ZMP_RP] < 0.05){
          touch_right = 0;
        }
      }
      if(zmp[ZMP_LP] > 0.05 && !touch_left){
        touch_left = 1;
        if(!pitch_follow_left){
          pitch_follow_left = 1;
        }
      }else{
        if(zmp[ZMP_LP] < 0.05){
          touch_left = 0;
        }
      }
      //右足PITCH
      if(pitch_follow_right){
        if(((zmp[ZMP_RY] < -0.1 && !(joint[4].d[JOINT_ERR] & SV_ERR_MIN_OVER)) || (zmp[ZMP_RY] > 0.1 && !(joint[4].d[JOINT_ERR] & SV_ERR_MAX_OVER)))){
          if(fabs(zmp[ZMP_RY]) > fabs(zmp[ZMP_LY])){
            calc_pid(&rollFollowRightForePid, zmp[ZMP_RY], def_rp > test_prm[0]);
            follow++;
          }
        }else{
          rollFollowRightForePid.integral = 0;
          pitch_follow_right = 0;
        }
      }
      //左足PITCH
      if(pitch_follow_left){
        if(((zmp[ZMP_LY] < -0.1 && !(joint[10].d[JOINT_ERR] & SV_ERR_MIN_OVER)) || (zmp[ZMP_LY] > 0.1 && !(joint[10].d[JOINT_ERR] & SV_ERR_MAX_OVER)))){
          if(fabs(zmp[ZMP_RY]) < fabs(zmp[ZMP_LY])){
            calc_pid(&rollFollowLeftForePid, zmp[ZMP_LY], def_lp > test_prm[0]);
            follow++;
          }
        }else{
          rollFollowLeftForePid.integral = 0;
          pitch_follow_left = 0;
        }
      }
      // 左右重心制御
      // 左右の荷重を比較して、重心を左右移動させる。
      FLOAT reaction_diff = zmp[ZMP_RP] - zmp[ZMP_LP];
      if((zmp[ZMP_RP] > 0.05 || zmp[ZMP_LP] > 0.05) && (reaction_diff > 0.1 || reaction_diff < -0.1)){
        calc_pid(&balanceControlSidePid, reaction_diff, def_r < test_prm[0] && def_l < test_prm[0]);
        follow++;
      }else{
        balanceControlSidePid.integral = 0;
      }
      // 前後重心制御
      FLOAT pitch_diff = zmp[ZMP_RY] + zmp[ZMP_LY];
      if((zmp[ZMP_RP] > 0.05 || zmp[ZMP_LP] > 0.05) && (pitch_diff > 0.05 || pitch_diff < -0.05)){
        calc_pid(&balanceControlPitchPid, pitch_diff, def_r < test_prm[0] && def_l < test_prm[0]);
        follow++;
      }else{
        balanceControlPitchPid.integral = 0;
      }

      // 遊脚時には倣いをリセット
      if(zmp[ZMP_RP] < 0.05){
        follow += release_pid(&rollFollowRightSidePid, 20, 50);
        follow += release_pid(&rollFollowRightForePid, 20, 50);
      }
      if(zmp[ZMP_LP] < 0.05){
        follow += release_pid(&rollFollowLeftSidePid, 20, 50);
        follow += release_pid(&rollFollowLeftForePid, 20, 50);
      }
      // 持ち上げた時には重心制御リセット
      if(zmp[ZMP_RP] < 0.05 &&  zmp[ZMP_LP] < 0.05){
        follow += release_pid(&balanceControlSidePid, 0, 2);
      }
      if(zmp[ZMP_RP] < 0.05 &&  zmp[ZMP_LP] < 0.05){
        follow += release_pid(&balanceControlPitchPid, 0, 2);
      }
      // 水平制御
      // 胴体部分の水平度を加速度センサーで見て、足上げ寸法を調整する。
      
      // 実行
      if(follow){
        lbp.o[X] = balanceControlSidePid.u;
        lbp.o[Y] = balanceControlPitchPid.u;
        res = lbp_cnv_l_sv(&lbp, gp, j);
        if(res & 0x4040){
          balanceControlSidePid.u -= balanceControlSidePid.ut;
        }else{
          for(i = 0; i < LEGS_SERVO_QT; i++){
            joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
            joint[i].d[JOINT_TIME] = 0;
          }
          joint[5].d[JOINT_TARGET] += rollFollowRightSidePid.u;
          joint[11].d[JOINT_TARGET] += rollFollowLeftSidePid.u;
          joint[4].d[JOINT_TARGET] += rollFollowRightForePid.u;
          joint[10].d[JOINT_TARGET] += rollFollowLeftForePid.u;
          for(i = 0; i < LEGS_SERVO_QT; i++){
          joint[i].d[JOINT_CMD] = SV_POS;
          }
        }
        follow = 0;
      }
      if(test_disp == 0) test_disp = 1;
    }
  }else if(test_on == 2){
    if(++test_phase >= test_prm[0]){
      test_phase = 0;
      if(joint[2].d[JOINT_PRM] == test_stretch){
        joint[2].d[JOINT_PRMCMD] = SV_STRETCH;
        joint[2].d[JOINT_PRM] = test_stretch2;
        joint[8].d[JOINT_PRMCMD] = SV_STRETCH;
        joint[8].d[JOINT_PRM] = test_stretch2;
      }else{
        joint[2].d[JOINT_PRMCMD] = SV_STRETCH;
        joint[2].d[JOINT_PRM] = test_stretch;
        joint[8].d[JOINT_PRMCMD] = SV_STRETCH;
        joint[8].d[JOINT_PRM] = test_stretch;
      }
    }
  }
  if(!zmp_walk_on && !motion_play_on){
	  int feedback_valid = 0;
	  if(gyro_feedback_on){
		  //sprintf(s, "gad,%f,%f,%f\r\n", gyro_angle_diff[X],gyro_angle_diff[Y],gyro_angle_diff[Z]);
		  //tmcom_puts(s);
		  FLOAT roll = gyro_angle[Y] + param.f[_gyro_angle_offset_roll] - lbp.p[LBP_ROLL];
		  FLOAT pitch = gyro_angle[X] + param.f[_gyro_angle_offset_pitch] - lbp.p[LBP_PITCH];
		  if(fabs(roll) > param.f[_criteria_of_standup] || fabs(pitch) > param.f[_criteria_of_standup]){
      		clear_pid2(&gyroFeedbackPid0);
      		clear_pid2(&gyroFeedbackPid1);
		  }
		  calc_pid2(&gyroFeedbackPid0, &param.f[_pid0_p_fact], roll); //roll
		  calc_pid2(&gyroFeedbackPid1, &param.f[_pid1_p_fact], pitch); //pitch
		  if(feedback_verbose){
			  sprintf(s, "r:%f p:%f ", gyro_angle[Y], gyro_angle[X]);
			  tmcom_puts(s);
			  sprintf(s, "r.u:%f p.u:%f\r\n", gyroFeedbackPid0.u, gyroFeedbackPid1.u);
			  tmcom_puts(s);
		  }
		  if((gyroFeedbackPid0.u != 0 || gyroFeedbackPid1.u != 0) && feedback_verbose < 2){
			  lbp_copy(&lbp, &gf_lbp);
			  if(gyroFeedbackPid0.u != 0){
				  gf_lbp.p[LBP_ROLL] -= gyroFeedbackPid0.u * param.f[_gyro_roll_feedback_retio];
			  }
			  if(gyroFeedbackPid1.u != 0){
#ifdef GYROFEEDBACK_TYPE1
				  gf_lbp.p[LBP_PITCH] -= gyroFeedbackPid1.u * param.f[_gyro_pitch_feedback_retio];
				  gf_lbp.o[Y] = gyroFeedbackPid1.u * param.f[_pid_y_offset_rate_gyro];
#else
#ifdef GYROFEEDBACK_TYPE2
				  zmp_feedback_ankle = gyroFeedbackPid1.u * param.f[_pos_pitch_calibration_retio];
#endif
#endif
			  }
			  if(feedback_verbose){
//				  sprintf(s, "r.u:%f p.u:%f\r\n", gyroFeedbackPid0.u, gyroFeedbackPid1.u);
//				  tmcom_puts(s);
			  }
			  feedback_valid = 1;
		  }else{
			  if(feedback_verbose){
//				  tmcom_puts("\r\n");
			  }
		  }
	  }
	  if(pos_calibration_on){
		  calc_pid2(&posCalibrationPid0, &param.f[_pid3_p_fact], zmp[ZMP_LY]); //zmp[LY]
		  calc_pid2(&posCalibrationPid1, &param.f[_pid4_p_fact], gyro_angle[X] + param.f[_gyro_angle_offset_pitch] - lbp.p[LBP_PITCH]); //pitch
		  if(feedback_verbose){
			  sprintf(s, "zmp RY%f LY:%f p:%f ", zmp[ZMP_RY], zmp[ZMP_LY], gyro_angle[X]);
			  tmcom_puts(s);
		  }
		  if(posCalibrationPid0.u != 0 || posCalibrationPid1.u != 0){
			  lbp_copy(&lbp, &gf_lbp);
			  if(posCalibrationPid0.u != 0){
				  gf_lbp.o[Y] = posCalibrationPid0.u * param.f[_pid_y_offset_rate_zmp];
			  }
			  if(posCalibrationPid1.u != 0){
				  zmp_feedback_ankle = posCalibrationPid1.u * param.f[_pos_pitch_calibration_retio];
			  }
			  if(feedback_verbose){
				  sprintf(s, "z.u:%f p.u:%f\r\n", posCalibrationPid0.u, posCalibrationPid1.u);
				  tmcom_puts(s);
			  }
			  feedback_valid = 1;
		  }else{
			  if(feedback_verbose){
				  tmcom_puts("\r\n");
			  }
		  }
	  }
	  if(com_calibration_on){
		  calc_pid2(&comCalibrationPid0, &param.f[_pid5_p_fact], zmp[ZMP_LY]); //zmp[LY]
		  if(feedback_verbose){
			  sprintf(s, "zmp RY%f LY:%f ", zmp[ZMP_RY], zmp[ZMP_LY]);
			  tmcom_puts(s);
		  }
		  if(comCalibrationPid0.u != 0){
			  lbp_copy(&lbp, &gf_lbp);
			  gf_lbp.o[Y] = comCalibrationPid0.u * param.f[_pid_y_offset_rate_zmp];
			  if(feedback_verbose){
				  sprintf(s, "u:%f\r\n", comCalibrationPid0.u);
				  tmcom_puts(s);
			  }
			  feedback_valid = 1;
		  }else{
			  if(feedback_verbose){
				  tmcom_puts("\r\n");
			  }
		  }
	  }
	  if(zmp_align_on){
		  calc_pid2(&zmpAlignPid0, &param.f[_pid6_p_fact], zmp[ZMP_RY] - zmp[ZMP_LY]);
		  if(feedback_verbose){
			  sprintf(s, "zmp LY%f RY:%f ", zmp[ZMP_LY], zmp[ZMP_RY]);
			  tmcom_puts(s);
		  }
		  if(zmpAlignPid0.u != 0){
			  lbp_copy(&lbp, &gf_lbp);
			  zmp_align_amount = zmpAlignPid0.u * param.f[_zmp_pitch_align_retio];
			  if(feedback_verbose){
				  sprintf(s, "u:%f\r\n", zmpAlignPid0.u);
				  tmcom_puts(s);
			  }
			  feedback_valid = 1;
		  }else{
			  if(feedback_verbose){
				  tmcom_puts("\r\n");
			  }
		  }
	  }
	  if(xzmp_calibration_on){
		  calc_pid2(&xzmpCalibrationPid0, &param.f[_pid7_p_fact], zmp[ZMP_RX]);
		  calc_pid2(&xzmpCalibrationPid1, &param.f[_pid8_p_fact], zmp[ZMP_LX]);
		  if(feedback_verbose){
			  sprintf(s, "zmp LX[%d]%f RX:[%d]%f ", ZMP_LX, zmp[ZMP_LX], ZMP_RX, zmp[ZMP_RX]);
			  tmcom_puts(s);
		  }
		  if(xzmpCalibrationPid0.u != 0 || xzmpCalibrationPid1.u != 0){
			  lbp_copy(&lbp, &gf_lbp);
			  if(xzmpCalibrationPid0.u != 0){
				  xrzmp_calibration_amount = xzmpCalibrationPid0.u * param.f[_xzmp_right_roll_calibration_retio];
			  }
			  if(xzmpCalibrationPid1.u != 0){
				  xlzmp_calibration_amount = xzmpCalibrationPid1.u * param.f[_xzmp_left_roll_calibration_retio];
			  }
			  if(feedback_verbose){
				  sprintf(s, "l.u:%f r.u:%f\r\n", xzmpCalibrationPid1.u, xzmpCalibrationPid0.u);
				  tmcom_puts(s);
			  }
			  feedback_valid = 1;
		  }else{
			  if(feedback_verbose){
				  tmcom_puts("\r\n");
			  }
		  }
	  }
	  if(feedback_valid){
		  gf_lbp.o[Y] += param.f[_com_calibration_amount];
		  if((res = lbp_cnv_l_sv(&gf_lbp, gp, j)) != 0){
		      //error
			  sprintf(s, "lbp error : %d p:%f o:%f (servo_data_set() feedback)\r\n", res, gf_lbp.p[param.i[_gyro_feedback_debug1]], gf_lbp.o[param.i[_gyro_feedback_debug2]]);
			  tmcom_puts(s);
		  }else{
              j[joint_right_ankle_pitch] += param.f[_ankle_pitch_calibration_amount] + param.f[_zmp_align_amount];
              j[joint_left_ankle_pitch] += param.f[_ankle_pitch_calibration_amount];
              j[joint_right_ankle_roll] += param.f[_ankle_roll_right_calibration_amount];
              j[joint_left_ankle_roll] += param.f[_ankle_roll_left_calibration_amount];
#ifdef GYROFEEDBACK_TYPE2
              if(gyro_feedback_on){
				  j[joint_right_ankle_pitch] += zmp_feedback_ankle;
				  j[joint_left_ankle_pitch] += zmp_feedback_ankle;
              }
#endif
			  if(pos_calibration_on){
				  j[joint_right_ankle_pitch] += zmp_feedback_ankle;
				  j[joint_left_ankle_pitch] += zmp_feedback_ankle;
			  }
			  if(zmp_align_on){
				  j[joint_right_ankle_pitch] += zmp_align_amount;
			  }
			  if(xzmp_calibration_on){
	              j[joint_right_ankle_roll] += xrzmp_calibration_amount;
	              j[joint_left_ankle_roll] += xlzmp_calibration_amount;
			  }
			  for(i = 0; i < LEGS_SERVO_QT; i++){
				  joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
				  joint[i].d[JOINT_TIME] = 0;
				  joint[i].d[JOINT_CMD] = SV_POS;
			  }
		  }
		  gf_lbp.o[Y] -= param.f[_com_calibration_amount];  // gf_lbpは使い捨てなので戻す必要は無いが。
	  }
  }
}

void RCC_Configuration(void)
{
#ifdef STBEEMINI
  // JTAGを無効にします。
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
  AFIO->MAPR = _BV(26);
#endif
  // ペリフェラルクロック有効化
  // USART1,SPI1: APB2に属する。
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_SPI1 , ENABLE);
  // USART3,UART4,TIM2,TIM3,TMI4,I2C1,I2C3: APB1に属する。
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3 | RCC_APB1Periph_UART4 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_I2C1 | RCC_APB1Periph_I2C3, ENABLE);
  // GPIOA,GPIOB,GPIOC,GPIOD: AHB1に属する。
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  GPIO_DeInit(GPIOB);

  // USART1 Tx (PA.09) を alternate function push-pull に設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           // 設定対象
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // 速度(これは以降も使いまわす)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     // モード
  GPIO_Init(GPIOA, &GPIO_InitStructure);              // 適用

  // USART1 Rx (PA.10) を alternate input floating に設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // 速度(これは以降も使いまわす)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA , GPIO_PinSource9 , GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA , GPIO_PinSource10 , GPIO_AF_USART1);

  // USART3 Tx (PB.10) を alternate function open-drain に設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;           // 設定対象
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // 速度(これは以降も使いまわす)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // モード
  GPIO_Init(GPIOB, &GPIO_InitStructure);              // 適用

  // USART3 Rx (PB.11) を alternate input floating に設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // 速度(これは以降も使いまわす)
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // モード
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB , GPIO_PinSource10 , GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB , GPIO_PinSource11 , GPIO_AF_USART3);

  // USART4 Tx (PC.10) を alternate function open-drain に設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;           // 設定対象
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // 速度(これは以降も使いまわす)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // モード
  GPIO_Init(GPIOC, &GPIO_InitStructure);              // 適用

  // USART43 Rx (PC.11) を alternate input floating に設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   // 速度(これは以降も使いまわす)
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // モード
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOC , GPIO_PinSource10 , GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC , GPIO_PinSource11 , GPIO_AF_UART4);

  // ADC の端子設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  // SPI1のGPIO設定
  GPIO_PinAFConfig(GPIOB , GPIO_PinSource3 , GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOB , GPIO_PinSource4 , GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOB , GPIO_PinSource5 , GPIO_AF_SPI1);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     // モード
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     // モード
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // I2C1のGPIO設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // モード
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB , GPIO_PinSource6 , GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB , GPIO_PinSource7 , GPIO_AF_I2C1);

  // GPIOD 2 を出力設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     // モード
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // GPIOA 8 を出力設定
/*  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // モード
  GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
  // I2C3のGPIO設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     // モード
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // モード
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     // モード
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     // モード
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA , GPIO_PinSource8 , GPIO_AF_I2C3);
  GPIO_PinAFConfig(GPIOC , GPIO_PinSource9 , GPIO_AF_I2C3);

}

void USART_Configuration( void)
{
  USART_InitTypeDef USART_InitStructure;
  USART_StructInit(&USART_InitStructure);
  
  // USART1設定
  //USART_InitStructure.USART_BaudRate = 57600;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // USART3設定
  USART_InitStructure.USART_BaudRate = 625000; // * 10 / 15;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);

  // USART4設定
  USART_InitStructure.USART_BaudRate = 625000; // * 10 / 15;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);

  // 割り込み　送信終了で割り込み
  USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  USART_ITConfig(USART3, USART_IT_TC, ENABLE);
  USART_ITConfig(UART4, USART_IT_TC, ENABLE);

  // USART有効化
  USART_Cmd( USART1, ENABLE);
  USART_Cmd( USART3, ENABLE);
  USART_Cmd( UART4, ENABLE);
}

void SPI_Configuration(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  SPI_StructInit(&SPI_InitStructure);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  //	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_RxOnly;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_Cmd(SPI1, ENABLE);
  //SPI_SSOutputCmd(SPI1, ENABLE);
}

void TIM_Configuration( void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  // TIM2設定
  // (168/2)MHz -> 10kHz -> 50Hz (20ms/cycle)
  TIM_TimeBaseStructure.TIM_Period = 199;  // (200 / 2) - 1
  TIM_TimeBaseStructure.TIM_Prescaler = 8399;  // (16800/2)-1
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure);

  // 割り込み許可...割り込み要因のORを引数に入れる
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  // TM2 タイマ2起動起動はメインループで

  //　TIM3設定
  // (168/2)MHz -> 1MHz -> 20kHz (50us)
  TIM_TimeBaseStructure.TIM_Period = 49;
  TIM_TimeBaseStructure.TIM_Prescaler = 83;  // (168/2)-1
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void TIM4_Setting(int len)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  // (168/2)MHz -> 1MHz =: 1us
  // 115200baud : 87us/1byte
  // margin = 1000us

  // 625000baud : 18us/1byte 実際にはバイト間の時間もあるので30us/1byte
  // margin = 100us
  TIM_TimeBaseStructure.TIM_Period = (uint16_t)(30 * len + 100);
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)83;  // (168/2)-1
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_DeInit(TIM4);
  TIM_TimeBaseInit( TIM4, &TIM_TimeBaseStructure);

  //TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);  // ワンショットモードだと、カウンターを途中で止められない。
  // 割り込み許可を入れる前に要因をリセット
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  // 割り込み許可...割り込み要因のORを引数に入れる
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

  // タイマ4起動
  TIM4->CNT = 0;
  TIM_Cmd(TIM4, ENABLE);
}

void NVIC_Configuration( void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

  //USART3 RX DMA
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;	// 設定対象となる割り込みソース // refer to lib/CMSIS/Core/CM3/stm32f10x.h file
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// 有効かどうか
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //USART3 TX
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		// 設定対象となる割り込みソース
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// 有効かどうか
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //UART4 RX DMA
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;	// 設定対象となる割り込みソース // refer to lib/CMSIS/Core/CM3/stm32f10x.h file
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// 有効かどうか
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //UART4 TX
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		// 設定対象となる割り込みソース
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		// 有効かどうか
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //USART1 TX
//  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;	// 設定対象となる割り込みソース
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	        // 設定対象となる割り込みソース
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //TIM2
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;		// 設定対象となる割り込みソース
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //TIM3
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;		// 設定対象となる割り込みソース
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //TIM4
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;		// 設定対象となる割り込みソース
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;	// 優先度
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);				// 反映

  //ADC
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream4_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		// 優先度
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
////////////////////////////  


void DMA_Configuration(void)
{
  //ADC : DMA2_Stream4
  DMA_StructInit(&DMA_InitStructure1);
  DMA_DeInit(DMA2_Stream4);
  DMA_InitStructure1.DMA_Channel = DMA_Channel_0;   // refer Reference Manual for ja to P309 (ja.DM00031020.pdf)
  DMA_InitStructure1.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
  DMA_InitStructure1.DMA_Memory0BaseAddr = (u32)ADC1ConvertedValue;
  DMA_InitStructure1.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure1.DMA_BufferSize = 10;
  DMA_InitStructure1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure1.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure1.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure1.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure1.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure1.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure1.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure1.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &DMA_InitStructure1);

  DMA_Cmd(DMA2_Stream4, ENABLE);

  //USART1 TX : DMA2_Stream7
  DMA_StructInit(&DMA_InitStructure1);
  DMA_InitStructure1.DMA_Channel = DMA_Channel_4;   // refer Reference Manual for ja to P309
  DMA_InitStructure1.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure1.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure1.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure1.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure1.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure1.DMA_Priority = DMA_Priority_High;
  //DMA_InitStructure1.DMA_FIFOMode = DMA_FIFOMode_Disable;
  //DMA_InitStructure1.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  //DMA_InitStructure1.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  //DMA_InitStructure1.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;


  //USART3 TX : DMA1_Stream3
  DMA_StructInit(&DMA_InitStructure3);
  DMA_InitStructure3.DMA_Channel = DMA_Channel_4;   // refer Reference Manual for ja to P308
  DMA_InitStructure3.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
  DMA_InitStructure3.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure3.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure3.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure3.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure3.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure3.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure3.DMA_Priority = DMA_Priority_High;
  //DMA_InitStructure3.DMA_FIFOMode = DMA_FIFOMode_Disable;
  //DMA_InitStructure3.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  //DMA_InitStructure3.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  //DMA_InitStructure3.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;




  //USART3 RX : DMA1_Stream1
  DMA_StructInit(&DMA_InitStructure2);  //USART3 RX DMA受信設定用構造体
  DMA_InitStructure2.DMA_Channel = DMA_Channel_4;   // refer Reference Manual for ja to P308
  DMA_InitStructure2.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
  DMA_InitStructure2.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure2.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure2.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure2.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure2.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure2.DMA_Priority = DMA_Priority_Medium;
  //DMA_InitStructure2.DMA_FIFOMode = DMA_FIFOMode_Disable;
  //DMA_InitStructure2.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  //DMA_InitStructure2.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  //DMA_InitStructure2.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  //DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);  //これ要らないかも

  //USART4 TX : DMA1_Stream4
  DMA_StructInit(&DMA_InitStructure4);  //USART4 TX
  DMA_InitStructure4.DMA_Channel = DMA_Channel_4;   // refer Reference Manual for ja to P308
  DMA_InitStructure4.DMA_PeripheralBaseAddr = (u32)&UART4->DR;
  DMA_InitStructure4.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure4.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure4.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure4.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure4.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure4.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure4.DMA_Priority = DMA_Priority_High;
  //DMA_InitStructure4.DMA_FIFOMode = DMA_FIFOMode_Disable;
  //DMA_InitStructure4.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  //DMA_InitStructure4.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  //DMA_InitStructure4.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  //USART4 RX : DMA1_Stream2
  DMA_StructInit(&DMA_InitStructure5);  //USART4 RX DMA受信設定用構造体
   DMA_InitStructure5.DMA_Channel = DMA_Channel_4;   // refer Reference Manual for ja to P308
  DMA_InitStructure5.DMA_PeripheralBaseAddr = (u32)&UART4->DR;
  DMA_InitStructure5.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure5.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure5.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure5.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure5.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure5.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure5.DMA_Priority = DMA_Priority_Medium;
  //DMA_InitStructure5.DMA_FIFOMode = DMA_FIFOMode_Disable;
  //DMA_InitStructure5.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  //DMA_InitStructure5.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  //DMA_InitStructure5.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

}

//手順は要見直し
void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInit(&ADC_CommonInitStructure);
  // ADC1 configuration ------------------------------------------------------
  ADC_DeInit();
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 10;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonStructInit(&ADC_CommonInitStructure);

  /* ADC1 regular channels configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_56Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_56Cycles);

  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  //↓STM32F4では要らないのかな。。。
  /* Enable ADC1 reset calibaration register */
  //ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  //while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  //ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  //while(ADC_GetCalibrationStatus(ADC1));

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConv(ADC1);
}

void I2C_Configuration(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 
  I2C_StructInit(&I2C_InitStructure);
  
  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);

  I2C_StructInit(&I2C_InitStructure);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 400000;

  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C3, &I2C_InitStructure);
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C3, ENABLE);

}

#define I2C_TIMEOUT 10000

uint32_t I2C1_BufferRead(uint8_t dev, uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
{
  int c;
  uint32_t i2cflag;
  

  c = I2C_TIMEOUT;
  /* While the bus is busy */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C1);
    if(i2cflag != I2C_FLAG_BUSY) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_GenerateSTART(I2C1, ENABLE);/* Send START condition */
  c = I2C_TIMEOUT;/* Test on EV5 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C1);
    if(i2cflag == I2C_EVENT_MASTER_MODE_SELECT) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_Send7bitAddress(I2C1, dev, I2C_Direction_Transmitter);/* Send EEPROM address for write */
  c = I2C_TIMEOUT;/* Test on EV6 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C1);
    if(i2cflag == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_SendData(I2C1, ReadAddr);/* Send the ADXL345's internal address to read from: Only one byte address */
  c = I2C_TIMEOUT;/* Test on EV8 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C1);
    if(i2cflag == I2C_EVENT_MASTER_BYTE_TRANSMITTED) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_GenerateSTART(I2C1, ENABLE);/* Send START condition a second time */
  c = I2C_TIMEOUT;/* Test on EV5 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C1);
    if(i2cflag == I2C_EVENT_MASTER_MODE_SELECT) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_Send7bitAddress(I2C1, dev, I2C_Direction_Receiver);/* Send EEPROM address for read */
  c = I2C_TIMEOUT;/* Test on EV6 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C1);
    if(i2cflag == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) break;
    c--;
  }
  if(!c) return i2cflag;
  while(NumByteToRead){/* While there is data to be read */
    if(NumByteToRead == 1){
      I2C_AcknowledgeConfig(I2C1, DISABLE);/* Disable Acknowledgement */
      I2C_GenerateSTOP(I2C1, ENABLE);/* Send STOP Condition */
    }
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){/* Test on EV7 and clear it */
      *pBuffer = I2C_ReceiveData(I2C1);/* Read a byte from the EEPROM */
      pBuffer++;/* Point to the next location where the byte read will be saved */
      NumByteToRead--;/* Decrement the read bytes counter */
    }
  }
  I2C_AcknowledgeConfig(I2C1, ENABLE);/* Enable Acknowledgement to be ready for another reception */
  return 0;
}

uint32_t I2C1_ByteWrite(uint8_t dev, uint8_t pBuffer, uint16_t WriteAddr)
{
  int c;
  uint32_t i2cflag;

  I2C_GenerateSTART(I2C1, ENABLE); /* Send STRAT condition */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  /* Test on EV5 and clear it */
  I2C_Send7bitAddress(I2C1, dev, I2C_Direction_Transmitter);  /* Send EEPROM address for write */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); /* Test on EV6 and clear it */
  I2C_SendData(I2C1, WriteAddr); /* Send the EEPROM's internal address to write to : only one byte Address */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));/* Test on EV8 and clear it */
  I2C_SendData(I2C1, pBuffer); /* Send the byte to be written */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));/* Test on EV8 and clear it */
  I2C_GenerateSTOP(I2C1, ENABLE);/* Send STOP condition */
  return 0;
}

uint32_t I2C3_BufferRead(uint8_t dev, uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
{
  int c;
  uint32_t i2cflag;


  c = I2C_TIMEOUT;
  /* While the bus is busy */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C3);
    if(i2cflag != I2C_FLAG_BUSY) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_GenerateSTART(I2C3, ENABLE);/* Send START condition */
  c = I2C_TIMEOUT;/* Test on EV5 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C3);
    if(i2cflag == I2C_EVENT_MASTER_MODE_SELECT) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_Send7bitAddress(I2C3, dev, I2C_Direction_Transmitter);/* Send EEPROM address for write */
  c = I2C_TIMEOUT;/* Test on EV6 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C3);
    if(i2cflag == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_SendData(I2C3, ReadAddr);/* Send the ADXL345's internal address to read from: Only one byte address */
  c = I2C_TIMEOUT;/* Test on EV8 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C3);
    if(i2cflag == I2C_EVENT_MASTER_BYTE_TRANSMITTED) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_GenerateSTART(I2C3, ENABLE);/* Send START condition a second time */
  c = I2C_TIMEOUT;/* Test on EV5 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C3);
    if(i2cflag == I2C_EVENT_MASTER_MODE_SELECT) break;
    c--;
  }
  if(!c) return i2cflag;
  I2C_Send7bitAddress(I2C3, dev, I2C_Direction_Receiver);/* Send EEPROM address for read */
  c = I2C_TIMEOUT;/* Test on EV6 and clear it */
  while(c){
    i2cflag = I2C_GetLastEvent(I2C3);
    if(i2cflag == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) break;
    c--;
  }
  if(!c) return i2cflag;
  while(NumByteToRead){/* While there is data to be read */
    if(NumByteToRead == 1){
      I2C_AcknowledgeConfig(I2C3, DISABLE);/* Disable Acknowledgement */
      I2C_GenerateSTOP(I2C3, ENABLE);/* Send STOP Condition */
    }
    if(I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED)){/* Test on EV7 and clear it */
      *pBuffer = I2C_ReceiveData(I2C3);/* Read a byte from the EEPROM */
      pBuffer++;/* Point to the next location where the byte read will be saved */
      NumByteToRead--;/* Decrement the read bytes counter */
    }
  }
  I2C_AcknowledgeConfig(I2C3, ENABLE);/* Enable Acknowledgement to be ready for another reception */
  return 0;
}

uint32_t I2C3_ByteWrite(uint8_t dev, uint8_t pBuffer, uint16_t WriteAddr)
{
  int c;
  uint32_t i2cflag;

  I2C_GenerateSTART(I2C3, ENABLE); /* Send STRAT condition */
  while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));  /* Test on EV5 and clear it */
  I2C_Send7bitAddress(I2C3, dev, I2C_Direction_Transmitter);  /* Send EEPROM address for write */
  while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); /* Test on EV6 and clear it */
  I2C_SendData(I2C3, WriteAddr); /* Send the EEPROM's internal address to write to : only one byte Address */
  while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));/* Test on EV8 and clear it */
  I2C_SendData(I2C3, pBuffer); /* Send the byte to be written */
  while(!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));/* Test on EV8 and clear it */
  I2C_GenerateSTOP(I2C3, ENABLE);/* Send STOP condition */
  return 0;
}

void tmcom_outval(long int val)
{
  n = outval((char*)msg, val);
  msg[n] = 0;
  tmcom_puts((char*)msg);
}

void tmcom_outvalf(int _order, char fill, long val)
{
  int i, nf, sf, sign;
  unsigned int order, number;

  nf = 0;  // 数字フラグ
  sf = 0;  // 記号、FILL文字フラグ
  order = 1;
  i = 1;
  //符号を調べる
  if( val < 0 ) {
    sign = 1;
    val = -val;
  }else{
    sign = 0;
  }
  if(i > _order)
    _order = i;
  //桁数を調べる
  while(i < _order || order * 10 < val){
    order *= 10;
    i++;
  }
  while( order > 0 ) {
    number = val / order;
    if( number > 0 || nf ) {
      if(!nf && !sf && sign){ //ﾏｲﾅｽで、まだ記号を出力していない
        tmcom_putc( '-' );
      }
      nf = 1;
      val -= number * order;
      tmcom_putc( '0' + number );
    }else if(order > 1){
      sf = 1;
      if((val * 10 / order) > 0 && sign){
        tmcom_putc( '-' );
      }else{
        tmcom_putc(fill);
      }
    }
    order /= 10;
  }
  if( !nf ) {
    tmcom_putc( '0' );
  }
}

void tmcom_outfloat(float val)
{
  n = outfloat((char*)msg, val);
  msg[n] = 0;
  tmcom_puts((char*)msg);
}

void tmcom_out2h(unsigned char val)
{
  n = out2h((char*)msg, val);
  msg[n] = 0;
  tmcom_puts((char*)msg);
}

void tmcom_out4h(unsigned short val)
{
  n = out4h((char*)msg, val);
  msg[n] = 0;
  tmcom_puts((char*)msg);
}

void tmcom_puts(char *send_data)
{
  short n;
  while(send_flg);
  n = strlen(send_data);
  memmove((void *)sendbuff, send_data, n);
  tmcom_send(n);
}

void tmcom_putc(char c)
{
  while(send_flg);
  sendbuff[0] = c;
  tmcom_send(1);
}

void tmcom_send(short n)
{
  while(send_flg);
  if(n){
    send_flg = 1;  //DMAスタート前にフラグUP
    tmcom_wd = 0;
    //GPIO_ResetBits(LED02);  // LED ON

    USART_DMACmd( USART1, USART_DMAReq_Tx, ENABLE);
    DMA_DeInit(DMA2_Stream7);
    DMA_InitStructure1.DMA_Memory0BaseAddr = (u32)sendbuff;
    DMA_InitStructure1.DMA_BufferSize = n;
    DMA_Init( DMA2_Stream7, &DMA_InitStructure1);
    DMA_Cmd( DMA2_Stream7, ENABLE);
  }
}

void tmcom_send_end(void)
{
  USART_DMACmd( USART1, USART_DMAReq_Tx , DISABLE);
  //GPIO_SetBits(LED02);  // LED OFF
  send_flg = 0;
}

void tmcom_watchdog(void)
{
  if(send_flg){
    tmcom_wd++;
  }
  if(tmcom_wd > 100){
    tmcom_send_end();
    tmcom_wd = 0;
    tmcom_senderr++;
  }
}

void tmcom_err_msg(int snd_n, char *errmsg)
{
  USART_DMACmd( USART1, USART_DMAReq_Tx, ENABLE);
  DMA_DeInit(DMA2_Stream7);   //DMA_InitはなくともDMA_DeInitは必須らしい。
  DMA_InitStructure1.DMA_Memory0BaseAddr = (u32)errmsg;
  DMA_InitStructure1.DMA_BufferSize = snd_n;
  DMA_Init( DMA2_Stream7, &DMA_InitStructure1);
  DMA_Cmd( DMA2_Stream7, ENABLE);
}

void svcom_send_usart3(int snd_n, char *snd_buff)
{
	  USART_DMACmd( USART3, USART_DMAReq_Tx, ENABLE);
	  DMA_DeInit(DMA1_Stream3);   //DMA_InitはなくともDMA_DeInitは必須らしい。
	  DMA_InitStructure3.DMA_Memory0BaseAddr = (u32)snd_buff;
	  DMA_InitStructure3.DMA_BufferSize = snd_n;
	  DMA_Init( DMA1_Stream3, &DMA_InitStructure3);
	  DMA_Cmd( DMA1_Stream3, ENABLE);
}

void svcom_send_uart4(int snd_n, char *snd_buff){
	  USART_DMACmd( UART4, USART_DMAReq_Tx, ENABLE);
	  DMA_DeInit(DMA1_Stream4);   //DMA_InitはなくともDMA_DeInitは必須らしい。
	  DMA_InitStructure4.DMA_Memory0BaseAddr = (u32)snd_buff;
	  DMA_InitStructure4.DMA_BufferSize = snd_n;
	  DMA_Init( DMA1_Stream4, &DMA_InitStructure4);
	  DMA_Cmd( DMA1_Stream4, ENABLE);
}

void svcom_send(int snd_n, char *snd_buff)
{
	if(jidx < 16){
		svcom_send_usart3(snd_n, snd_buff);
	}else{
		svcom_send_uart4(snd_n, snd_buff);
	}
}

void servo_ack_receive(int rcv_size)
{
	if(jidx < 16){
		  USART_DMACmd( USART3, USART_DMAReq_Tx , DISABLE);
	}else{
		  USART_DMACmd( UART4, USART_DMAReq_Tx , DISABLE);
	}
  svcom_receive(rcv_size); //受信処理開始
  TIM4_Setting(rcv_size); //タイムアウト監視タイマー起動
}

void servo_ack_receive_end(void)
{
  short _data;
  char rtn_cmd;
  int ec;
  int ndtr;
  if(jidx < 16){
	  USART_DMACmd( USART3, USART_DMAReq_Rx, DISABLE);
	  ndtr = DMA1_Stream1->NDTR;
	  svcom_rcv_size = DMA_InitStructure2.DMA_BufferSize - ndtr;
	  ec = USART_GetFlagStatus(USART3, USART_FLAG_FE | USART_FLAG_PE);
  }else{
	  USART_DMACmd( UART4, USART_DMAReq_Rx, DISABLE);
	  ndtr = DMA1_Stream2->NDTR;
	  svcom_rcv_size = DMA_InitStructure5.DMA_BufferSize - ndtr;
	  ec = USART_GetFlagStatus(UART4, USART_FLAG_FE | USART_FLAG_PE);
  }
  if(ec){
	    joint[jidx].d[JOINT_ERR] |= SV_ERR_ACK_RECEIVE_FALSE;
	    joint[jidx].d[JOINT_RTN] = svcom_rcv_size;
	    joint[jidx].d[JOINT_FLG] = svcom_rcv_buff[0];
	    joint[jidx].d[JOINT_ERRCNT]++;
  }else if(ndtr != 0 || joint[jidx].d[JOINT_SVID] != (svcom_rcv_buff[0] & 0x1f)){
	    joint[jidx].d[JOINT_ERR] |= SV_ERR_ACK_RECEIVE_FALSE;
	    joint[jidx].d[JOINT_RTN] = svcom_rcv_size;
	    joint[jidx].d[JOINT_FLG] = svcom_rcv_buff[0];
	    joint[jidx].d[JOINT_ERRCNT]++;
  }else{
	    cnv_servo_pos(svcom_rcv_buff[1], svcom_rcv_buff[2], &_data);
	    //joint[jidx].d[JOINT_CAPTURE] -= joint[jidx].d[JOINT_TRIM];    //POSとCAPTUREのレベルを合わせるためにはTRIMを引いてはいけない。
	    joint[jidx].d[JOINT_RTN] = svcom_rcv_size;
	    joint[jidx].d[JOINT_FLG] = 0;
	    joint[jidx].d[JOINT_ERR] &= 0xFF00;
	    rtn_cmd = svcom_rcv_buff[0] & 0x60;
	    if(rtn_cmd == 0x00){ // ポジション設定
		    joint[jidx].d[JOINT_CAPTURE] = _data;
		    if(_data == 0){
		    	joint[jidx].d[JOINT_STATUS] = SV_UNLIVE;
		    }
	    }else if(rtn_cmd == 0x20){  // read
	    	if((svcom_rcv_buff[1] & 0x07) == 0){  // ROMデータ　（STATUS誤書き込み防止）
	    		memcpy(sv_rom_data, (char *)svcom_rcv_buff, svcom_rcv_size);
	    		sv_rom_disp = jidx;
	    	}else{
		    	joint[jidx].d[JOINT_STATUS + (svcom_rcv_buff[1] & 0x07) ] = svcom_rcv_buff[2];
	    	}
	    }else if(rtn_cmd == 0x40){  // set
	    	//何もしない
	    }
  }
}

void servo_ack_receive_timeout(void)
{
  if(joint[jidx].d[JOINT_FLG] < 0){
    joint[jidx].d[JOINT_ERR] |= SV_ERR_TIMEOUT; // timeout
  }
}

void servo_disable(void)
{
	for(int i = 0; i < SV_VOL; i++){
		//joint[i].d[JOINT_STATUS] = -3;
	}
}

void svcom_receive_usart3(int rcv_n)
{
  while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)){
    USART_ReceiveData(USART3);
  }
  USART_DMACmd( USART3, USART_DMAReq_Rx, ENABLE);
  DMA_DeInit(DMA1_Stream1);
  DMA_InitStructure2.DMA_BufferSize = rcv_n;
  DMA_InitStructure2.DMA_Memory0BaseAddr = (u32)svcom_rcv_buff;
  DMA_Init( DMA1_Stream1, &DMA_InitStructure2);
  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
  DMA_Cmd( DMA1_Stream1, ENABLE);
}

void svcom_receive_uart4(int rcv_n)
{
  while(USART_GetFlagStatus(UART4, USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)){
    USART_ReceiveData(UART4);
  }
  USART_DMACmd( UART4, USART_DMAReq_Rx, ENABLE);
  DMA_DeInit(DMA1_Stream2);
  DMA_InitStructure5.DMA_BufferSize = rcv_n;
  DMA_InitStructure5.DMA_Memory0BaseAddr = (u32)svcom_rcv_buff;
  DMA_Init( DMA1_Stream2, &DMA_InitStructure5);
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
  DMA_Cmd( DMA1_Stream2, ENABLE);
}

void svcom_receive(int rcv_n)
{
	if(jidx < 16){
		svcom_receive_usart3(rcv_n);
	}else{
		svcom_receive_uart4(rcv_n);
	}
}

void svcom_receive_reset_usart3(void)
{
  USART3->CR1 &= (uint16_t)~USART_Mode_Rx;
  while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)){
    USART_ReceiveData(USART3);
  }
  USART3->CR1 |= USART_Mode_Rx;
  USART_DMACmd( USART3, USART_DMAReq_Rx, DISABLE);
}

void svcom_receive_reset_uart4(void)
{
  UART4->CR1 &= (uint16_t)~USART_Mode_Rx;
  while(USART_GetFlagStatus(UART4, USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)){
    USART_ReceiveData(UART4);
  }
  UART4->CR1 |= USART_Mode_Rx;
  USART_DMACmd( UART4, USART_DMAReq_Rx, DISABLE);
}

void svcom_receive_reset(void)
{
	if(jidx < 16){
		svcom_receive_reset_usart3();
	}else{
		svcom_receive_reset_uart4();
	}
}

void svcom_receive_abort(void)
{
/*  USART3->CR1 &= (uint16_t)~USART_Mode_Rx;
  while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)){
    USART_ReceiveData(USART3);
  }
  USART3->CR1 |= USART_Mode_Rx;*/
}

void cntlspi_read(void)
{
  int s;

  for(s = 0; s < 8; s++){
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, 0x55);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    spidata[s] = SPI_I2S_ReceiveData(SPI1);
  }
  spi_disp = 1;
}

int servo_cntl(void)
{
  //サーボトルクのON/OFF
  if(jidx < SV_VOL){
#if 0
	  if(joint[jidx].d[JOINT_CMD] != SV_OFF && joint[jidx].d[JOINT_CMD] != joint[jidx].d[JOINT_DUMMY1]){
		  servo_cmd_err++;
		  servo_cmd_err_id[servo_cmd_err - 1] = jidx;
		  servo_cmd_err_cmd[servo_cmd_err - 1] = joint[jidx].d[JOINT_CMD];
	  }
#endif
	if(joint[jidx].d[JOINT_STATUS] == SV_DISCONNECT){
		joint[jidx].d[JOINT_CMD] = SV_OFF;
	}
    if(joint[jidx].d[JOINT_CMD] == SV_OFF){
      // servo off
      joint[jidx].d[JOINT_POS] = 0;
      if(joint[jidx].d[JOINT_STATUS] != SV_DISCONNECT){
          joint[jidx].d[JOINT_STATUS] = SV_OFF;
      }
    }else if(joint[jidx].d[JOINT_CMD] == SV_ON){
      // servo on
      if(joint[jidx].d[JOINT_STATUS] != SV_ON){
        joint[jidx].d[JOINT_POS] = joint[jidx].d[JOINT_CAPTURE];
      }
      joint[jidx].d[JOINT_STATUS] = SV_ON;
    }else if(joint[jidx].d[JOINT_CMD] == SV_POS){
      // position set
      //joint[jidx].d[JOINT_TARGET] には目標角度をセット済
      short pos;
      if(joint[jidx].d[JOINT_TIME] == 1){
        joint[jidx].d[JOINT_TIME] = 0;
      }
      if(joint[jidx].d[JOINT_TIME] != 0){
        short lpos;
        if(joint[jidx].d[JOINT_STATUS] == SV_ON){
          lpos = joint[jidx].d[JOINT_LPOS];
        }else{
          lpos = (joint[jidx].d[JOINT_CAPTURE] - 7500 - joint[jidx].d[JOINT_OFFSET] - joint[jidx].d[JOINT_TRIM]) * joint[jidx].d[JOINT_SIGN];
        }
        short mpos = (joint[jidx].d[JOINT_TARGET] - lpos ) / joint[jidx].d[JOINT_TIME];
        pos = (lpos + mpos) * joint[jidx].d[JOINT_SIGN] + 7500 + joint[jidx].d[JOINT_OFFSET] + joint[jidx].d[JOINT_TRIM];
        joint[jidx].d[JOINT_LPOS] = lpos + mpos;
        joint[jidx].d[JOINT_TIME]--;
      }else{
        pos = joint[jidx].d[JOINT_TARGET] * joint[jidx].d[JOINT_SIGN] + 7500 + joint[jidx].d[JOINT_OFFSET] + joint[jidx].d[JOINT_TRIM];
        joint[jidx].d[JOINT_LPOS] = joint[jidx].d[JOINT_TARGET];
      }
//      if(pos >= joint[jidx].d[JOINT_MIN] && pos < joint[jidx].d[JOINT_MAX]){
//        joint[jidx].d[JOINT_POS] = pos;
//      }
      if(pos < joint[jidx].d[JOINT_MIN]){
        joint[jidx].d[JOINT_ERR] |= SV_ERR_MIN_OVER;
      }else if(pos > joint[jidx].d[JOINT_MAX]){
        joint[jidx].d[JOINT_ERR] |= SV_ERR_MAX_OVER;
      }else{
        joint[jidx].d[JOINT_POS] = pos;
        joint[jidx].d[JOINT_ERR] &= 0xFCFF;
      }
      joint[jidx].d[JOINT_STATUS] = SV_ON;
    }else{
      joint[jidx].d[JOINT_POS] = 0;
      joint[jidx].d[JOINT_STATUS] = SV_OFF;
    }
    // サーボへコマンド送信
    sv_cmd_real = SV_POS;
    joint[jidx].d[JOINT_FLG] = -1;  // 送信中
    make_servo_target(joint[jidx].d[JOINT_SVID], joint[jidx].d[JOINT_POS], (char*)svcom_buff);
    svcom_send(3, (char*)svcom_buff);
    return 1;
  }else{
    return 0;
  }
  /*
    ・毎回コマンドを送信してリターンにて現在位置を取得したい
    ・サーボOFFのままなら、POS=0を送り続ける
    ・サーボON→OFFならPOS=0を送る
    ・サーボONのままなら、現在の目標POSを送り続ける
    ・サーボOFF→ONなら現在位置をPOSにする
    ・サーボOFF→POS指定なら、一気に目標POSを送る
    ・サーボON→POS指定なら、目標POSを送る
    ・POS指定コマンドが指定されるとSTATUS=ONとなる。
    ⇒サーボONコマンドの時、ステータスによって動作が変わる。
    */
}

void servo_status_read(void)
{
	static int rd_jidx = 0;
	static short rd_prmt = SV_STRETCH;

	if(joint[rd_jidx].d[JOINT_PRMCMD] == SV_PRM){
		joint[rd_jidx].d[JOINT_PRMCMD] = rd_prmt | 0x0010;
		if(++rd_jidx == SV_VOL){
			rd_jidx = 0;
			if(++rd_prmt > SV_CRNT) rd_prmt = SV_STRETCH;
		}
	}
}

int servo_parameter_cntl(void)
{
  //サーボのパラメーター設定
  while(jidx < SV_VOL && joint[jidx].d[JOINT_PRMCMD] == SV_PRM){
    jidx++;
  }
  if(jidx < SV_VOL){
    // サーボへコマンド送信
    if(joint[jidx].d[JOINT_PRMCMD] != SV_PRM){
#if 0
  	  if(joint[jidx].d[JOINT_PRMCMD] != joint[jidx].d[JOINT_DUMMY1]){
  		  servo_cmd_err++;
  		  servo_cmd_err_id[servo_cmd_err - 1] = jidx;
  		  servo_cmd_err_cmd[servo_cmd_err - 1] = joint[jidx].d[JOINT_CMD];
  	  }
#endif
    	joint[jidx].d[JOINT_FLG] = -2;  // 送信中
    	if(joint[jidx].d[JOINT_PRMCMD] & 0x10){  // parameter read
    		make_servo_read_para(joint[jidx].d[JOINT_SVID], (joint[jidx].d[JOINT_PRMCMD] & 0x07), (char*)svcom_buff);
    		svcom_send(2, (char*)svcom_buff);
    	}else{  // parameter set
    	    make_servo_setting(joint[jidx].d[JOINT_SVID], joint[jidx].d[JOINT_PRMCMD], joint[jidx].d[JOINT_PRM], (char*)svcom_buff);
    	    svcom_send(3, (char*)svcom_buff);
    	}
    	sv_cmd_real = joint[jidx].d[JOINT_PRMCMD] | 0x20;
    	joint[jidx].d[JOINT_PRMCMD] = SV_PRM;
    }
    return 1;
  }else{
    jidx = 0;
    sv_prm = 0;
    return 0;
  }
}

void servo_setting_command_chech(void)
{
	servo_setting_command_find = 0;
	for(int i = 0; i < SV_VOL; i++){
		if(joint[i].d[JOINT_PRMCMD] > SV_PRM && (joint[i].d[JOINT_PRMCMD] & 0x10) == 0){
			servo_setting_command_find++;
			servo_setting_command_id[servo_setting_command_find - 1] = i;
			servo_setting_command_command[servo_setting_command_find - 1] = joint[i].d[JOINT_PRMCMD];
		}
	}

}

void key_control(void)
{
  if(controler_key_task(0, spidata, joy, &d_command, &d_command_exor, &shift_code, &command_code, &c_num)){
    //spidata[8] spiからの受信パケットが格納されている
    //d_command ダイレクトキーステータス　押されているビットが1
    //d_command_exortkey ダイレクトキー変化　変化があるとビットが1
    //shift_code シフトコード
    //command_req　コマンドコード
    //joy[4] アナログスティックの値 -1～+1
    joy_dir[0] = atan2(joy[1], -joy[0]);
    joy_dir[1] = atan2(joy[3], -joy[2]);
    joy_mag[0] = sqrt(joy[0] * joy[0] +joy[1] * joy[1]);
    joy_mag[1] = sqrt(joy[2] * joy[2] +joy[3] * joy[3]);
    command_req = command_code;
    controler_connect = 1;
  }else{
    controler_connect = 0;
  }
}

void direct_command_control()
{
  if(controler_connect){
    ///////////ダイレクトコマンド実行 start
    if(d_command & DCMD_SERVO_ON){
      if(!servo_on){
        command_req = CMD_SERVO_ON;
      }
    }else{
      if(servo_on)
        command_req = CMD_SERVO_OFF;
    }
    if(d_command & DCMD_TRIGGER_ON){
//      if(servo_on) GPIO_SetBits(TRIGGER);
    }else{
//      GPIO_ResetBits(TRIGGER);
    }
    ///////////ダイレクトコマンド実行 end
  }
}

void batt_check(void)
{
  batt_check_on = 1;
}

//////////////////////////////////////////////////////////////
// FLASH領域は 0x08000000 - 0x080FFFFF (1024kB)
// 保存データ領域として、0x080FF000 - 0x080FFFFFを割り当てている。
// FLASH erase はセクター単位となるため　0x080E0000 - 0x080FFFFF
//////////////////////////////////////////////////////////////

void save_flash(void)
{
  uint32_t flashaddr;
  uint32_t addr;
  volatile FLASH_Status FLASHStatus;
  uint32_t vsize;
  uint32_t vtop,vpos;

  //erase flash
  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
  if(FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE){
	  tmcom_puts("flash erase err.\r\n");
  }else{
	  tmcom_puts("flash erase success.\r\n");
      flashaddr = FLASH_JOINT;
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(Joint) * SV_VOL;
	  sprintf(s, "joint data         : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)joint;
	  while((vpos < vtop + vsize) && (addr < flashaddr + 0x800) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
		  vpos += 2;
	  }
	  flashaddr = FLASH_SENSOR;
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(SensorUniqueData);
	  sprintf(s, "sensor unique data : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)&sud;
	  while((vpos < vtop + vsize) && (addr < flashaddr + 0x300) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
	      vpos += 2;
	  }
      flashaddr = FLASH_PARAM;
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(Parameters);
	  sprintf(s, "parameters         : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)&param;
	  while((vpos < vtop + vsize) && (addr < flashaddr + 0x300) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
		  vpos += 2;
	  }
	  tmcom_puts("flash write complete.\r\n");
  }
  FLASH_Lock();
}

void load_flash(void)
{
  uint32_t flashaddr;
  flashaddr = FLASH_JOINT;
  uint32_t vsize;
  int i;
  vsize = sizeof(Joint) * SV_VOL;
  sprintf(s, "joint data         : %ld bytes\r\n", vsize);
  tmcom_puts(s);
  memmove((void *)joint, (void *)flashaddr, vsize);

  flashaddr = FLASH_SENSOR;
  vsize = sizeof(SensorUniqueData);
  sprintf(s, "sensor unique data : %ld bytes\r\n", vsize);
  tmcom_puts(s);
  memmove((void *)&sud, (void *)flashaddr, vsize);

  for(i = 0; i < SV_VOL; i++){
    joint[i].d[JOINT_CMD] = 0;
    joint[i].d[JOINT_TARGET] = 0;
    joint[i].d[JOINT_CAPTURE] = 0;
    joint[i].d[JOINT_ERRCNT] = 0;
    joint[i].d[JOINT_STATUS] = SV_UNDEF;
    joint[i].d[JOINT_RTN] = 0;
    joint[i].d[JOINT_ERR] = 0;
  }
/*
  flashaddr = 0x080FFC00;
  vsize = sizeof(Parameters);
  sprintf(s, "parameters         : %ld bytes\r\n", vsize);
  tmcom_puts(s);
  memmove((void *)&param, (void *)flashaddr, vsize);
*/
  tmcom_puts("load end.\r\n");
}

void load_parameters(void)
{
	  uint32_t flashaddr;
	  uint32_t vsize;
	  Parameters *prptr;
	  flashaddr = FLASH_PARAM;
	  vsize = sizeof(Parameters);
	  sprintf(s, "parameters         : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  prptr = (Parameters *)flashaddr;
/*	  for(int i = 0; i < param.i_vol_old; i++){
		  param.i[i] = prptr->i[i];
		  sprintf(s, "param.i[%d] = %d prptr->i[%d] = %d\r\n", i, param.i[i], i, prptr->i[i]);
		  tmcom_puts(s);
	  }
	  for(int i = 0; i < param.f_vol_old; i++){
		  param.f[i] = prptr->f[i];
		  sprintf(s, "param.f[%d] = %f prptr->f[%d] = %f\r\n", i, param.f[i], i, prptr->f[i]);
		  tmcom_puts(s);
	  }*/
	  memmove(param.i, prptr->i, sizeof(int) * param.i_vol_old);
	  memmove(param.f, prptr->f, sizeof(FLOAT) * param.f_vol_old);
	  //memmove((void *)&param, (void *)flashaddr, vsize);
}

void save_motion_flash(int bank)
{
  uint32_t flashaddr;
  uint32_t addr;
  volatile FLASH_Status FLASHStatus;
  uint32_t vsize;
  uint32_t vtop,vpos;
  uint32_t sector;

  if(bank == 0){
	  sector = FLASH_Sector_9;
	  flashaddr = 0x080A0000;
  }else if(bank == 1){
	  sector = FLASH_Sector_10;
	  flashaddr = 0x080C0000;
  }

  sprintf(s, "write bank : %d %d\r\n", bank, sector);
  tmcom_puts(s);

  //erase flash
  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
  if(FLASH_EraseSector(sector, VoltageRange_3) != FLASH_COMPLETE){
	  tmcom_puts("flash erase err.\r\n");
  }else{
	  tmcom_puts("flash erase success.\r\n");
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(Joint) * SV_VOL;
	  sprintf(s, "joint data         : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)joint;
	  while((vpos < vtop + vsize) && (addr < flashaddr + 0x800) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
		  vpos += 2;
	  }
	  flashaddr = 0x080FF800;
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(SensorUniqueData);
	  sprintf(s, "sensor unique data : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)&sud;
	  while((vpos < vtop + vsize) && (addr < flashaddr + 0x300) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
	      vpos += 2;
	  }
      flashaddr = 0x080FFC00;
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(Parameters);
	  sprintf(s, "parameters         : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)&param;
	  while((vpos < vtop + vsize) && (addr < flashaddr + 0x300) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
		  vpos += 2;
	  }
	  tmcom_puts("flash write complete.\r\n");
  }
  FLASH_Lock();
}

void ik_walk(int l)
{
  
}

#ifdef WALK
void walk(void)
{
//  FLOAT walk_x, walk_bx, walk_y;
  FLOAT start_depth, end_depth;
  FLOAT depth_rate;
  int i;
  unsigned short res;
  
  if(walk_on || walk_end){
    if(walk_cycle++ == 0){
      //歩のセッティング

      //depthが+なら右足が前
      //        depth  io  start_y  end_y
      //forward     +   -        -      +
      //forward     -   +        -      +
      //back        +   +        +      -
      //back        -   -        +      -

      if(walk_step == 0){
        // first step
        wps.io = (lbp.p[LBP_DEPTH] < 0 ? 1 : -1) * (wps.stride < 0 ? -1 : 1) * lbp.p[LBP_WIDTH] * 0.5;
      }else{
        // other step
        wps.io = -wps.io;
      }
      wps.start_y = (wps.stride > 0 ? -1 : 1) * (lbp.p[LBP_DEPTH] > 0 ? 1 : -1) * lbp.p[LBP_DEPTH] * 0.5;
      wps.end_y = wps.stride * 0.5;
    }
    // make step motion frame
    calc_walk(&wps, walk_cycle * 0.02, &walk_x, &walk_bx, &walk_y);

    start_depth = wps.start_y * 2;
    end_depth = wps.end_y * 2;
    if(wps.io < 0){
      start_depth = - start_depth;
      end_depth = - end_depth;
    }
    depth_rate = calc_depth(&wps, walk_cycle);
    lbp.p[LBP_DEPTH] = depth_rate * (end_depth - start_depth) + start_depth;
    lbp.p[LBP_HUNG] = wps.hung * sin(depth_rate * M_PI) * (wps.io < 0 ? -1 : 1);
    lbp.o[X] = walk_x - wps.io;
    lbp.o[Y] = (wps.io < 0 ? -1 : 1) * lbp.p[LBP_DEPTH] * 0.5 - walk_y;
    sign_io = wps.io < 0 ? -1 : 1;
    sign_stride = wps.stride < 0 ? -1 : 1;

    if(walk_cycle >= wps.w_cycle){
      walk_cycle = 0;
      walk_step++;
    }
    if(walk_end && walk_cycle == 0){
      walk_on = 0;
      walk_end = 0;
    }
//    walk_data = 1;
    res = lbp_cnv_l_sv(&lbp, gp, j);
    if(res == 0){
      for(i = 0; i < LEGS_SERVO_QT; i++){
        joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
        joint[i].d[JOINT_TIME] = 0;
        joint[i].d[JOINT_CMD] = SV_POS;
      }
    }else{
      //エラー処理
    }
  }
}
#endif

void sensor_read(void)
{
  int err, i;
  sensor_read_flag = 1;
  err = I2C1_BufferRead(ADXL345, accld, 0x32, 6);
  // 0x32:DATAX0 0x33:DATAX1 0x34:DATAY0 0x35:DATAY1 0x36:DATAZ0 0x37:DATAZ1 

  Node *stab_n;
  static int stab_size = 0;
  FLOAT df;

  int ch_compass;

  if(err){
    //SensorData[]にエラー用データを入れる
    accld[0] = 255;
    err = 0;
  }

  // acc posture
  short acc_x = *(short*)&accld[s_idx[X] * 2] * s_sgn[X];
  short acc_y = *(short*)&accld[s_idx[Y] * 2] * s_sgn[Y];
  short acc_z = *(short*)&accld[s_idx[Z] * 2] * s_sgn[Z];
  acc_angle[X] = atan2f(acc_y, acc_z);  //pitch
  acc_angle[Y] = -atan2f(acc_x , sqrtf(acc_y * acc_y + acc_z * acc_z)); //roll
  acc_angle[Z] = 0; //yaw

  err = I2C1_BufferRead(ITG3200, gyrod, 0x1d, 6);
  // 1D:GYRO_XOUT_H 1E:GYRO_XOUT_L 1F:GYRO_YOUT_H 20:GYRO_YOUT_L 21:GYRO_ZOUT_H 22:GYRO_ZOUT_L
  if(err){
    //SensorData[]にエラー用データを入れる
    gyrod[0] = 255;
    err = 0;
  }
  for(i = 0; i < 3; i++){
    conv_endian(&gyrod[i*2]);
  }

  //stability
  stab_n = list_push_back(&stab);
  stab_size++;

  for(i = 0; i < 3; i++){
    short gyro_tmp = (*(short*)&gyrod[s_idx[i]*2] - param.i[s_idx[i]]) * s_sgn[i];
    FLOAT gyro_d = (FLOAT)gyro_tmp * M_PI / (14.375 * 180);
    gyro_angle[i] = 0.95 * (gyro_angle_bfr[i] + gyro_d * interval) + 0.05 * acc_angle[i];
    df = gyro_angle_bfr[i] - gyro_angle[i];
    stab_n->d[i] = df * df;
    gyro_angle_bfr[i] = gyro_angle[i];
  }

  vct_add(stab_n->d, stability);
  if(stab_size > 20){
	  vct_sub(stab.head->d, stability);
	  list_pop_front(&stab);
	  stab_size--;
  }
  stability_update = 1;

#ifdef QMC5883L
  /*err = I2C3_BufferRead(QMC5883L, &ch_compass, 0x06, 1);
  if(ch_compass){
	  err = I2C3_BufferRead(QMC5883L, compass, 0x00, 6);
  }*/
#endif
  //zmp for foot pressure
  if(!legpressure_row_flag){
    for(i = 0; i < 8; i++){
      int tmp;
      if(ADC1ConvertedValue[i] > sud.lp_adc_max[i]) lp_adc[i] = 0;
      else lp_adc[i] = sud.lp_adc_max[i] - ADC1ConvertedValue[i];
    }
    zmp[ZMP_RP] = (lp_adc[0] + lp_adc[1] + lp_adc[2] + lp_adc[3]);
    zmp[ZMP_RX] = (lp_adc[1] + lp_adc[3] - lp_adc[0] - lp_adc[2]) * 2 / zmp[ZMP_RP];
    zmp[ZMP_RY] = (lp_adc[0] + lp_adc[1] - lp_adc[2] - lp_adc[3]) * 2 / zmp[ZMP_RP];
    zmp[ZMP_LP] = (lp_adc[4] + lp_adc[5] + lp_adc[6] + lp_adc[7]);
    zmp[ZMP_LX] = (lp_adc[5] + lp_adc[7] - lp_adc[4] - lp_adc[6]) * 2 / zmp[ZMP_LP];
    zmp[ZMP_LY] = (lp_adc[4] + lp_adc[5] - lp_adc[6] - lp_adc[7]) * 2 / zmp[ZMP_LP];
  }else{
    for(i = 0; i < 8; i++){
      lp_adc[i] = ADC1ConvertedValue[i];
      if(lp_adc[i] > sud.lp_adc_max[i]) sud.lp_adc_max[i] = lp_adc[i];
      if(lp_adc[i] < sud.lp_adc_min[i]) sud.lp_adc_min[i] = lp_adc[i];
    }
  }
  sensor_read_flag = 0;
}

void led_blink(void)
{
volatile static int c = 0;
  if(c++ > 4){
    if(b % 2){
    	//LED ON
#if defined(STBEE) || defined(STBEEMINI)
      GPIO_ResetBits(LED01);
#else
#ifdef STBEEF4
      GPIO_SetBits(LED01);
#endif
#endif
    }else{
    	//LED OFF
#if defined(STBEE) || defined(STBEEMINI)
      GPIO_SetBits(LED01);
#else
#ifdef STBEEF4
      GPIO_ResetBits(LED01);
#endif
#endif
    }
    b++;
    c = 0;
  }
}

char* strtok_(char* str, char delim)
{
  volatile static char *sptr;
  char *ptr, *eptr;

  if(str) ptr = str;
  else ptr = (char*)sptr;
  eptr = ptr;
  if(*eptr){
    while(*eptr){
      if(*eptr == delim){
        *eptr = 0;
        eptr++;
        break;
      }
      eptr++;
    }
    sptr = eptr;
    return ptr;
  }else{
    return NULL;
  }

}

int str_cpy(char *dst, char *src){
  int l = 0;
  char *sp = src, *dp = dst;
  while(*sp){
    *dp = *sp;
    sp++;
    dp++;
    l++;
  }
  return l;
}

void str_cpyn(char *dst, char *src, int n){
  int i;
  char *sp = src, *dp = dst;
  for(i = 0; i < n; i++){
    *dp = *sp;
    sp++;
    dp++;
  }
}

int str_len(char *str){
  int l = 0;
  while(str[l]){
    l++;
  }
  return l;
}

void out1h(char *txt, unsigned char value)
{
  value &= 0x0F;
  if( value < 10 ) {
    value += '0';
  }
  else {
    value -= 10;
    value += 'A';
  }
  *txt = value;
}

int out2h(char *txt, unsigned char value)
{
  out1h(txt, value >>  4 );
  txt++;
  out1h(txt, value );
  txt++;
  return 2;
}

int out4h(char *txt, unsigned short value)
{
  out2h(txt, value >> 8);
  txt += 2;
  out2h(txt, value & 0xff);
  return 4;
}

int bin2hex(int bn, char *bin, char *hex)
{
  int i;
  char *ptr;
  ptr = hex;
  for(i = 0; i < bn; i++){
    out2h(ptr, bin[i]);
    ptr++; ptr++;
  }
  *ptr = '\r'; ptr++;
  *ptr = '\n'; ptr++;
  *ptr = 0;
  return str_len(hex);
}

int outval(char *txt, int val)
{
  int	flag = 0;
  unsigned int	order, number;
  int c = 0;

  order = 1000000000UL;

  if( val < 0 ) {
    *txt = '-';
    val = -val;
    txt++;
    c++;
  }
  while( order > 0 ) {
    number = val / order;
    if( number > 0 || flag == 1 ) {
      flag = 1;
      val -= number * order;
      *txt = '0' + number;
      txt++;
      c++;
    }
    order /= 10;
  }
  if( flag == 0 ) {
    *txt = '0';
    c++;
  }
  return c;
}

int outfloat(char *txt, float val)
{
  long upper;
  float lower;
  float digit = 10.0;
  int i = 0;
  int c = 0, n;
  
  if(isnan(val)){
    c = str_cpy(txt, "NaN");
  }else if(val > 10000000000.0){
    c = str_cpy(txt, "Too BIG");
  }else if(val < -10000000000.0){
    c = str_cpy(txt, "Too SMALL");
  }else{
    if(val < 0){
      *txt = '-';
      txt++; c++;
      val = -val;
    }
    upper = val;
    lower = val - (float)upper;
    n = outval(txt, upper);
    txt += n;
    c += n;
    *txt = '.';
    txt++; c++;
    while(lower * digit < 1.0 && i < 5){
    	*txt = '0';
    	txt++; c++;
    	digit *= 10.0;
    	i++;
    }
    c += outval(txt, (int)(lower * 1000000.0));
  }
  return c;
}

void add_eol(char *txt)
{
  *txt = '\r';
  *(++txt) = '\n';
  *(++txt) = 0;
}

void conv_endian(char *val)
{
  char t;
  t = val[0];
  val[0] = val[1];
  val[1] = t;
}

void show_sw_status(int sw)
{
  tmcom_puts(cm_str[sw]); tmcom_puts(" : "); tmcom_outval(sw_status[sw - SW_PRM_MON]); tmcom_puts("\r\n");
}

void arm_data_set()
{
	for(int i = 0; i < 4; i++){
		arm_data[1][i] = param.f[_default_ubp_1 + i];
		arm_data[0][i] = param.i[_arm_swing_b_1 + i];
		arm_data[2][i] = param.i[_arm_swing_f_1 + i];
		arm_data[1][i+4] = (i == 0 || i == 2 ? 1 : -1) * param.f[_default_ubp_1 + i];
		arm_data[0][i+4] = (i == 0 || i == 2 ? 1 : -1) * param.i[_arm_swing_f_1 + i];
		arm_data[2][i+4] = (i == 0 || i == 2 ? 1 : -1) * param.i[_arm_swing_b_1 + i];
	}
	//for(int i = 0; i < 8; i++){
	//	sprintf(s, "%f %f %f \r\n", arm_data[0][i], arm_data[1][i], arm_data[2][i]);
	//	tmcom_puts(s);
	//}
}

void arm_swing_action(FLOAT step, FLOAT depth, FLOAT *j, int sw)
{
  int index;
  FLOAT rate;
  if (depth == 0){
    index = 1;
    rate = 1.0;
  }
  else{
    index = depth < 0 ? 0 : 2;
    rate = fabs(depth / step) * 0.5;
  }
  if(sw){
  for (int i = 0; i < 8; i++){
    j[LEGS_SERVO_QT + i] = ((arm_data[index][i] - arm_data[1][i]) * rate + arm_data[1][i]) * M_PI / 180.0f;
  }
  }
}

void zmp_walk(void)
{
	massPoint *m_zmp;
	FLOAT dk_zmp[12];
	FLOAT gangle_zmp;
	FLOAT gt_zmp[3];

	//リビルドデータがあれば、差し替えを実施
	if (zmp_walk_cont_on == 4 && zmpplan.head == rmk_start_node){  //予見データ再構築完了フラグが立っているなら
		//目標ZMP列の接続点から後ろを削除
		while (zmpplan.tail != cp_node){
			list_pop_back(&zmpplan);
		}
		list_pop_back(&zmpplan);
		//ZMP列を接続
		list_chain(&zmpplan, &zmpplan2);
		prediction_copy(&pred_rmk, &pred);
		//mtp構造体コピー
		motion_para_copy(&mtp_rmk, &mtp_fs);
		//lbpコピー
		lbp_copy(&lbp_rmk, &lbp_fs);
		//ZMP差分リスト(dzmp)の入れ替え
/*		Node *nd = dzmp.head;
		tmcom_puts("dzmp,");
		while(nd){
			sprintf(s,"%f,", nd->d[X] );
			tmcom_puts(s);
			nd = nd->next;
		}
		tmcom_puts("\r\n");
		nd = dzmp_rmk.head;
		tmcom_puts("_rmk,");
		while(nd){
			sprintf(s,"%f,", nd->d[X] );
			tmcom_puts(s);
			nd = nd->next;
		}
		tmcom_puts("\r\n");*/
		list_clear(&dzmp);
		list_chain(&dzmp, &dzmp_rmk);
		//質点モデル内データのコピー（変更(予見項再構築で最新化された)→予見）
		base_get_gp(&robot, j, REMAKE, FORESEE);
		zmpp = zmpp_rmk;
		bzmpp = bzmpp_rmk;
        walk_rest_frame = list_size(&zmpplan);
		zmp_walk_cont_on = 0;
	}

	if(zmpplan.head == NULL){
		zmp_walk_on = 0;
		sensor_out_flag = 0;  // verbose off
		return;
	}
#ifdef USE_ZMP_DF
//動力学フィルター予見分計算
//動力学フィルター用のZMP差分列を作成
  Node *dzn;
  FLOAT adjz[3] = { 0, 0, 0 };
  FLOAT rz[3];
  static int inres = 0;
  //予見項分未来のgt[]決定
  zmp_walk_online(interval, zmpp, gt_zmp, &zx, &zy, &pred, &inres, ini);
  //lbp決定
  cnt = zmp_make_walk_motion(gt_zmp, zmpp, &lbp_fs, s_pivot, n_pivot, param.f[_hung], param.i[_delay], &gangle_zmp, &mtp_fs, &dcp, &dcp_pivot);
  //関節角度列j[]決定
  if((res = lbp_cnv_l_sv(&lbp_fs, gp, j)) != 0){
      //error
	  sprintf(s, "lbp error : %d (foresee section)\r\n", res);
	  tmcom_puts(s);
  }
#ifdef ARM_SWING
  arm_swing_action(step, lbp_fs.p[LBP_DEPTH], j, 0);
#endif
  //ZMP算出
  gt_zmp[Z] = lbp_fs.p[LBP_HEIGHT] + lbp_fs.o[Z] + param.f[_gtz_adjust];
  m_zmp = robot.blist->mphead->child->child;
  biarticular_dk(j, dk_zmp, 1, m_zmp);
  m_zmp = robot.blist->next->mphead->child->child;
  biarticular_dk(&j[LEG_LEFT], &dk_zmp[LEG_DK_LEFT], 2, m_zmp);
  base_get_zmp(&robot, gt_zmp, gangle_zmp, j, interval, NOLOAD, FORESEE);
  //ZMP差分格納
  dzn = list_push_back(&dzmp);
  vct_copy(bzmpp->d, dzn->d);
  vct_sub(robot.zmp[FORESEE], dzn->d); //dzn->d - robot.zmp
  vct_sub(lbp_fs.g, dzn->d);     //dzn->d - lbp.g => Pref - P
  bzmpp = zmpp;
  if (zmpp->next != NULL){
    zmpp = zmpp->next;
  }

  // ZMP列変更に備えた予備予見器の更新 START
  zmp_walk_online(interval, zmpplan.head, gt_zmp, &zx, &zy, &pred_rsv, &inres_rsv, lbp_rsv.o);
  //lbp決定
  cnt = zmp_make_walk_motion(gt_zmp, zmpplan.head, &lbp_rsv, s_pivot, n_pivot, param.f[_hung], param.i[_delay], &gangle_zmp, &mtp_rsv, &dcp, &dcp_pivot);
  //関節角度列j[]決定
  if((res = lbp_cnv_l_sv(&lbp_rsv, gp, j)) != 0){
    //error
	  sprintf(s, "lbp error : %d (preliminary section)\r\n", res);
	  tmcom_puts(s);
  }
#ifdef ARM_SWING
  arm_swing_action(step, lbp_rsv.p[LBP_DEPTH], j, 0);
#endif
#if 0    // ZMP列変更時の再構築のための質点モデル計算は不要（rebuid時の遅延時間で実施する）
  //ZMP算出
  gt_zmp[Z] = lbp_rsv.p[LBP_HEIGHT] + lbp_rsv.o[Z] + param.f[_gtz_adjust];
  m_zmp = robot.blist->mphead->child->child;
  biarticular_dk(j, dk, 1, m);
  m_zmp = robot.blist->next->mphead->child->child;
  biarticular_dk(&j[LEG_LEFT], &dk[LEG_DK_LEFT], 2, m_zmp);
  m_zmp = robot.blist->next->mphead->child->child;
  base_get_zmp(&robot, gt_zmp, gangle_zmp, j_zmp, interval, NOLOAD, RESERVE);
  // ZMP列変更に備えた予備予見器の更新 END
#endif
  
  //補正値算出
  dynamics_filter(interval, dzmp.head, adjz, rz, &pred_df, &res_pred);  //res_predは予見データ作成部で=1としている。

  list_pop_front(&dzmp);
  vct_add(adjz, gt_zmp);
  //姿勢決定
  cnt = zmp_make_walk_motion(gt_zmp, zmpplan.head, &lbp, s_pivot, n_pivot, param.f[_hung], param.i[_delay], &gangle_zmp, &mtp, &dcp, &dcp_pivot);
////////////////////////////
  /*
　　lbp を　leg　にかえて、legにアレンジを加えてサーボに与えるようにする。
*/
  ////////////////////////////
  FLOAT leg_r[6], leg_l[6];
  unsigned short reslt0, reslt1;

  walkstatus[walkstatus_index].dx = lbp.o[X] - bfr_lbp.o[X];
  walkstatus[walkstatus_index].ddx = walkstatus[walkstatus_index].dx - walkstatus[walkStatusIndex(walkstatus_index, -1)].dx;
  int top = walkstatus[walkstatus_index].dx * walkstatus[walkStatusIndex(walkstatus_index, -1)].dx < 0 ? 1 : 0;
  int zmp_change = walkstatus[walkstatus_index].ddx * walkstatus[walkStatusIndex(walkstatus_index, -1)].ddx < 0 ? 1 : 0;
  walkstatus[walkstatus_index].hdx = lbp.p[LBP_HUNG] - bfr_lbp.p[LBP_HUNG];
  int hung_top = walkstatus[walkstatus_index].hdx * walkstatus[walkStatusIndex(walkstatus_index, -2)].hdx < 0 ? 1 : 0;
  int hung_start_end = lbp.p[LBP_HUNG] * bfr_lbp.p[LBP_HUNG] == 0 && lbp.p[LBP_HUNG] + bfr_lbp.p[LBP_HUNG] != 0 ? 1 : 0;

  if(top){
	  walkstatus[walkstatus_index].pivot = plsTop;
	  walkstatus[walkstatus_index].pivot_count = 0;
  }else if(zmp_change && walkstatus_zmpchange_reject_counter > WS_ZMPCHANGE_REJECT_COUNT){
	  walkstatus[walkstatus_index].pivot = plsChange;
	  walkstatus[walkstatus_index].pivot_count = 0;
  }else{
	  walkstatus[walkstatus_index].pivot = walkstatus[walkStatusIndex(walkstatus_index, -1)].pivot;
	  walkstatus[walkstatus_index].pivot_count = walkstatus[walkStatusIndex(walkstatus_index, -1)].pivot_count + 1;
  }

  if(hung_top){
	  walkstatus[walkstatus_index].free = flsTop;
	  walkstatus[walkstatus_index].free_count = 0;
  }else if(hung_start_end){
	  if(walkstatus[walkStatusIndex(walkstatus_index, -1)].free == flsTop){
		  walkstatus[walkstatus_index].free = flsTouch;
	  }else{
		  walkstatus[walkstatus_index].free = flsTakeoff;
	  }
	  walkstatus[walkstatus_index].free_count = 0;
  }else{
	  walkstatus[walkstatus_index].free = walkstatus[walkStatusIndex(walkstatus_index, -1)].free;
	  walkstatus[walkstatus_index].free_count = walkstatus[walkStatusIndex(walkstatus_index, -1)].free_count + 1;
  }
  if(walkstatus[walkstatus_index].pivot == plsChange && walkstatus[walkstatus_index].pivot_count == 0){
	  if(walkstatus[walkStatusIndex(walkstatus_index, -1)].leg == plBoth){
		  if(lbp.o[X] > 0){
			  walkstatus[walkstatus_index].leg = plRight;
		  }else{
			  walkstatus[walkstatus_index].leg = plLeft;
		  }
	  }else{
		  if(walkstatus[walkStatusIndex(walkstatus_index, -1)].leg == plRight){
			  walkstatus[walkstatus_index].leg = plLeft;
		  }else{
			  walkstatus[walkstatus_index].leg = plRight;
		  }
	  }
  }else{
	  walkstatus[walkstatus_index].leg = walkstatus[walkStatusIndex(walkstatus_index, -1)].leg;
  }
  walkstatus_zmpchange_reject_counter++;

 static int step_count;
 static int bfr_pivot;
 if(zmpplan.head->di[1] == -1){
	 step_count = 0;
	 bfr_pivot = -1;
 }else{
	 if(bfr_pivot != zmpplan.head->di[1]){
		 step_count = 0;
	 }else{
		 step_count++;
	 }
 }
 bfr_pivot = zmpplan.head->di[1];

 if(zmpplan.head->di[1] > -1){
	 walk_tilt_accumulate(step_count, param.f[_walk_follow_threshold], &walk_tilt, zmp[zmpplan.head->di[1] * 3]);
	 // ZMP_RX = 0   pivot_right = 0
	 // ZMP_LX = 3   pivot_left = 1
	 if(wfa_sw == 0){
		 if(walk_tilt > param.f[_walk_follow_threshold] * param.i[_walk_follow_threshold_times]){
			 walk_follow_s = 1;
			 walk_follow = 1;
			 sprintf(s,"walk follow f : %f \r\n", walk_tilt);
			 tmcom_puts(s);
		 }else if(walk_tilt < -param.f[_walk_follow_threshold] * param.i[_walk_follow_threshold_times]){
			 walk_follow_s = 1;
			 walk_follow = -1;
			 sprintf(s,"walk follow b : %f \r\n", walk_tilt);
			 tmcom_puts(s);
		 }
	 }
 }


 if(walk_follow_s){
	 walk_follow_action_ini(step_count, param.i[_delay]);
	 walk_follow_s = 0;
 }

 // up down action =>左右のスイング動作時に上下動を加えて重力加速度の盈虚を軽減させる狙い。オーバースイングの場合に重心を上に加速すれば重力方向の加速度が増してオーバースイングの抑制になるという狙い
 FLOAT uda; //swing up
 FLOAT wfa; //walk follow
 FLOAT wfab;
 if(zmpplan.head->di[1] != -1){
	 uda = up_down_action(step_count, interval, param.f[_up_down_action_accel]);
	 wfab = walk_follow_back(step_count);
	 wfa = walk_follow_action(step_count, walk_follow, zmpplan.head->di[1], param.f[_walk_follow_max] * walk_follow_ratio);
 }else{
	 uda = 0;
	 wfa = 0;
 }


// sprintf(s, "%f,%d,%d,%f,%f\r\n", lbp.p[LBP_HUNG], zmpplan.head->di[1], step_count,wfa, wfab);
// tmcom_puts(s);
   //x, pls, pivot_count, leg, top, zmp_change, hung, fls, free_count, hung_top, hung_start_end, zmp, pivot, zmpL, zmpR
  //sprintf(s, "%f,%f,%d,%d,%d,%d,%d,%f,%d,%d,%d,%d,%f,%d,%f,%f,%f,%f,%d,%f\r\n", lbp.p[LBP_WIDTH], lbp.o[X], walkstatus[walkstatus_index].pivot, walkstatus[walkstatus_index].pivot_count, walkstatus[walkstatus_index].leg, top, zmp_change, lbp.p[LBP_HUNG], walkstatus[walkstatus_index].free, walkstatus[walkstatus_index].free_count, hung_top, hung_start_end, zmpplan.head->d[X], zmpplan.head->di[1], zmp[ZMP_LX], zmp[ZMP_RX], zmp[ZMP_LY], zmp[ZMP_RY],step_count,uda);
  //tmcom_puts(s);

  //足を上げた途端に体が内側に倒れる事を検知
  //計画
  //足を上げるフェーズに入ったら胴体ロール軸傾きを監視
  //規定フレームで閾値を超える傾きを記録したら検知とする。
#if 1
  if(walkstatus[walkstatus_index].free == flsTakeoff){
	  if(walkstatus[walkstatus_index].free_count == 0){
		  innerfall_mark = gyro_angle[Y];
	  }else if(walkstatus[walkstatus_index].free_count == param.i[_innerfall_timing]){
		  //sprintf(s, "checktiming\r\n");
		  //tmcom_puts(s);
		  if((gyro_angle[Y] - innerfall_mark) * (walkstatus[walkstatus_index].leg == plRight ? -1 : 1) > param.i[_innerfall_threshold]){
			  //検知
			  sprintf(s, "detect! mark=%d gyro_angle[y]=%f\r\n", innerfall_mark, gyro_angle[Y]);
			  tmcom_puts(s);
		  }
	  }
  }
#endif

  lbp_copy(&lbp, &bfr_lbp);
  walkstatus_index = walkStatusIndex(walkstatus_index, 1);

//Body roll adjust
//  lbp.p[LBP_ROLL] = body_roll_adj() * (-2 * n_pivot + 1);

  lbp.o[Y] += param.f[_com_calibration_amount];
  lbp.p[LBP_HEIGHT] += uda;
  lbp.p[LBP_DEPTH] += wfa + wfab;
  if(gyro_feedback_on){
	  FLOAT roll = gyro_angle[Y] + param.f[_gyro_angle_offset_roll] - lbp.p[LBP_ROLL];
	  FLOAT pitch = gyro_angle[X] + param.f[_gyro_angle_offset_pitch] - lbp.p[LBP_PITCH];
	  if(fabs(roll) > param.f[_criteria_of_standup] || fabs(pitch) > param.f[_criteria_of_standup]){
		//こけている間は累積させない（クリアし続ける）
  		clear_pid2(&gyroFeedbackPid0);
  		clear_pid2(&gyroFeedbackPid1);
	  }
	  lbp_copy(&lbp, &gf_lbp);
	  calc_pid2(&gyroFeedbackPid0, &param.f[_pid0_p_fact], roll); //roll
	  calc_pid2(&gyroFeedbackPid1, &param.f[_pid1_p_fact], pitch); //pitch
	  if(gyroFeedbackPid0.u != 0 || gyroFeedbackPid1.u != 0){
		  if(gyroFeedbackPid0.u != 0){
			  gf_lbp.p[LBP_ROLL] -= gyroFeedbackPid0.u * param.f[_gyro_roll_feedback_retio];
		  }
		  if(gyroFeedbackPid1.u != 0){
#ifdef GYROFEEDBACK_TYPE1
			  gf_lbp.p[LBP_PITCH] -= gyroFeedbackPid1.u * param.f[_gyro_pitch_feedback_retio];
			  gf_lbp.o[Y] += gyroFeedbackPid1.u * param.f[_pid_y_offset_rate_gyro];
#else
#ifdef GYROFEEDBACK_TYPE2
			  zmp_feedback_ankle = gyroFeedbackPid1.u * param.f[_pos_pitch_calibration_retio];
#endif
#endif
		  }
	  }
	  lbp_cnv_lbp_leg(&gf_lbp, gp, leg_r, leg_l);
  }else{
	  lbp_cnv_lbp_leg(&lbp, gp, leg_r, leg_l);
  }
  lbp.o[Y] -= param.f[_com_calibration_amount];
  lbp.p[LBP_HEIGHT] -= uda;
  lbp.p[LBP_DEPTH] -= wfa + wfab;


  //Pivot leg height adjust
  height_adj_zmp = zmpplan.head->di[1];
  if(height_adj_zmp == 0){  //right
	  leg_r[LEG_Z] -= param.f[_height_adjust];
  }else if(height_adj_zmp == 1){  //left
	  leg_l[LEG_Z] -= param.f[_height_adjust];
  }

  reslt0 = (unsigned short)biarticular_ik(leg_r, j, IK_RIGHT);
  reslt1 = (unsigned short)biarticular_ik(leg_l, &j[LEG_LEFT], IK_LEFT);


  //if((res = lbp_cnv_l_sv(&lbp, gp, j)) != 0){
  if((res = reslt0 | reslt1 << 8) != 0){
      //error
	  if(gyro_feedback_on){
		  //sprintf(s, "lbp error : %d p:%f o:%f (zmp_walk() with gyro feedback)\r\n", res, gf_lbp.p[param.i[_gyro_feedback_debug1]], gf_lbp.o[param.i[_gyro_feedback_debug2]]);
		  lbp_print(&gf_lbp);
	  }else{
		  sprintf(s, "lbp error : %d (zmp_walk())\r\n", res);
	  }
	  tmcom_puts(s);
  }

#ifdef ARM_SWING
  arm_swing_action(step, lbp.p[LBP_DEPTH], j, 1);
#endif
  vct_copy(zmpplan.head->d, bzmp);
  list_pop_front(&zmpplan);
#else
  //gt[]決定
  zmp_walk_online(interval, zmpplan.head, gt, &zx, &zy, &pred, &res_pred, ini);
  gt[Z] = lbp.p[LBP_HEIGHT] + lbp.o[Z] + param.f[_gtz_adjust];
  //lbp決定
  cnt = zmp_make_walk_motion(gt, zmpplan.head, &lbp, s_pivot, n_pivot, param.f[_hung], param.i[_delay], &gangle, &mtp, &dcp, &dcp_pivot);
  //関節角度列j[]決定
  if((res = lbp_cnv_l_sv(&lbp, gp, j)) != 0){
      //error
	  sprintf(s, "lbp error : %d (real secition)\r\n", res);
	  tmcom_puts(s);
  }
#ifdef ARM_SWING
  arm_swing_action(step, lbp.p[LBP_DEPTH], j, 1);
#endif
#if 0
  //ZMP算出
  m = robot.blist->mphead->child->child;
  biarticular_dk(j, dk, 1, m);
  m = robot.blist->mphead->child->child;
  m = robot.blist->next->mphead->child->child;
  biarticular_dk(&j[LEG_LEFT], &dk[LEG_DK_LEFT], 2, m);
  m = robot.blist->next->mphead->child->child;
  base_get_zmp(&robot, gt, gangle, j, interval, NOLOAD, NO_DYN_FILTER, FORESEE);
#endif
  (&zmpplan);
#endif
  if(!res){
	  zmp_walk_data_ready = 1;
  }
}

void zmp_walk_rebuild_pre(int md,FLOAT wd, FLOAT twd, int stp, FLOAT rd, FLOAT pc, int nstp)
{

	//予見データ再構築実施
	list_clear(&zmpplan2);
	//cp_nodecが最終歩なら、next_stepを適用する。
	//cp_nodecの検査
	Node *np;
	FLOAT _d[3];
	np = cp_nodec;
	vct_copy(cp_nodec->d, _d);
	while(np && np->d[X] == _d[X] && np->d[Y] == _d[Y]){
		np = np->next;
	}
	if(!np){
		sprintf(s, "cp_nodec->d %f %f => next_step %f %f\r\n", _d[X], _d[Y], next_step[X], next_step[Y]);
		tmcom_puts(s);
		vct_copy(next_step, _d);
	}
	//継続分のZMP列を生成
	if (md == STRATE){
		zmp_make_plan(cp_pivot, n_pivot, wd, stp, pc, nstp, interval, _d, ini, &zmpplan2);
	}
	else if (md == CURVE){
		zmp_make_plan_curve(cp_pivot, n_pivot, wd, stp, rd, pc, nstp, interval, _d, ini, &zmpplan2);
	}
	else if (md == TURN){
		zmp_make_plan_turn(cp_pivot, n_pivot, wd, twd, stp, pc, nstp, interval, _d, ini, &zmpplan2);
	}
	else if (md == SIDE){
		zmp_make_plan_side(cp_pivot, n_pivot, wd, stp, pc, nstp, interval, _d, ini, &zmpplan2);
	}
}

void zmp_walk_rebuild(int i)
{
	//新規ZMP列で予見データを生成
	massPoint *m_rmk;
	Node *dzn, *_dcp;
	int _dcp_pivot;
	FLOAT gt_rmk[3];
	FLOAT j_rmk[22], dk_rmk[12];
	FLOAT gangle_rmk;
	static int res_rmk;

	if(i == 0){
		zmpp_rmk = zmpplanc.head;
		bzmpp_rmk = NULL;
		list_clear(&dzmp_rmk);
		res_rmk = 0;
	}

		zmp_walk_online(interval, zmpp_rmk, gt_rmk, &zx, &zy, &pred_rmk, &res_rmk, ini);
		//lbp決定
		cnt = zmp_make_walk_motion(gt_rmk, zmpp_rmk, &lbp_rmk, s_pivot, n_pivot, param.f[_hung], param.i[_delay], &gangle_rmk, &mtp_rmk, &_dcp, &_dcp_pivot);
		//関節角度列j[]決定
#ifdef ARM_SWING
		arm_swing_action(step, lbp_rmk.p[LBP_DEPTH], j_rmk, 0);
#endif
		if(i > param.i[_d_step] - 3){
			int r = lbp_cnv_l_sv(&lbp_rmk, gp, j_rmk);
			if (r){  //error
				  sprintf(s, "lbp error : %d (walk_cont section)\r\n", res);
				  tmcom_puts(s);
			}
			gt_rmk[Z] = lbp_rmk.p[LBP_HEIGHT] + lbp_rmk.o[Z] + param.f[_gtz_adjust];
			m_rmk = robot.blist->mphead->child->child;
			biarticular_dk(j_rmk, dk_rmk, 1, m_rmk);
			m_rmk = robot.blist->next->mphead->child->child;
			biarticular_dk(&j_rmk[LEG_LEFT], &dk_rmk[LEG_DK_LEFT], 2, m_rmk);
			base_get_zmp(&robot, gt_rmk, gangle_rmk, j_rmk, interval, NOLOAD, REMAKE);
			dzn = list_push_back(&dzmp_rmk);
			if (!bzmpp_rmk){              //D_STEP-3以降でしか差分計算しないならbzmpの準備は不要
				vct_copy(bzmpc, dzn->d);
			}
			else{
				vct_copy(bzmpp_rmk->d, dzn->d);
			}
			vct_sub(robot.zmp[REMAKE], dzn->d); //dzn->d - robot.zmp
			vct_sub(lbp_rmk.g, dzn->d);     //dzn->d - lbp.g => Pref - P
			if(i > F_STEP + param.i[_d_step] - 3){
				list_pop_front(&dzmp_rmk);
			}
		}
		bzmpp_rmk = zmpp_rmk;
		if (zmpp_rmk->next != NULL){
			zmpp_rmk = zmpp_rmk->next;
			if (zmpp_rmk == cp_nodec){
				zmpp_rmk = zmpplan2.head;
			}
		}
}

void search_cp(void)
{
	cp_nodec = zmpplanc.tail;
	cp_node = zmpplan.tail;
	while(cp_node->prev){  //zmpplanが短くなっていることを考慮して後ろからzmpplanの先頭と同じ位置のzmpplancを探す
		cp_nodec = cp_nodec->prev;
		cp_node = cp_node->prev;
	}
	cp_space = 0;
	while(cp_node && cp_space++ < param.i[_d_step]){
		cp_node = cp_node->next;
		cp_nodec = cp_nodec->next;
	}
	rmk_start_node = cp_node;
	FLOAT zx = cp_node->d[X];
	FLOAT zy = cp_node->d[Y];
	cp_space = 0;
	while(cp_node && zx == cp_node->d[X] && zy == cp_node->d[Y]){
		cp_node = cp_node->next;
		cp_nodec = cp_nodec->next;
		cp_space++;
	}
	if(cp_node){
		cp_pivot = (cp_node->prev->di[1] + 1) % 2;
		sprintf(s, "cp_pivot %d cp_node_pivot %d\r\n", cp_pivot, cp_node->di[1]);
		tmcom_puts(s);
	}
	sprintf(s, "search_cp : %d\r\n", cp_space);
	tmcom_puts(s);
}

void zmp_walk_data_load(void)
{
  int i;

  if(zmp_walk_data_ready){
    j[joint_right_ankle_pitch] += param.f[_ankle_pitch_calibration_amount] + param.f[_zmp_align_amount];
    j[joint_left_ankle_pitch] += param.f[_ankle_pitch_calibration_amount];
    j[joint_right_ankle_roll] += param.f[_ankle_roll_right_calibration_amount];
    j[joint_left_ankle_roll] += param.f[_ankle_roll_left_calibration_amount];
#ifdef GYROFEEDBACK_TYPE2
	  if(gyro_feedback_on){
		  j[joint_right_ankle_pitch] += zmp_feedback_ankle;
		  j[joint_left_ankle_pitch] += zmp_feedback_ankle;
	  }
#endif
    for(i = 0; i < SV_VOL; i++){
      joint[i].d[JOINT_TARGET] = conv_f_sv(j[i]);
      joint[i].d[JOINT_TIME] = 0;
      joint[i].d[JOINT_CMD] = SV_POS;
    }
    zmp_walk_data_ready = 0;
    walk_rest_frame--;
  }
}

void store_log(void)
{
	NodeS *wlogn;

	if(zmp_walk_on && store_log_on){
	    wlogn = listS_push_back(&walklog);
	    for(int i = 0; i < 14; i++){
	    	wlogn->d[i] = joint[i].d[JOINT_TARGET];
	    	wlogn->d[i+14] = (joint[i].d[JOINT_CAPTURE] - joint[i].d[JOINT_TRIM] - joint[i].d[JOINT_OFFSET] - 7500) * joint[i].d[JOINT_SIGN];
	    }
        for(int i = 0; i < 3; i++){
          wlogn->d[i+28] = *(short*)&accld[i*2];
          wlogn->d[i+31] = *(short*)&gyrod[i*2];
        }
        for(int i = 0; i < 8; i++){
        	wlogn->d[i+34] = lp_adc[i];
        }
        wlogn->d[42] = (short)zmpplan.head->d[X];
        wlogn->d[43] = (short)zmpplan.head->d[Y];
        wlogn->d[44] = (short)zmpplan.head->di[0];
        wlogn->d[45] = (short)zmpplan.head->di[1];
	}
}

/*FLOAT body_roll_adj(void)
{
	static body_roll_adj_angle;

	body_roll_adj_count++;
	if(body_roll_adj_count > body_roll_adj_space && body_roll_adj_count <= body_roll_adj_space + body_roll_adj_step + body_roll_adj_stay + body_roll_adj_recover){
		if(body_roll_adj_count <= body_roll_adj_space + body_roll_adj_step){
			return (body_roll_adj_max * (body_roll_adj_count - body_roll_adj_space) / body_roll_adj_step);
		}else{
			if(body_roll_adj_count <= body_roll_adj_space + body_roll_adj_step + body_roll_adj_stay){
				return body_roll_adj_max;
			}else{
				return (body_roll_adj_max * (body_roll_adj_recover - (body_roll_adj_count - body_roll_adj_space - body_roll_adj_step - body_roll_adj_stay)) / body_roll_adj_recover);
			}
		}
	}else{
		return 0;
	}
}
*/

int walkStatusIndex(int wsi, int di)
{
	// -WS_SIZE < di < WS_SIZE
	if(wsi + di >= WS_SIZE){
		return wsi + di - WS_SIZE;
	}else if(wsi + di < 0){
		return wsi + di + WS_SIZE;
	}
	return wsi + di;
}

void walkStatusInit()
{
	int i;
	for (i = 0; i < WS_SIZE; i++){
		walkstatus[i].leg = plBoth;
		walkstatus[i].dir = wdFront;
		walkstatus[i].pivot = plsNop;
		walkstatus[i].free = flsNop;
		walkstatus[i].pivot_count = 0;
		walkstatus[i].free_count = 0;
		walkstatus[i].dx = 0;
		walkstatus[i].ddx = 0;
		walkstatus[i].hdx = 0;
		walkstatus_zmpchange_reject_counter = 0;
	}
}

void exec_motion_data(motionData *md)
{
	exec_motion_frame(&md->frame);
}

void exec_motion()
{
	FLOAT x_diff, y_diff;

	if(motion_play_on != 0){
		if(!motion_play_time){
			if(motion_play_id > -1){
				if(motion_play_on < 0){
					motion_play_on = 1;
					motion_play_frame = motion_info.table[motion_play_id].start;
					motion_play_frame_count = 0;
					motion_rest_frame = 0;
					motion_emergency_stop = 0;
					for(int i = motion_play_frame; i < motion_info.table[motion_play_id].steps; i++){
						motion_rest_frame += motion_frame[i].time;
					}
				}else{
					//angle check
					x_diff = gyro_angle[X] - motion_bfr_angle[X];
					y_diff = gyro_angle[Y] - motion_bfr_angle[Y];
					if(!motion_stability_check_cansel && (fabs(x_diff) > param.f[_criteria_of_motion_posture] || fabs(y_diff) > param.f[_criteria_of_motion_posture])){
						motion_emergency_stop = 1;
					}else{
						motion_play_frame++;
						motion_play_frame_count++;
					}
				}
				if(motion_play_frame_count < motion_info.table[motion_play_id].steps && !motion_emergency_stop){
					motion_play_time = motion_frame[motion_play_frame].time;
					exec_motion_frame(&motion_frame[motion_play_frame]);

				}else{
					if(motion_emergency_stop){
						sprintf(s, "!!emergency stop!! xd:%f yd:%f\r\n", x_diff, y_diff);
						tmcom_puts(s);
					}
					motion_play_on = 0;
					motion_rest_frame = 0;
				}
			}else{  //play motion list
				if(motion_play_on < 0){
					motion_play_on = 1;
					motion_play_data = motion.head;
					motion_emergency_stop = 0;
				}else{
					//angle check
					x_diff = gyro_angle[X] - motion_bfr_angle[X];
					y_diff = gyro_angle[Y] - motion_bfr_angle[Y];
					if(!motion_stability_check_cansel && (fabs(x_diff) > param.f[_criteria_of_motion_posture] || fabs(y_diff) > param.f[_criteria_of_motion_posture])){
						motion_emergency_stop = 1;
					}else{
						motion_play_data = motion_play_data->next;
					}
				}
				if(motion_play_data && !motion_emergency_stop){
					sprintf(s, "[%lx]\r\n", motion_play_data);
					tmcom_puts(s);
					motion_play_time = motion_play_data->frame.time;
					exec_motion_frame(&motion_play_data->frame);
				}else{
					if(motion_emergency_stop){
						sprintf(s, "!!emergency stop!! xd:%f yd:%f\r\n", x_diff, y_diff);
						tmcom_puts(s);
					}
					motion_play_on = 0;
					motion_rest_frame = 0;
				}
			}
		}else{
			motion_play_time--;
			motion_rest_frame--;
		}
	}
}

void exec_motion_frame(motionFrame *mf)
{
	int stid;
	if(mf->time > 0){
		if(mf->sw[1]){
			stid = LEGS_SERVO_QT;
		}else{
			stid = 0;
		}
		for(int i = 0; i < 4; i++){
		  joint[motion_swid[i]].d[JOINT_STATUS] = mf->sw[0];
		}
		for(int i = stid; i < SV_VOL; i++){
		  joint[i].d[JOINT_TARGET] = mf->dt[i];
		  joint[i].d[JOINT_TIME] = mf->time;
		  joint[i].d[JOINT_CMD] = SV_POS;
		}
		for(int i = 0; i < 2; i++){
			motion_bfr_angle[i] = mf->angle[i];
		}
		motion_stability_check_cansel = mf->sw[3];
		if(mf->sw[4]){ //servo off sw
			for(int i = stid; i < SV_VOL; i++){
			  joint[i].d[JOINT_CMD] = SV_OFF;
			}
		}
	}
	if(mf->sw[2]){
	}else if(strcmp(prm_str[0], "posture") == 0){
        lbp.p[LBP_WIDTH] = param.f[_default_lbp_width];
        lbp.p[LBP_DEPTH] = param.f[_default_lbp_depth];
        lbp.p[LBP_HEIGHT] = param.f[_default_lbp_height];
        lbp.p[LBP_HUNG] = param.f[_default_lbp_hung];
	}
}
