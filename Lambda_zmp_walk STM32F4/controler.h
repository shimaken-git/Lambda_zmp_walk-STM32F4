#ifndef __CONTROLER_H__ 
#define __CONTROLER_H__

// csi レジスタアドレス
#define CSI_REG_ARROW  1
#define CSI_REG_BUTTON 2
#define CSI_REG_AN_R_X 3
#define CSI_REG_AN_R_Y 4
#define CSI_REG_AN_L_X 5
#define CSI_REG_AN_L_Y 6
#define CSI_REG_CNTL   7


//CSI_REG_ARROW
#define ARRW_LEFT    0x80
#define ARRW_DOWN    0x40
#define ARRW_RIGHT   0x20
#define ARRW_UP      0x10
#define ARRW_START   0x08
#define ARRW_R_ANLG  0x04
#define ARRW_L_ANLG  0x02
#define ARRW_SELECT  0x01

//CSI_REG_BUTTON
#define BTTN_4       0x80
#define BTTN_3       0x40
#define BTTN_2       0x20
#define BTTN_1       0x10
#define BTTN_R1      0x08
#define BTTN_L1      0x04
#define BTTN_R2      0x02
#define BTTN_L2      0x01

//CSI_REG_CNTL
#define CNTL_MODE    0x08
#define CNTL_CNCT    0x04
#define CNTL_F2      0x02
#define CNTL_F1      0x01

extern int controler_reg_list[];

typedef struct _controler_config{
  unsigned char controler_directkey_bitmask_noshift[3];
  unsigned char controler_directkey_bitmask_shift[3];
  unsigned char controler_shiftkey_bitmask_all[3];
  unsigned char controler_shiftkey_bitmask[20][3];
  unsigned char controler_commandkey_bitmask[10][3];
  int controler_command_assign[20][8];
  unsigned char controler_directkey_bitmask[10][3];
  unsigned char controler_exclusive_directkey_bitmask[10][3];
  int controler_direct_command_assign[10];
} ControlerConfig;

int controler_key_task(int md, volatile unsigned short *spi_data, volatile float *joy, volatile int *d_bit, volatile int *d_bit_exor, volatile int *scode, volatile int *ccode, volatile int *cnum);

#endif


/*
// ①ダイレクトキーマップの指定
// 　ダイレクトキーが押された状態で、コマンドキーを押すことができる。
// 　プレーン状態とシフト状態で、ダイレクトキーマップを変えることができる。
//   ダイレクトキーを押した状態で、シフトキーを検出できることはできるが、
//   シフトキー押下状態でダイレクトキーとなっていないキーは押下状態を保存できない。

  
char controler_directkey_bitmask_noshift[] = {
  0 | CNTL_R_ANLG | CNTL_L_ANLG,
  0 | CNTL_R1 | CNTL_L1,
  0 | CNTL_MODE,
};

char controler_directkey_bitmask_shift[] = {
  0 | CNTL_R_ANLG | CNTL_L_ANLG,
  0 | CNTL_R1 | CNTL_L1,
  0 | CNTL_MODE,
};

// ②シフトキー登録
// 　シフト付きコマンドは、複数キーが同時にビットが立つだけではなく、その前にシフトキーが押された経緯が必要。
// 　「S」からの「SC」S:をシフトキー、Cをコマンドキーとする。
// 　「S」のステップなしに「SC」を押してもだめ。これは、「C」状態から「SC」になった場合を無効とするため。
//   シフト状態はコマンドに関係なく取り出せる。
  
  シフトパターンすべてをANDしたもの。
//char controler_shiftkey_bitmask_all[3] =
    {
      0 | CNTL_LEFT | CNTL_DOWN | CNTL_RIGHT | CNTL_UP,
      0 | CNTL_R2 | CNTL_L2,
      0 ,
    },

char controler_shiftkey_bitmask[][3] = {
  { 0, 0, 0 },
  { CNTL_LEFT,  0, 0 },
  { CNTL_DOWN,  0, 0 },
  { CNTL_RIGHT, 0, 0 },
  { CNTL_UP,    0, 0 },
  { CNTL_LEFT | CNTL_UP,    0, 0 },
  { CNTL_DOWN | CNTL_LEFT,  0, 0 },
  { CNTL_RIGHT | CNTL_DOWN, 0, 0 },
  { CNTL_UP | CNTL_RIGHT,   0, 0 },
  { 0, CNTL_R2 | CNTL_L2, 0 },
  { 0, 0, 0 }
};

char controler_commandkey_bitmask[][3] = {
  { 0, CNTL_1, 0 },
  { 0, CNTL_2, 0 },
  { 0, CNTL_3, 0 },
  { 0, CNTL_4, 0 },
  { CNTL_SELECT, 0, 0 },
  { CNTL_START, 0, 0 },
  { 0, 0, CNTL_F1 },
  { 0, 0, CNTL_F2 },
  { 0, 0, 0 }
};

// controler_command_assign[shift][command]
//       →command
//       ↓shift
int controler_command_assign[][8] = {
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_BASEPOSE, CMD_STANCE_WIDE, CMD_STANCE_NARROW, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_SENSOR_SWITCH, CMD_DAMAGE_RESET, CMD_LASER_SWITCH, CMD_SPEED_SWITCH, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
  {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },
};

// ③ダイレクトキーマスク
// 　ダイレクトキーは同時に複数キーが押されても有効となる場合が多いが、
// 　左右キーなど、同時に押された場合には無効とする必要があるため、排他条件をつける。
// 　controler_directkey_bitmask[1][]の場合、controler_exclusive_directkey_bitmask[1][]とセットで使い、
// 　CNTL_R1が立っているとき、CNTL_L1は立っていてはならない。その他のビットは評価対象外となる。
//   →register
//   ↓direct key list  ※この順番がcommand assignに適用される。

char controler_directkey_bitmask[][3] = {
  { 0, CNTL_R1, 0 },
  { 0, CNTL_L1, 0 },
  { 0, CNTL_R2, 0 },
  { 0, CNTL_L2, 0 },
  { CNTL_R_ANLG, 0, 0 },
  { CNTL_L_ANLG, 0, 0 },
  { 0, 0, CNTL_MODE },
  { 0, 0, 0 }
};

char controler_exclusive_directkey_bitmask[][3] = {
  { 0, CNTL_L1, 0 },
  { 0, CNTL_R1, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { CNTL_L_ANLG, 0, 0 },
  { CNTL_R_ANLG, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

int controler_direct_command_assign[] = {
  DCMD_TURN_RIGHT,
  DCMD_TURN_LEFT,
  DCMD_NOP,
  DCMD_TRIGGER_ON,
  DCMD_NOP,
  DCMD_NOP,
  DCMD_SERVO_ON,
};

// ダイレクトキー登録ビットマスク
// ダイレクトキーとして登録すると、シフトキーとしては使えない。そのキーを押したことでコマンドが発動する。
// 同時動作が可能。　例：前進しながら射撃を行うなど
// ↓すべてのキーがダイレクトキーの場合の記述。最初の0は、そのレジスタでダイレクトキー登録がない場合の数値。
//int controler_directkey_accept[] = {
//		0 | ARRW_LEFT | ARRW_DOWN | ARRW_RIGHT | ARRW_UP | ARRW_START | ARRW_R_ANLG | ARRW_L_ANLG | ARRW_SELECT,
//		0 | CNTL_4 | CNTL_3 | CNTL_2 | CNTL_1 | CNTL_R1 | CNTL_L1 | CNTL_R2 | CNTL_L2,
//		0 | CNTL_MODE | CNTL_F2 | CNTL_F1
//};
*/
