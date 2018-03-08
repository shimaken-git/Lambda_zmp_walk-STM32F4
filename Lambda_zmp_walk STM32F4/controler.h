#ifndef __CONTROLER_H__ 
#define __CONTROLER_H__

// csi ���W�X�^�A�h���X
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
// �@�_�C���N�g�L�[�}�b�v�̎w��
// �@�_�C���N�g�L�[�������ꂽ��ԂŁA�R�}���h�L�[���������Ƃ��ł���B
// �@�v���[����ԂƃV�t�g��ԂŁA�_�C���N�g�L�[�}�b�v��ς��邱�Ƃ��ł���B
//   �_�C���N�g�L�[����������ԂŁA�V�t�g�L�[�����o�ł��邱�Ƃ͂ł��邪�A
//   �V�t�g�L�[������ԂŃ_�C���N�g�L�[�ƂȂ��Ă��Ȃ��L�[�͉�����Ԃ�ۑ��ł��Ȃ��B

  
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

// �A�V�t�g�L�[�o�^
// �@�V�t�g�t���R�}���h�́A�����L�[�������Ƀr�b�g���������ł͂Ȃ��A���̑O�ɃV�t�g�L�[�������ꂽ�o�܂��K�v�B
// �@�uS�v����́uSC�vS:���V�t�g�L�[�AC���R�}���h�L�[�Ƃ���B
// �@�uS�v�̃X�e�b�v�Ȃ��ɁuSC�v�������Ă����߁B����́A�uC�v��Ԃ���uSC�v�ɂȂ����ꍇ�𖳌��Ƃ��邽�߁B
//   �V�t�g��Ԃ̓R�}���h�Ɋ֌W�Ȃ����o����B
  
  �V�t�g�p�^�[�����ׂĂ�AND�������́B
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
//       ��command
//       ��shift
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

// �B�_�C���N�g�L�[�}�X�N
// �@�_�C���N�g�L�[�͓����ɕ����L�[��������Ă��L���ƂȂ�ꍇ���������A
// �@���E�L�[�ȂǁA�����ɉ����ꂽ�ꍇ�ɂ͖����Ƃ���K�v�����邽�߁A�r������������B
// �@controler_directkey_bitmask[1][]�̏ꍇ�Acontroler_exclusive_directkey_bitmask[1][]�ƃZ�b�g�Ŏg���A
// �@CNTL_R1�������Ă���Ƃ��ACNTL_L1�͗����Ă��Ă͂Ȃ�Ȃ��B���̑��̃r�b�g�͕]���ΏۊO�ƂȂ�B
//   ��register
//   ��direct key list  �����̏��Ԃ�command assign�ɓK�p�����B

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

// �_�C���N�g�L�[�o�^�r�b�g�}�X�N
// �_�C���N�g�L�[�Ƃ��ēo�^����ƁA�V�t�g�L�[�Ƃ��Ă͎g���Ȃ��B���̃L�[�����������ƂŃR�}���h����������B
// �������삪�\�B�@��F�O�i���Ȃ���ˌ����s���Ȃ�
// �����ׂẴL�[���_�C���N�g�L�[�̏ꍇ�̋L�q�B�ŏ���0�́A���̃��W�X�^�Ń_�C���N�g�L�[�o�^���Ȃ��ꍇ�̐��l�B
//int controler_directkey_accept[] = {
//		0 | ARRW_LEFT | ARRW_DOWN | ARRW_RIGHT | ARRW_UP | ARRW_START | ARRW_R_ANLG | ARRW_L_ANLG | ARRW_SELECT,
//		0 | CNTL_4 | CNTL_3 | CNTL_2 | CNTL_1 | CNTL_R1 | CNTL_L1 | CNTL_R2 | CNTL_L2,
//		0 | CNTL_MODE | CNTL_F2 | CNTL_F1
//};
*/
