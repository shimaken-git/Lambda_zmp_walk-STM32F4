//controler.c

#include <stdlib.h>
#include <string.h>
#include "controler.h"
#include "command.h"

void uart2_outval(long int val);
void uart2_outfloat(float val);
void uart2_puts(char *snd_data);
void uart2_putc(char c);
void out1h(char *txt, unsigned char value);
int out2h(char *txt, unsigned char value);
int out4h(char *txt, unsigned short value);
int bin2hex(int bn, char *bin, char *hex);
int outval(char *txt, int val);
int outfloat(char *txt, float val);
void add_eol(char *txt);

extern char msg[];

////////////////////////////////////////////////////////////////////
///////////////////////////////// Controler  ///////////////////////
////////////////////////////////////////////////////////////////////

// �L�[�o�^
// �L�[�Ƃ��Ĉ�����r�b�g�}�X�N
unsigned char controler_key_bitmask[3] = {
  0xff,
  0xff,
  0x0b
  };

ControlerConfig cc[] = {
  {  //[0]
//controler_directkey_bitmask_noshift[3] =
//�V�t�g�Ȃ���Ԃł̃_�C���N�g�L�[�̈ꗗ
    {
      0 | ARRW_LEFT | ARRW_DOWN | ARRW_RIGHT | ARRW_UP,
      0 | BTTN_R1 | BTTN_L1 | BTTN_R2 | BTTN_L2,
      0 | CNTL_MODE,
    },
//controler_directkey_bitmask_shift[3] =
//�V�t�g�����Ԃł̃_�C���N�g�L�[�̈ꗗ
    {
      0 | ARRW_LEFT | ARRW_DOWN | ARRW_RIGHT | ARRW_UP,
      0 | BTTN_R1 | BTTN_L1 | BTTN_R2 | BTTN_L2,
      0 | CNTL_MODE,
    },
//controler_shiftkey_bitmask_all[3] =
//�V�t�g�Ɏg�p����L�[�ꗗ
    {
      0 | ARRW_START | ARRW_SELECT,
      0 ,
      0 | CNTL_F2 | CNTL_F1,
    },
//controler_shiftkey_bitmask[20][3] =
//�V�t�g�p�^�[���ꗗ�@�V�t�g�Ɏg�p����L�[�̑g�ݍ��킹���L�q���Ă���
    {
      { 0, 0, 0 },				//shift pattern 0 (no shift)
      { ARRW_SELECT, 0, 0 },			//shift pattern 1
      { ARRW_START,  0, 0 },			//shift pattern 2
      { 0, 0, CNTL_F1 },			//shift pattern 3
      { 0, 0, CNTL_F2 },			//shift pattern 4
      { 0, 0, 0 }							//�G���h�}�[�N 
    },
//controler_commandkey_bitmask[10][3] =
//�R�}���h�L�[�ꗗ�@�o�^���ɃR�}���h�R�[�h�Y���������蓖�Ă���B
    {
      { 0, BTTN_1, 0 },			//controler_command_assign[shift pattern][0]
      { 0, BTTN_2, 0 },			//controler_command_assign[shift pattern][1]
      { 0, BTTN_3, 0 },			//controler_command_assign[shift pattern][2]
      { 0, BTTN_4, 0 },			//controler_command_assign[shift pattern][3]
      { 0, 0, 0 }			//�G���h�}�[�N
    },
//controler_command_assign[20][8] =
//�V�t�g�p�^�[���ƃR�}���h�R�[�h�̑g�ݍ��킹�Ɏ��s�R�}���h�����蓖�Ă�B
    {
//     command code��
      {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },// ��shiftpattern
      {CMD_BASEPOSE, CMD_LASER_SWITCH, CMD_SENSOR_SWITCH, CMD_DAMAGE_RESET, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },  // ARRW_SELECT
      {CMD_BASECHANGE, CMD_SPEED_CHANGE, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },  // ARRW_START
      {CMD_STANCE_HEIGHT, CMD_STANCE_WIDE, CMD_STANCE_LOW, CMD_STANCE_NARROW, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },  // CNTL_F1
      {CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, CMD_NOP, },// CNTL_F2
    },
//controler_directkey_bitmask[10][3] =
//�_�C���N�g�L�[�ꗗ�@�o�^���Ƀ_�C���N�g�R�}���h�R�[�h�����蓖�Ă���B
    {
      { ARRW_LEFT, 0, 0 },
      { ARRW_DOWN, 0, 0 },
      { ARRW_RIGHT, 0, 0 },
      { ARRW_UP, 0, 0 },
      { 0, BTTN_R1, 0 },
      { 0, BTTN_L1, 0 },
      { 0, BTTN_R2, 0 },
      { 0, BTTN_L2, 0 },
      { 0, 0, CNTL_MODE },
      { 0, 0, 0 }
    },
//controler_exclusive_directkey_bitmask[10][3] =
//�_�C���N�g�L�[�R�[�h�̔r���L�[�@�_�C���N�g�L�[��������Ă��鎞�ɉ�����Ă��Ă͂Ȃ�Ȃ��L�[�̑g�ݍ��킹
    {
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, BTTN_L1, 0 },
      { 0, BTTN_R1, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 }
    },
//controler_direct_command_assign[10] =
//�_�C���N�g�L�[�Ɏ��s�R�}���h�����蓖�Ă�B
    {
      DCMD_WALK_LEFT,
      DCMD_WALK_BACK,
      DCMD_WALK_RIGHT,
      DCMD_WALK_FOWORD,
      DCMD_TURN_RIGHT,
      DCMD_TURN_LEFT,
      DCMD_SPEED_UP,
      DCMD_TRIGGER_ON,
      DCMD_SERVO_ON,
      DCMD_NOP,
    }
  },
};

int controler_reg_list[] = {
  CSI_REG_ARROW,
  CSI_REG_BUTTON,
  CSI_REG_CNTL,
};

int controler_key_task(int md, volatile unsigned short *spi_data, volatile float *joy, volatile int *d_bit, volatile int *d_bit_exor, volatile int *scode, volatile int *ccode, volatile int *cnum){
  int d;
  int i, j;
  static int shift = 0;
  int c_num, s_num, d_num;
  static int d_bit_before = 0;
  unsigned char *direct_key_bitmask;
  unsigned char spi_char[3], spi_xor[3], spi_on[3], spi_off[3];
  static unsigned char spi_before[3];
  if(~(spi_data[CSI_REG_CNTL] >> 8) & CNTL_CNCT){
    // �A�i���O�X�e�B�b�N����
    for(i = 3; i < 7; i++){
      d = (unsigned char)(spi_data[i] >> 8);
      d = 0x80 - d;
      joy[i - 3] = d / 128.0;
    }
    for(i = 0; i < 3; i++){
      j = controler_reg_list[i];
      spi_char[i] = (unsigned char)(spi_data[j] >> 8);
      spi_xor[i] = spi_before[i] ^ spi_char[i];
      spi_on[i] = ~spi_char[i] & spi_xor[i];
      spi_off[i] = spi_char[i] & spi_xor[i];
      spi_before[i] = spi_char[i];
    }
    if(shift == 0){
      direct_key_bitmask = cc[md].controler_directkey_bitmask_noshift;
    }else{
      direct_key_bitmask = cc[md].controler_directkey_bitmask_shift;
    }
    // �R�}���h�L�[����
    c_num = 0;
    while(cc[md].controler_commandkey_bitmask[c_num][0] || cc[md].controler_commandkey_bitmask[c_num][1] || cc[md].controler_commandkey_bitmask[c_num][2]){
      if(!((spi_on[0] & ~direct_key_bitmask[0] & controler_key_bitmask[0]) ^ cc[md].controler_commandkey_bitmask[c_num][0]) &&  //�R�}���h�L�[���o�����F
         !((spi_on[1] & ~direct_key_bitmask[1] & controler_key_bitmask[1]) ^ cc[md].controler_commandkey_bitmask[c_num][1]) &&  //�@ON�^�C�~���O�Ō��o�i�V�t�g�L�[�͉������ςȂ��Ȃ̂ŏ��O���Č��o�j
         !((spi_on[2] & ~direct_key_bitmask[2] & controler_key_bitmask[2]) ^ cc[md].controler_commandkey_bitmask[c_num][2]) &&  //
         !((~spi_char[0] & ~direct_key_bitmask[0] & controler_key_bitmask[0]) ^ (cc[md].controler_commandkey_bitmask[c_num][0] | cc[md].controler_shiftkey_bitmask[shift][0])) &&  //�V�t�g���p
         !((~spi_char[1] & ~direct_key_bitmask[1] & controler_key_bitmask[1]) ^ (cc[md].controler_commandkey_bitmask[c_num][1] | cc[md].controler_shiftkey_bitmask[shift][1])) &&  //�@�V�t�g�L�[�����ɉ�����Ă��邱�Ƃ����o
         !((~spi_char[2] & ~direct_key_bitmask[2] & controler_key_bitmask[2]) ^ (cc[md].controler_commandkey_bitmask[c_num][2] | cc[md].controler_shiftkey_bitmask[shift][2])))    //
        break;
      c_num++;
    }
    if(!(cc[md].controler_commandkey_bitmask[c_num][0] || cc[md].controler_commandkey_bitmask[c_num][1] || cc[md].controler_commandkey_bitmask[c_num][2]))
      c_num = -1;
    *cnum = c_num;
    // �R�}���h�R�[�h�ݒ�
    if(c_num > -1)
      *ccode = cc[md].controler_command_assign[shift][c_num];
    else
      *ccode = 0;
    // �_�C���N�g�L�[����
    d_num = 0;
    *d_bit = 0;
    while(cc[md].controler_directkey_bitmask[d_num][0] || cc[md].controler_directkey_bitmask[d_num][1] || cc[md].controler_directkey_bitmask[d_num][2]){
      if(!((~spi_char[0] & direct_key_bitmask[0] & cc[md].controler_directkey_bitmask[d_num][0]) ^ cc[md].controler_directkey_bitmask[d_num][0]) &&  //������Ă���ׂ��L�[��������Ă��邩
         !((~spi_char[1] & direct_key_bitmask[1] & cc[md].controler_directkey_bitmask[d_num][1]) ^ cc[md].controler_directkey_bitmask[d_num][1]) &&  //
         !((~spi_char[2] & direct_key_bitmask[2] & cc[md].controler_directkey_bitmask[d_num][2]) ^ cc[md].controler_directkey_bitmask[d_num][2]) &&  //
         !((spi_char[0] & cc[md].controler_exclusive_directkey_bitmask[d_num][0]) ^ cc[md].controler_exclusive_directkey_bitmask[d_num][0]) &&  //������Ă��Ă͂Ȃ�Ȃ��L�[��������Ă��Ȃ���
         !((spi_char[1] & cc[md].controler_exclusive_directkey_bitmask[d_num][1]) ^ cc[md].controler_exclusive_directkey_bitmask[d_num][1]) &&  //
         !((spi_char[2] & cc[md].controler_exclusive_directkey_bitmask[d_num][2]) ^ cc[md].controler_exclusive_directkey_bitmask[d_num][2]))    //
        *d_bit |= cc[md].controler_direct_command_assign[d_num];
      d_num++;
    }
//    if(c_num == -1){
      // �V�t�g�L�[����
      // �V�t�g�L�[�����̓R�}���h�L�[��������Ă��Ȃ����̂ݎ��{�B�R�}���h�L�[���L���ȏꍇ�͑O��̃V�t�g�L�[�i���o�[���L���ƂȂ�B
      s_num = 1;
      while(cc[md].controler_shiftkey_bitmask[s_num][0] || cc[md].controler_shiftkey_bitmask[s_num][1] || cc[md].controler_shiftkey_bitmask[s_num][2]){
        if(!((~spi_char[0] & cc[md].controler_shiftkey_bitmask_all[0]) ^ cc[md].controler_shiftkey_bitmask[s_num][0]) &&  //�V�t�g�L�[���o
           !((~spi_char[1] & cc[md].controler_shiftkey_bitmask_all[1]) ^ cc[md].controler_shiftkey_bitmask[s_num][1]) &&  //�@�V�t�g�L�[�݂̂�������Ă��邩�����o
           !((~spi_char[2] & cc[md].controler_shiftkey_bitmask_all[2]) ^ cc[md].controler_shiftkey_bitmask[s_num][2]))    //
          break;
        s_num++;
      }
      if(!(cc[md].controler_shiftkey_bitmask[s_num][0] || cc[md].controler_shiftkey_bitmask[s_num][1] || cc[md].controler_shiftkey_bitmask[s_num][2])){
        shift = 0;
        *scode = 0;
      }else{
        *scode = s_num;
        if(c_num == -1) shift = s_num;
      }
//    }
    *d_bit_exor = *d_bit ^ d_bit_before;
    d_bit_before = *d_bit;
    return 1;
  }else{
    return 0;
  }
}

