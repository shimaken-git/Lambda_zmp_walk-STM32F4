//controler.c

#include <stdlib.h>
#include <string.h>
#include "controler.h"
#include "command.h"

////////////////////////////////////////////////////////////////////
///////////////////////////////// Controler  ///////////////////////
////////////////////////////////////////////////////////////////////

char *cm_str[] = {
  "servoenable",
  "servomonon",
  "servoon",
  "servooff",
  "servozero",
  "servoget",
  "svget",
  "servo",
  "stretch",
  "speed",
  "rom",
  "ik",
  "dk",
  "lbp",
  "rpy",
  "getgp",
  "gpset",
  "offset",
  "ubp",
  "ubpset",
  "test",
  "testprm",
#ifdef WALK
  "wps",
  "calcwalk",
  "walk",
#endif
  "zmp",
  "cont",
  "c",
  "stop",
  "s",
  "walklog",
//  "bodyrolladj",
  "parami",
  "paramf",
  "joint",
  "trim",
  "linktrimset",
  "legpressure",       // �Z���T�[�f�[�^�̈��\���B�ŗL�f�[�^���\������B
  "legpressurerow",    // �����Z���T�[�̃L�����u���[�V�������s���B���s����ƌŗL�f�[�^�����Z�b�g�����B
  "gyroread",
  "sensordisp",
  "verbosesensor",
  "vs",
  "sensorpace",
  "sp",
  "gyrofeedback",
  "zmpfeedback",
  "calibration",
  "e",
  "poscalibration",
  "comcalibration",
  "zmpalign",
  "xzmpcalibration",
  "f",
  "b",
  "motion",
  "m",
  "ms",  //motion stop
  "up",
  "a",
  "stability",
  "battery",
  "batterylimit",
  "saveflash",
  "loadflash",
  "loadparam",
  "unloadflash",
  "flashdisp",
  "information",
  "help",
  "prmmon",  // switch
  "spimon",
  "controlermon",
  "statusmon",
  "hidan",
  ""
};

char *cm_help[] = {
		  "servoenable",
		  "servomonon",
		  "servoon",
		  "servooff",
		  "servozero",
		  "servoget",
		  "svget",
		  "servo",
		  "stretch",
		  "speed",
		  "rom",
		  "ik",
		  "dk",
		  "lbp",
		  "rpy",
		  "getgp",
		  "gpset",
		  "offset",
		  "ubp",
		  "ubpset",
		  "test",
		  "testprm",
		#ifdef WALK
		  "wps",
		  "calcwalk",
		  "walk",
		#endif
		  "zmp",
		  "cont",
		  "c",
		  "stop",
		  "s",
		  "walklog",
		//  "bodyrolladj",
		  "parami",
		  "paramf",
		  "joint",
		  "trim",
		  "linktrimset",
		  "legpressure",       // �Z���T�[�f�[�^�̈��\���B�ŗL�f�[�^���\������B
		  "legpressurerow",    // �����Z���T�[�̃L�����u���[�V�������s���B���s����ƌŗL�f�[�^�����Z�b�g�����B
		  "gyroread",
		  "sensordisp",
		  "verbosesensor",
		  "vs",
		  "sensorpace",
		  "sp",
		  "gyrofeedback",
		  "zmpfeedback",
		  "calibration",
		  "e",
		  "poscalibration",
		  "comcalibration",
		  "zmpalign",
		  "xzmpcalibration",
		  "f",
		  "b",
		  "motion",
		  "m",
		  "ms",
		  "up",
		  "a",
		  "stability",
		  "battery",
		  "batterylimit",
		  "saveflash",
		  "loadflash",
		  "loadparam",
		  "unloadflash",
		  "flashdisp",
		  "information",
		  "help",
		  "prmmon",  // switch
		  "spimon",
		  "controlermon",
		  "statusmon",
		  "hidan",
		  ""
};

int sw_status[SW_NUM];