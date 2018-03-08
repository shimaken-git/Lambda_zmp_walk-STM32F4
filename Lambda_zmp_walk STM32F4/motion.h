 /***********************************************/
/*                                             */
/*   File        :motion.h                     */
/*   Version     :v1.0.1                       */
/*   Date        :2014/01/05                   */
/*   Author      :Kenji Shimada                */
/*                                             */
/***********************************************/
#ifndef __MOTION_H__ 
#define __MOTION_H__

#include "lambda.h"

#define MOTION_VAL 100                 // ƒ‚[ƒVƒ‡ƒ“”
#define MOTION_DATA_VOL 500

typedef struct _motion_frame{
    short time;
    short dt[SV_VOL];
    FLOAT angle[2];  //0:axis X angle(pitch), 1:axis Y angle(roll)
    short sw[5];   // 0: right left eleveter straid on/off 1: arm only motoin 2: ik 3: 4:
} motionFrame;

typedef struct _motion_data{
	struct _motion_data *prev;
	struct _motion_data *next;
	motionFrame frame;
} motionData;

typedef struct _motion_id{
  char name[16];
  short start;
  short steps;
} motionId;

typedef struct _motion{
  struct _motion_data *head;
  struct _motion_data *tail;
  motionId id;
} Motion;

typedef struct _motion_info{
	int version;
	int motions;
	motionId table[MOTION_VAL];
} motionInfo;

void motion_ini(Motion *motion);
void motion_clear(Motion *motion);
motionData *motion_push_back(Motion *motion);
motionData *motion_push_front(Motion *motion);
motionData *motion_add(Motion *motion, motionData* mp);  // add frame behid mp
motionData *motion_del(Motion *motion, motionData* mp);  // del frame mp
void motion_pop_back(Motion *motion);
void motion_pop_front(Motion *motion);
int motion_size(Motion *motion);
void motion_chain(Motion *motion1, Motion *motion2);  //moiton1 = motion1 + motion2
void motion_copy(Motion *motion1, Motion *motion2); // motion1 -> motion2
void motion_info_ini(motionInfo *info);
motionId *motion_info_load_flash(motionInfo *info);
motionId *motion_save_flash(motionInfo *info, Motion *motion);

extern Motion motion;
extern motionInfo motion_info;
extern int motion_swid[];
extern int motion_edit_id;
extern int motion_play_id;
extern motionFrame *motion_frame;
extern int motion_info_load;
extern int motion_play_on;
extern int motion_play_time;
extern int motion_play_frame;
extern int motion_play_frame_count;
extern motionData *motion_play_data;
extern int motion_rest_frame;
extern FLOAT motion_bfr_angle[];
extern int motion_emergency_stop;
extern int motion_stability_check_cansel;

#endif
