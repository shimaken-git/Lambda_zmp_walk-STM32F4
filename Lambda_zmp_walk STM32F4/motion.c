/*************************************************/
/*                                               */
/*   File        :motion.c                       */
/*   Version     :v1.0.1                         */
/*   Date        :2014/01/05                     */
/*   Author      :Kenji Shimada                  */
/*                                               */
/*************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stm32f4xx_flash.h>

#include "lambda.h"
#include "vector.h"
#include "motion.h"

Motion motion;
motionInfo motion_info;
int motion_swid[] = {2,3,10,11};
motionFrame *motion_frame;
int motion_info_load = 0; //load flag
int motion_play_on = 0;
int motion_play_time = 0;
int motion_play_frame;
int motion_play_frame_count;
motionData *motion_play_data;

int motion_edit_id = -1;
int motion_play_id = -1;

int motion_rest_frame = 0;
FLOAT motion_bfr_angle[2];
int motion_stability_check_cansel = 0;
int motion_emergency_stop = 0;

char s[100];

void motion_ini(Motion *motion)
{
  motion->head = NULL;
  motion->tail = NULL;
  motion->id.name[0] = 0;
}

void motion_clear(Motion *motion)
{
  motionData *n = motion->head;
  motionData *nn;
  while(n){
    nn = n;
    n = n->next;
    free(nn);
  }
  motion_ini(motion);
}

motionData *motion_push_back(Motion *motion)
{
  motionData *n = (motionData *)malloc(sizeof(motionData));
  if(!n){
#ifndef TEST
	sprintf(s,"malloc(sizeof(motionData)) in motion_push_back() = %d\n", n);
	tmcom_puts(s);
#else
    printf("malloc(sizeof(motionData)) in motion_push_back() = %d\n", n);
#endif
    while(1);
  }
  if(motion->tail){
    motion->tail->next = n;
  }else{
    motion->head = n;
  }
  n->prev = motion->tail;
  n->next = NULL;
  motion->tail = n;
  return n;
}

motionData *motion_push_front(Motion *motion)
{
  motionData *n = (motionData *)malloc(sizeof(motionData));
  if(!n){
#ifndef TEST
		sprintf(s,"malloc(sizeof(motionData)) in motion_push_front() = %d\n", n);
		tmcom_puts(s);
#else
    printf("malloc(sizeof(motionData)) in motion_push_front() = %d\n", n);
#endif
    while(1);
  }
  if(motion->head){
    motion->head->prev = n;
  }else{
    motion->tail = n;
  }
  n->next = motion->head;
  n->prev = NULL;
  motion->head = n;
  return n;
}

motionData *motion_add(Motion *motion, motionData* mp)
{
  motionData *n = motion->head;
  while(n != mp && n){
    n = n->next;
  }
  if(!n){
    return NULL;
  }else{
    n = (motionData *)malloc(sizeof(motionData));
    if(!n){
#ifndef TEST
      sprintf(s,"malloc(sizeof(motionData)) in motion_add() = %d\n", n);
      tmcom_puts(s);
#else
      printf("malloc(sizeof(motionData)) in motion_add() = %d\n", n);
#endif
      while(1);
    }else{
      n->prev = mp;
      n->next = mp->next;
      mp->next = n;
      if(n->next){
        n->next->prev = n;
      }else{
        motion->tail = n;
      }
    }
    return n;
  }
}

motionData *motion_del(Motion *motion, motionData* mp)
{
  motionData *n = motion->head;
  while(n != mp && n){
    n = n->next;
  }
  if(!n){
    return NULL;
  }else{
    if(mp->next){
      mp->next->prev = mp->prev;
    }else{
      motion->tail = mp->prev;
    }
    if(mp->prev){
      mp->prev->next = mp->next;
    }else{
      motion->head = mp->next;
    }
    n = mp->next;
    if(!n){
    	n = motion->tail;
    }
    free(mp);
    return n;
  }
}

void motion_pop_back(Motion *motion)
{
  motionData *n;
  if(motion->tail){
    n = motion->tail;
    motion->tail = motion->tail->prev;
    if(!motion->tail)
      motion->head = motion->tail;
    else
      motion->tail->next = NULL;
    free(n);
  }
}

void motion_pop_front(Motion *motion)
{
  motionData *n;
  if(motion->head){
    motion->head = motion->head->next;
    if(!motion->head)
      motion->tail = motion->head;
    else
      motion->head->prev = NULL;
    free(n);
  }
}

int motion_size(Motion *motion)
{
  int n = 0;
  motionData *nd = motion->head;
  while(nd){
    n++;
    nd = nd->next;
  }
  return n;
}

void motion_chain(Motion *motion1, Motion *motion2)  //motion1 = motion1 + motion2
{
	if (!motion1->head){
		motion1->head = motion2->head;
	}
	else{
		motion1->tail->next = motion2->head;
		motion2->head->prev = motion1->tail;
	}
	motion1->tail = motion2->tail;
	motion2->head = NULL;
	motion2->tail = NULL;
}

void motion_copy(Motion *motion1, Motion *motion2) // motion1 -> motion2
{
	motionData *nd1 = motion1->head;
	motionData *nd2;
	motion_clear(motion2);
	while(nd1){
		nd2 = motion_push_back(motion2);
		memcpy(nd2->frame.dt, nd1->frame.dt, sizeof(short[SV_VOL]));
		memcpy(nd2->frame.angle, nd1->frame.angle, sizeof(FLOAT[2]));
		memcpy(nd2->frame.sw, nd1->frame.sw, sizeof(short[5]));
		nd2->frame.time = nd1->frame.time;
		nd1 = nd1->next;
	}
}
void motion_info_ini(motionInfo *info)
{
	info->motions = -1;
}

motionId *motion_info_load_flash(motionInfo *info)
{
  uint32_t flashaddr;
  motionInfo *info_1, *info_2;

  info_1 = (motionInfo *)0x080A0000;  //FLASH_Sector_9
  info_2 = (motionInfo *)0x080C0000;  //FLASH_Sector_10
  if(info_1->version < info_2->version){
    info_1 = info_2; // bigger vertion
  }
  sprintf(s,"read sector=>%x version:%d\r\n", info_1, info_1->version);
  tmcom_puts(s);
  memcpy(info, info_1, sizeof(motionInfo));
  return (motionId *)((uint32_t)info_1 + sizeof(motionInfo));
}

motionId *motion_save_flash(motionInfo *info, Motion *motion)
{
  uint32_t sector;
  uint32_t flashaddr;
  motionInfo *info_1, *info_2;
  volatile FLASH_Status FLASHStatus;
  uint32_t addr;
  uint32_t vsize;
  uint32_t vtop,vpos;
  motionFrame *motion_tbl;
  motionData *motion_d;
  uint32_t motion_id;

  info_1 = (motionInfo *)0x080A0000;  //FLASH_Sector_9
  info_2 = (motionInfo *)0x080C0000;  //FLASH_Sector_10
  vsize = sizeof(motionInfo);
  if(info_1->version < info_2->version){ // lower version
    sector = FLASH_Sector_9;
    flashaddr = (uint32_t)info_1;
    motion_tbl = (motionFrame *)((uint32_t)info_2 + vsize);
    info->version = info_2->version + 1;
  }else{
    sector = FLASH_Sector_10;
    flashaddr = (uint32_t)info_2;
    motion_tbl = (motionFrame *)((uint32_t)info_1 + vsize);
    info->version = info_1->version + 1;
  }
  motion_id = flashaddr;
  sprintf(s,"write sector=>%x version:%d\r\n", flashaddr, info->version);
  tmcom_puts(s);

  //条件としてmotionは空ではないとする。

  // section1
  int sec1_st = 0;
  int sec1_stp = 0;
  int sec2_st;
  int sec2_stp;
  int sec3_read_st;
  int sec3_write_st;
  int sec3_stp = 0;

  //新規モーションにID付与
  if(motion_edit_id < 0){
	  motion_edit_id = info->motions;
  }

  int id = 0;
  while(id < motion_edit_id){
    sec1_stp += info->table[id].steps;
    id++;
  }
  sec2_st = sec1_stp;
  sec2_stp = motion_size(motion);
  sec3_write_st = sec2_st + sec2_stp;
  if(info->motions > motion_edit_id+1 && motion_edit_id > -1){
    sec3_read_st = info->table[motion_edit_id].start + info->table[motion_edit_id].steps;
  }else{
    sec3_read_st = -1;
  }
  if(sec3_read_st >= 0){
	  id = motion_edit_id + 1;
	  while(id < info->motions){
	    sec3_stp += info->table[id].steps;
	    id++;
	  }
  }
  sprintf(s, "1_step:%d 2_start:%d 2_step:%d 3_read_start:%d 3_write_start:%d 3_step:%d\r\n", sec1_stp, sec2_st, sec2_stp, sec3_read_st, sec3_write_st, sec3_stp);
  tmcom_puts(s);
  
  if(info->motions == motion_edit_id){
	  info->motions++;
  }
  //テーブルの更新　テーブルの更新は最後
  strcpy(info->table[motion_edit_id].name, motion->id.name);
  info->table[motion_edit_id].start = sec2_st;
  info->table[motion_edit_id].steps = sec2_stp;
  for(int i = motion_edit_id + 1; i < info->motions; i++){
	  info->table[i].start = info->table[i-1].start + info->table[i-1].steps;
	  sprintf(s, "talble[%d].start = %d steps = %d\r\n", i, info->table[i].start, info->table[i].steps);
	  tmcom_puts(s);
  }

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
  if(FLASH_EraseSector(sector, VoltageRange_3) != FLASH_COMPLETE){  //erase flash
	  tmcom_puts("flash erase err.\r\n");
  }else{
	  tmcom_puts("flash erase success.\r\n");
      //motion_info write to flash
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(motionInfo);
	  sprintf(s, "motionInfo : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)info;
	  while((vpos < vtop + vsize) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
		  vpos += 2;
	  }
	  sprintf(s, "sizeof(motionFrame) = %d\r\n", sizeof(motionFrame));
	  tmcom_puts(s);

      //motion frame write to flash
	  //sec1
	  flashaddr += vsize;
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  vsize = sizeof(motionFrame) * sec1_stp;
	  sprintf(s, "motion save section1 data size : %ld bytes\r\n", vsize);
	  tmcom_puts(s);
	  vtop = vpos = (uint32_t)motion_tbl;
	  while((vpos < vtop + vsize) && (FLASHStatus == FLASH_COMPLETE)){
		  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
		  addr += 2;
	      vpos += 2;
	  }

	  //sec2
      flashaddr += vsize;
	  addr = flashaddr;
	  FLASHStatus = FLASH_COMPLETE;
	  sprintf(s, "motion save section2 data size : %ld bytes\r\n", sizeof(motionFrame) * sec2_stp);
	  tmcom_puts(s);
	  motion_d = motion->head;
	  //for(int i = 0; i < sec2_stp; i++){
	  int sec2_frame = 0;
	  while(motion_d){
		  vsize = sizeof(motionFrame);
		  vtop = vpos = (uint32_t)(&motion_d->frame);
		  while((vpos < vtop + vsize) && (FLASHStatus == FLASH_COMPLETE)){
			  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
			  addr += 2;
			  vpos += 2;
		  }
		  motion_d = motion_d->next;
		  sprintf(s, "sec2 : %d write\r\n", sec2_frame++);
		  tmcom_puts(s);
	  }
	  //sec3
	  if(sec3_read_st >= 0){
		  flashaddr += sizeof(motionFrame) * sec2_stp;
		  addr = flashaddr;
		  FLASHStatus = FLASH_COMPLETE;
		  vsize = sizeof(motionFrame) * sec3_stp;
		  sprintf(s, "motion save section3 data size : %ld bytes\r\n", vsize);
		  tmcom_puts(s);
		  sprintf(s, "sec3_read_st=%d &motion_tble[sec3_read_st]=%lx write addr=%lx\r\n", sec3_read_st, (uint32_t)(&motion_tbl[sec3_read_st]), addr);
		  tmcom_puts(s);
		  vtop = vpos = (uint32_t)(&motion_tbl[sec3_read_st]);
		  while((vpos < vtop + vsize) && (FLASHStatus == FLASH_COMPLETE)){
			  FLASHStatus = FLASH_ProgramHalfWord(addr, *(uint16_t *)vpos);
			  addr += 2;
		      vpos += 2;
		  }
	  }else{
		  tmcom_puts("sec3 no data\r\n");
	  }
	  tmcom_puts("flash write complete.\r\n");
  }
  FLASH_Lock();
  return (motionId *)(motion_id + sizeof(motionInfo));
}
