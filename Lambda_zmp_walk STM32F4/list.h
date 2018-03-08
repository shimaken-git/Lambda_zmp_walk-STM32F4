#ifndef __LIST_H__ 
#define __LIST_H__
#include <stdio.h>
#include "lambda.h"

typedef struct _node{
  FLOAT d[3];
  short di[2];
//  FLOAT dummy[3];
  struct _node *next;
  struct _node *prev;
} Node;

typedef struct _list{
  struct _node *head;
  struct _node *tail;
} List;

void list_ini(List *list);
void list_clear(List *list);
Node *list_push_back(List *list);
Node *list_push_front(List *list);
void list_pop_back(List *list);
void list_pop_front(List *list);
int list_size(List *list);
void list_chain(List *list1, List *list2);  //list1 = list1 + list2
void list_copy(List *list1, List *list2); // list1 -> list2

#if 1
typedef struct _node14{
  FLOAT d[16];
  FLOAT dummy[3];
  struct _node14 *next;
  struct _node14 *prev;
} Node14;

typedef struct _list14{
  struct _node14 *head;
  struct _node14 *tail;
} List14;

void list14_ini(List14 *list);
void list14_clear(List14 *list);
Node14 *list14_push_back(List14 *list);
Node14 *list14_push_front(List14 *list);
void list14_pop_back(List14 *list);
void list14_pop_front(List14 *list);
int list14_size(List14 *list);
void list14_replace(List14 *list, List14 *list2, Node14 *node);  //listのnode以降をlist2のﾒﾝﾊﾞｰに置き換える（nodeは置き換え対象）
#endif

typedef struct _nodeS{
  short d[46];
  struct _nodeS *next;
  struct _nodeS *prev;
} NodeS;

typedef struct _listS{
  struct _nodeS *head;
  struct _nodeS *tail;
} ListS;

void listS_ini(ListS *list);
void listS_clear(ListS *list);
NodeS *listS_push_back(ListS *list);
NodeS *listS_push_front(ListS *list);
void listS_pop_back(ListS *list);
void listS_pop_front(ListS *list);
int listS_size(ListS *list);


#endif
