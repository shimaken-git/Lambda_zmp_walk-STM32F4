//list.c

#include <stdlib.h>
#include "lambda.h"
#include "list.h"

//#define TEST
////////////////////////////////////////////////////////////////////
///////////////////////////////// List /////////////////////////////
////////////////////////////////////////////////////////////////////

void list_ini(List *list)
{
  list->head = NULL;
  list->tail = NULL;
}

void list_clear(List *list)
{
  Node *n = list->head;
  Node *nn;
  while(n){
    nn = n;
    n = n->next;
    free(nn);
  }
  list->head = NULL;
  list->tail = NULL;
}

Node *list_push_back(List *list)
{
  Node *n = (Node *)malloc(sizeof(Node));
  if(!n){
#ifndef TEST
    //uart1_puts( "malloc(sizeof(Node)) in list_push_back() = " );
    //uart1_out8h( n );
    //uart1_puts( "\n" );
#else
    printf("malloc(sizeof(Node)) in list_push_back() = %d\n", n);
#endif
    while(1);
  }
  if(list->tail){
    list->tail->next = n;
  }else{
    list->head = n;
  }
  n->prev = list->tail;
  n->next = NULL;
  list->tail = n;
  return n;

}

Node *list_push_front(List *list)
{
  Node *n = (Node *)malloc(sizeof(Node));
  if(!n){
#ifndef TEST
    //uart1_puts( "malloc(sizeof(Node)) in list_push_front() = " );
    //uart1_out8h( n );
    //uart1_puts( "\n" );
#else
    printf("malloc(sizeof(Node)) in list_push_front() = %d\n", n);
#endif
    while(1);
  }
  if(list->head){
    list->head->prev = n;
  }else{
    list->tail = n;
  }
  n->next = list->head;
  n->prev = NULL;
  list->head = n;
  return n;
}

void list_pop_back(List *list)
{
  Node *n;
  if(list->tail){
    n = list->tail;
    list->tail = list->tail->prev;
    if(!list->tail)
      list->head = list->tail;
    else
      list->tail->next = NULL;
    free(n);
  }
}

void list_pop_front(List *list)
{
  Node *n;
  if(list->head){
	n = list->head;
    list->head = list->head->next;
    if(!list->head)
      list->tail = list->head;
    else
      list->head->prev = NULL;
    free(n);
  }
}

int list_size(List *list)
{
  int n = 0;
  Node *nd = list->head;
  while(nd){
    n++;
    nd = nd->next;
  }
  return n;
}

void list_chain(List *list1, List *list2)  //list1 = list1 + list2
{
	if (!list1->head){
		list1->head = list2->head;
	}
	else{
		list1->tail->next = list2->head;
		list2->head->prev = list1->tail;
	}
	list1->tail = list2->tail;
	list2->head = NULL;
	list2->tail = NULL;
}

void list_copy(List *list1, List *list2) // list1 -> list2
{
	Node *nd1 = list1->head;
	Node *nd2;
	list_clear(list2);
	while(nd1){
		nd2 = list_push_back(list2);
		memcpy(nd2->d, nd1->d, sizeof(FLOAT[3]));
		memcpy(nd2->di, nd1->di, sizeof(short[2]));
		nd1 = nd1->next;
	}
}


#if 1
///////////////////////////////////////////////////////////////////
/////////////////////////////// List14 ////////////////////////////
///////////////////////////////////////////////////////////////////
void list14_ini(List14 *list)
{
  list->head = NULL;
  list->tail = NULL;
}

void list14_clear(List14 *list)
{
  Node14 *n = list->head;
  Node14 *nn;
  while(n){
    nn = n;
    n = n->next;
    free(nn);
  }
  list->head = NULL;
  list->tail = NULL;
}

Node14 *list14_push_back(List14 *list)
{
  Node14 *n = (Node14 *)malloc(sizeof(Node14));
  if(!n){
#ifndef TEST
    //uart1_puts( "malloc(sizeof(Node)) in list14_push_back() = " );
    //uart1_out8h( n );
    //uart1_puts( "\n" );
#else
    printf("malloc(sizeof(Node)) in list14_push_back() = %d\n", n);
#endif
    while(1);
  }
  if(list->tail){
    list->tail->next = n;
  }else{
    list->head = n;
  }
  n->prev = list->tail;
  n->next = NULL;
  list->tail = n;
  return n;
}

Node14 *list14_push_front(List14 *list)
{
  Node14 *n = (Node14 *)malloc(sizeof(Node14));
  if(!n){
#ifndef TEST
    //uart1_puts( "malloc(sizeof(Node)) in list14_push_front() = " );
    //uart1_out8h( n );
    //uart1_puts( "\n" );
#else
    printf("malloc(sizeof(Node)) in list14_push_front() = %d\n", n);
#endif
    while(1);
  }
  if(list->head){
    list->head->prev = n;
  }else{
    list->tail = n;
  }
  n->next = list->head;
  n->prev = NULL;
  list->head = n;
  return n;
}

void list14_pop_back(List14 *list)
{
  Node14 *n;
  if(list->tail){
    n = list->tail;
    list->tail = list->tail->prev;
    if(!list->tail)
      list->head = list->tail;
    else
      list->tail->next = NULL;
    free(n);
  }
}

void list14_pop_front(List14 *list)
{
  Node14 *n;
  if(list->head){
    n = list->head;
    list->head = list->head->next;
    if(!list->head)
      list->tail = list->head;
    else
      list->head->prev = NULL;
    free(n);
  }
}

int list14_size(List14 *list)
{
  int n = 0;
  Node14 *nd = list->head;
  while(nd){
    n++;
    nd = nd->next;
  }
  return n;
}

void list14_replace(List14 *list, List14 *list2, Node14 *node)
{
  if(!node)
    return;
  if(node->prev){  //node‚Ílist‚Ìæ“ª‚¶‚á‚È‚¢
    node->prev->next = list2->head;
    list2->head->prev = node->prev;
  }else{ //æ“ª‚Ìê‡
    list->head = list2->head;
  }
  Node14 *n = node, *nn;
  while(n){
    nn = n;
    n = nn->next;
    free(nn);
  }
  list->tail = list2->tail;
  list2->head = NULL;
  list2->tail = NULL;
}
#endif //list14

////////////////////////////////////////////////////////////////////
//////////////////////////// ListS ///////////////////////////////
////////////////////////////////////////////////////////////////////

void listS_ini(ListS *list)
{
  list->head = NULL;
  list->tail = NULL;
}

void listS_clear(ListS *list)
{
  NodeS *n = list->head;
  NodeS *nn;
  while(n){
    nn = n;
    n = n->next;
    free(nn);
  }
  list->head = NULL;
  list->tail = NULL;
}

NodeS *listS_push_back(ListS *list)
{
  NodeS *n = (NodeS *)malloc(sizeof(NodeS));
  if(!n){
#ifndef TEST
    //uart1_puts( "malloc(sizeof(Node)) in listS_push_back() = " );
    //uart1_out8h( n );
    //uart1_puts( "\n" );
#else
    printf("malloc(sizeof(Node)) in listS_push_back() = %d\n", n);
#endif
    while(1);
  }
  if(list->tail){
    list->tail->next = n;
  }else{
    list->head = n;
  }
  n->prev = list->tail;
  n->next = NULL;
  list->tail = n;
  return n;
}

NodeS *listS_push_front(ListS *list)
{
  NodeS *n = (NodeS *)malloc(sizeof(NodeS));
  if(!n){
#ifndef TEST
    //uart1_puts( "malloc(sizeof(Node)) in listS_push_front() = " );
    //uart1_out8h( n );
    //uart1_puts( "\n" );
#else
    printf("malloc(sizeof(Node)) in listS_push_front() = %d\n", n);
#endif
    while(1);
  }
  if(list->head){
    list->head->prev = n;
  }else{
    list->tail = n;
  }
  n->next = list->head;
  n->prev = NULL;
  list->head = n;
  return n;
}

void listS_pop_back(ListS *list)
{
  NodeS *n;
  if(list->tail){
    n = list->tail;
    list->tail = list->tail->prev;
    if(!list->tail)
      list->head = list->tail;
    else
      list->tail->next = NULL;
    free(n);
  }
}

void listS_pop_front(ListS *list)
{
  NodeS *n;
  if(list->head){
    list->head = list->head->next;
    if(!list->head)
      list->tail = list->head;
    else
      list->head->prev = NULL;
    free(n);
  }
}

int listS_size(ListS *list)
{
  int n = 0;
  NodeS *nd = list->head;
  while(nd){
    n++;
    nd = nd->next;
  }
  return n;
}
