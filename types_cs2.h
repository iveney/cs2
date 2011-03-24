/* defs.h */

typedef long long int excess_t;
typedef long long int price_t;

#define MAX_64 (0x7fffffffffffffffLL)
#define MAX_32 (0x7fffffff)
#define PRICE_MAX MAX_64

typedef  /* arc */
   struct arc_st
{
   long             r_cap;           /* residual capasity */
   price_t          cost;            /* cost  of the arc*/
   struct node_st   *head;           /* head node */
   struct arc_st    *sister;         /* opposite arc */
}
  arc;

typedef  /* node */
   struct node_st
{
   arc              *first;           /* first outgoing arc */
   arc              *current;         /* current outgoing arc */
   arc              *suspended;
   excess_t         excess;           /* excess of the node */
   price_t          price;            /* distance from a sink */
   struct node_st   *q_next;          /* next node in push queue */
   struct node_st   *b_next;          /* next node in bucket-list */
   struct node_st   *b_prev;          /* previous node in bucket-list */
   long             rank;             /* bucket number */
   long             inp;              /* auxilary field */
} node;

typedef /* bucket */
   struct bucket_st
{
   node             *p_first;         /* 1st node with positive excess 
				         or simply 1st node in the buket */
} bucket;
