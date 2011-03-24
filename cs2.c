/* min-cost flow */
/* successive approximation algorithm */
/* Copyright C IG Systems, igsys@eclipse.com */
/* any use except for evaluation purposes requires a licence */

/************************************** constants  &  parameters ********/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
//#include <values.h>

/* for measuring time */

/* definitions of types: node & arc */

#include "types_cs2.h"

/* function 'timer()' for measuring processor time */

#ifdef WINDOWS_TIMER
#include "winTimer.c"
#else
#include "timer.c"
#endif

#define N_NODE( i ) ( ( (i) == NULL ) ? -1 : ( (i) - ndp + nmin ) )
#define N_ARC( a ) ( ( (a) == NULL )? -1 : (a) - arp )

#define UNFEASIBLE          2
#define ALLOCATION_FAULT    5
#define PRICE_OFL           6

/* parameters */

#define UPDT_FREQ      0.4
#define UPDT_FREQ_S    30

#define SCALE_DEFAULT  12.0

/* PRICE_OUT_START may not be less than 1 */
#define PRICE_OUT_START 1

#define CUT_OFF_POWER    0.44
#define CUT_OFF_COEF     1.5
#define CUT_OFF_POWER2   0.75
#define CUT_OFF_COEF2    1
#define CUT_OFF_GAP      0.8
#define CUT_OFF_MIN      12
#define CUT_OFF_INCREASE 4

/*
#define TIME_FOR_PRICE_IN    5
*/
#define TIME_FOR_PRICE_IN1    2
#define TIME_FOR_PRICE_IN2    4
#define TIME_FOR_PRICE_IN3    6

//#define EMPTY_PUSH_COEF      1.0
/*
#define MAX_CYCLES_CANCELLED 10
#define START_CYCLE_CANCEL   3
*/
#define MAX_CYCLES_CANCELLED 0
#define START_CYCLE_CANCEL   100
/************************************************ shared macros *******/

#define MAX( x, y ) ( ( (x) > (y) ) ?  x : y )
#define MIN( x, y ) ( ( (x) < (y) ) ? x : y )

#define OPEN( a )   ( a -> r_cap > 0 )
#define CLOSED( a )   ( a -> r_cap <= 0 )
#define REDUCED_COST( i, j, a ) ( (i->price) + (a->cost) - (j->price) )
#define FEASIBLE( i, j, a )     ( (i->price) + (a->cost) < (j->price) )
#define ADMISSIBLE( i, j, a )   ( OPEN(a) && FEASIBLE( i, j, a ) )


#define INCREASE_FLOW( i, j, a, df )\
{\
   (i) -> excess            -= df;\
   (j) -> excess            += df;\
   (a)            -> r_cap  -= df;\
  ((a) -> sister) -> r_cap  += df;\
}\

/*---------------------------------- macros for excess queue */

#define RESET_EXCESS_Q \
{\
   for ( ; excq_first != NULL; excq_first = excq_last )\
     {\
	excq_last            = excq_first -> q_next;\
        excq_first -> q_next = sentinel_node;\
     }\
}

#define OUT_OF_EXCESS_Q( i )  ( i -> q_next == sentinel_node )

#define EMPTY_EXCESS_Q    ( excq_first == NULL )
#define NONEMPTY_EXCESS_Q ( excq_first != NULL )

#define INSERT_TO_EXCESS_Q( i )\
{\
   if ( NONEMPTY_EXCESS_Q )\
     excq_last -> q_next = i;\
   else\
     excq_first  = i;\
\
   i -> q_next = NULL;\
   excq_last   = i;\
}

#define INSERT_TO_FRONT_EXCESS_Q( i )\
{\
   if ( EMPTY_EXCESS_Q )\
     excq_last = i;\
\
   i -> q_next = excq_first;\
   excq_first  = i;\
}

#define REMOVE_FROM_EXCESS_Q( i )\
{\
   i           = excq_first;\
   excq_first  = i -> q_next;\
   i -> q_next = sentinel_node;\
}

/*---------------------------------- excess queue as a stack */

#define EMPTY_STACKQ      EMPTY_EXCESS_Q
#define NONEMPTY_STACKQ   NONEMPTY_EXCESS_Q

#define RESET_STACKQ  RESET_EXCESS_Q

#define STACKQ_PUSH( i )\
{\
   i -> q_next = excq_first;\
   excq_first  = i;\
}

#define STACKQ_POP( i ) REMOVE_FROM_EXCESS_Q( i )

/*------------------------------------ macros for buckets */

node dnd, *dnode;

#define RESET_BUCKET( b )  ( b -> p_first ) = dnode;

#define INSERT_TO_BUCKET( i, b )\
{\
i -> b_next                  = ( b -> p_first );\
( b -> p_first ) -> b_prev   = i;\
( b -> p_first )             = i;\
}

#define NONEMPTY_BUCKET( b ) ( ( b -> p_first ) != dnode )

#define GET_FROM_BUCKET( i, b )\
{\
i    = ( b -> p_first );\
( b -> p_first ) = i -> b_next;\
}

#define REMOVE_FROM_BUCKET( i, b )\
{\
if ( i == ( b -> p_first ) )\
       ( b -> p_first ) = i -> b_next;\
  else\
    {\
       ( i -> b_prev ) -> b_next = i -> b_next;\
       ( i -> b_next ) -> b_prev = i -> b_prev;\
    }\
}

/*------------------------------------------- misc macros */

#define UPDATE_CUT_OFF \
{\
   if (n_bad_pricein + n_bad_relabel == 0) \
     {\
	cut_off_factor = CUT_OFF_COEF2 * pow ( (double)n, CUT_OFF_POWER2 );\
        cut_off_factor = MAX ( cut_off_factor, CUT_OFF_MIN );\
        cut_off        = cut_off_factor * epsilon;\
        cut_on         = cut_off * CUT_OFF_GAP;\
      }\
     else\
       {\
	cut_off_factor *= CUT_OFF_INCREASE;\
        cut_off        = cut_off_factor * epsilon;\
        cut_on         = cut_off * CUT_OFF_GAP;\
	}\
}

#define TIME_FOR_UPDATE \
( n_rel > n * UPDT_FREQ + n_src * UPDT_FREQ_S )

#define FOR_ALL_NODES_i        for ( i = nodes; i != sentinel_node; i ++ )

#define FOR_CURRENT_ARCS_a_FROM_i \
for ( a = i -> current, a_stop = ( i + 1 ) -> suspended; a != a_stop; a ++ )

#define FOR_ALL_ARCS_a_FROM_i \
for ( a = i -> suspended, a_stop = ( i + 1 ) -> suspended; a != a_stop; a ++ )

#define FOR_ACTIVE_ARCS_a_FROM_i \
for ( a = i -> first, a_stop = ( i + 1 ) -> suspended; a != a_stop; a ++ )

#define WHITE 0
#define GREY  1
#define BLACK 2

arc     *sa, *sb;
long    d_cap;

#define EXCHANGE( a, b )\
{\
if ( a != b )\
  {\
     sa = a -> sister;\
     sb = b -> sister;\
\
     d_arc.r_cap = a -> r_cap;\
     d_arc.cost  = a -> cost;\
     d_arc.head  = a -> head;\
\
     a -> r_cap  = b -> r_cap;\
     a -> cost   = b -> cost;\
     a -> head   = b -> head;\
\
     b -> r_cap  = d_arc.r_cap;\
     b -> cost   = d_arc.cost;\
     b -> head   = d_arc.head;\
\
     if ( a != sb )\
       {\
	  b -> sister = sa;\
	  a -> sister = sb;\
	  sa -> sister = b;\
	  sb -> sister = a;\
        }\
\
     d_cap       = cap[a-arcs];\
     cap[a-arcs] = cap[b-arcs];\
     cap[b-arcs] = d_cap;\
  }\
}

#define SUSPENDED( i, a ) ( a < i -> first ) 



long n_push      =0,
     n_relabel   =0,
     n_discharge =0,
     n_refine    =0,
     n_update    =0,
     n_scan      =0,
     n_prscan    =0,
     n_prscan1   =0,
     n_prscan2   =0,
     n_bad_pricein = 0,
     n_bad_relabel = 0,
     n_prefine   =0;

long   n,                    /* number of nodes */
       m;                    /* number of arcs */

long   *cap;                 /* array containig capacities */

node   *nodes,               /* array of nodes */
       *sentinel_node,       /* next after last */
       *excq_first,          /* first node in push-queue */
       *excq_last;           /* last node in push-queue */

arc    *arcs,                /* array of arcs */
       *sentinel_arc;        /* next after last */

bucket *buckets,             /* array of buckets */
       *l_bucket;            /* last bucket */
long   linf;                 /* number of l_bucket + 1 */

int time_for_price_in;

price_t epsilon,              /* quality measure */
       dn;                   /* cost multiplier - number of nodes  + 1 */
price_t price_min,           /* lowest bound for prices */
       mmc;                  /* multiplied maximal cost */
double f_scale,              /* scale factor */
       cut_off_factor,       /* multiplier to produce cut_on and cut_off
				from n and epsilon */
       cut_on,               /* the bound for returning suspended arcs */
       cut_off;              /* the bound for suspending arcs */

excess_t total_excess;       /* total excess */

long   n_rel,                /* number of relabels from last price update */
       n_ref,                /* current number of refines */
       n_src;                /* current number of nodes with excess */

int   flag_price = 0,        /* if = 1 - signal to start price-in ASAP - 
				maybe there is infeasibility because of
				susoended arcs */
      flag_updt = 0;         /* if = 1 - update failed some sources are 
				unreachable: either the problem is
				unfeasible or you have to return 
                                suspended arcs */

//long  empty_push_bound;      /* maximal possible number of zero pushes
//                                during one discharge */

int   snc_max;               /* maximal number of cycles cancelled
                                during price refine */

arc   d_arc;                 /* dummy arc - for technical reasons */

node  d_node,                /* dummy node - for technical reasons */
      *dummy_node;           /* the address of d_node */

#ifdef CHECK_SOLUTION
long long int *node_balance;
#endif

/* parser for getting  DIMACS format input and transforming the
   data to the internal representation */

#include "parser_cs2.c"


/************************************************ abnormal finish **********/

void err_end ( int cc )

{
fprintf ( stderr, "\nError %d\n", cc );

/*
2 - problem is unfeasible
5 - allocation fault
6 - price overflow
*/

exit ( cc );
}

/************************************************* initialization **********/

void cs_init (long n_p, long m_p, node *nodes_p, arc *arcs_p, 
	      long f_sc, price_t max_c, long *cap_p )

{
node   *i;          /* current node */
arc    *a;          /* current arc */
arc    *a_stop;
long   df;
bucket *b;          /* current bucket */

n             = n_p;
nodes         = nodes_p;
sentinel_node = nodes + n;

m    = m_p;
arcs = arcs_p;
sentinel_arc  = arcs + m;

cap = cap_p;

FOR_ALL_NODES_i 
  {
    i -> price  = 0;
    i -> suspended = i -> first;
    i -> q_next = sentinel_node;
  }

sentinel_node -> first = sentinel_node -> suspended = sentinel_arc;

/* saturate negative arcs, e.g. in the circulation problem case */
FOR_ALL_NODES_i 
  {
    FOR_ACTIVE_ARCS_a_FROM_i 
      {
	if (a -> cost < 0)
	  {
	    if ( ( df = a -> r_cap ) > 0 )
	      {
		INCREASE_FLOW ( i, a -> head, a, df )
	      }
	  }

      }
  }

f_scale = f_sc;

dn = n+1;
#ifdef NO_ZERO_CYCLES 
 dn = 2*dn;
#endif

for ( a = arcs ; a != sentinel_arc ; a ++ )
  a -> cost *= dn;

#ifdef NO_ZERO_CYCLES
for ( a = arcs ; a != sentinel_arc ; a ++ )
  if ((a->cost == 0) && (a->sister->cost == 0)) {
    a->cost = 1;
    a->sister->cost = -1;
  }
#endif

 if ((double) max_c * (double) dn > MAX_64)
   fprintf(stdin, "Warning: arc lengths too large, overflow possible\n");
mmc = max_c * dn;

linf   = (long) (dn * ceil(f_scale) + 2);

buckets = (bucket*) calloc ( linf, sizeof (bucket) );
if ( buckets == NULL ) 
   err_end ( ALLOCATION_FAULT );

l_bucket = buckets + linf;

dnode = &dnd;

for ( b = buckets; b != l_bucket; b ++ )
   RESET_BUCKET ( b );

epsilon = mmc;
if ( epsilon < 1 )
  epsilon = 1;

price_min = -PRICE_MAX;

cut_off_factor = CUT_OFF_COEF * pow ( (double)n, CUT_OFF_POWER );

cut_off_factor = MAX ( cut_off_factor, CUT_OFF_MIN );

n_ref = 0;

flag_price = 0;

dummy_node = &d_node;

excq_first = NULL;

//empty_push_bound = n * EMPTY_PUSH_COEF;

} /* end of initialization */

/********************************************** up_node_scan *************/

void up_node_scan ( node *i )

{
node   *j;                     /* opposite node */
arc    *a,                     /* ( i, j ) */
       *a_stop,                /* first arc from the next node */
       *ra;                    /* ( j, i ) */
bucket *b_old,                 /* old bucket contained j */
       *b_new;                 /* new bucket for j */
long   i_rank,
       j_rank,                 /* ranks of nodes */
       j_new_rank;             
price_t rc,                    /* reduced cost of (j,i) */
       dr;                     /* rank difference */

n_scan ++;

i_rank = i -> rank;

FOR_ACTIVE_ARCS_a_FROM_i 
  {

    ra = a -> sister;

    if ( OPEN ( ra ) )
      {
	j = a -> head;
	j_rank = j -> rank;

	if ( j_rank > i_rank )
	  {
	    if ( ( rc = REDUCED_COST ( j, i, ra ) ) < 0 ) 
	        j_new_rank = i_rank;
	    else
	      {
		dr = rc / epsilon;
		j_new_rank = ( dr < linf ) ? i_rank + (long)dr + 1
		                            : linf;
	      }

	    if ( j_rank > j_new_rank )
	      {
		j -> rank = j_new_rank;
		j -> current = ra;

		if ( j_rank < linf )
		  {
		    b_old = buckets + j_rank;
		    REMOVE_FROM_BUCKET ( j, b_old )
		  }

		b_new = buckets + j_new_rank;
		INSERT_TO_BUCKET ( j, b_new )  
	      }
	  }
      }
  } /* end of scanning arcs */

i -> price -= i_rank * epsilon;
i -> rank = -1;
}


/*************************************************** price_update *******/

void  price_update ()

{

register node   *i;

excess_t remain;               /* total excess of unscanned nodes with
                                  positive excess */
bucket *b;                     /* current bucket */
price_t dp;                    /* amount to be subtracted from prices */

n_update ++;

FOR_ALL_NODES_i 
  {

    if ( i -> excess < 0 )
      {
	INSERT_TO_BUCKET ( i, buckets );
	i -> rank = 0;
      }
    else
      {
        i -> rank = linf;
      }
  }

remain = total_excess;
if ( remain < 0.5 ) return;

/* main loop */

for ( b = buckets; b != l_bucket; b ++ )
  {

    while ( NONEMPTY_BUCKET ( b ) )
       {
	 GET_FROM_BUCKET ( i, b )

	 up_node_scan ( i );

	 if ( i -> excess > 0 )
	   {
	     remain -= (i -> excess);
             if ( remain <= 0  ) break; 
	   }

       } /* end of scanning the bucket */

    if ( remain <= 0  ) break; 
  } /* end of scanning buckets */

if ( remain > 0.5 ) flag_updt = 1;

/* finishup */
/* changing prices for nodes which were not scanned during main loop */

dp = ( b - buckets ) * epsilon;

FOR_ALL_NODES_i 
  {

    if ( i -> rank >= 0 )
    {
      if ( i -> rank < linf )
	REMOVE_FROM_BUCKET ( i, (buckets + i -> rank) );

      if ( i -> price > price_min )
	i -> price -= dp;
    }
  }

} /* end of price_update */



/****************************************************** relabel *********/

int relabel ( node *i )

{
register arc    *a,       /* current arc from  i  */
                *a_stop,  /* first arc from the next node */
                *a_max;   /* arc  which provides maximum price */
register price_t p_max,    /* current maximal price */
                 i_price,  /* price of node  i */
                 dp;       /* current arc partial residual cost */

p_max = price_min;
i_price = i -> price;

 a_max = NULL;
for ( 
      a = i -> current + 1, a_stop = ( i + 1 ) -> suspended;
      a != a_stop;
      a ++
    )
  {
    if ( OPEN ( a )
	 &&
	 ( ( dp = ( ( a -> head ) -> price ) - ( a -> cost ) ) > p_max )
       )
      {
	if ( i_price < dp )
	  {
	    i -> current = a;
	    return ( 1 );
	  }

	p_max = dp;
	a_max = a;
      }
  } /* 1/2 arcs are scanned */


for ( 
      a = i -> first, a_stop = ( i -> current ) + 1;
      a != a_stop;
      a ++
    )
  {
    if ( OPEN ( a )
	 &&
	 ( ( dp = ( ( a -> head ) -> price ) - ( a -> cost ) ) > p_max )
       )
      {
	if ( i_price < dp )
	  {
	    i -> current = a;
	    return ( 1 );
	  }

	p_max = dp;
	a_max = a;
      }
  } /* 2/2 arcs are scanned */

/* finishup */

if ( p_max != price_min )
  {
    i -> price   = p_max - epsilon;
    i -> current = a_max;
  }
else
  { /* node can't be relabelled */
    if ( i -> suspended == i -> first )
      {
	if ( i -> excess == 0 )
	  {
	    i -> price = price_min;
	  }
	else
	  {
	    if ( n_ref == 1 )
	      {
		err_end ( UNFEASIBLE );
	      }
	    else
	      {
		err_end ( PRICE_OFL );
	      }
	  }
      }
    else /* node can't be relabelled because of suspended arcs */
      {
	flag_price = 1;
      }
   }


n_relabel ++;
n_rel ++;

return ( 0 );

} /* end of relabel */


/***************************************************** discharge *********/


void discharge ( node *i )

{

register arc  *a;       /* an arc from  i  */

register node *j;       /* head of  a  */
register long df;       /* amoumt of flow to be pushed through  a  */
excess_t j_exc;             /* former excess of  j  */

n_discharge ++;

a = i -> current;
j = a -> head;

if ( !ADMISSIBLE ( i, j, a ) ) 
  { 
    relabel ( i );
    a = i -> current;
    j = a -> head;
  }

while ( 1 )
{
  j_exc = j -> excess;

  if ( j_exc >= 0 )
    {
      df = MIN( i -> excess, a -> r_cap );
      if (j_exc == 0) n_src++;
      INCREASE_FLOW ( i, j, a, df );
      n_push ++;

      if ( OUT_OF_EXCESS_Q ( j ) )
	{
	  INSERT_TO_EXCESS_Q ( j );
	}
    }
  else /* j_exc < 0 */
    { 
      df = MIN( i -> excess, a -> r_cap );
      INCREASE_FLOW ( i, j, a, df );
      n_push ++;

      if ( j -> excess >= 0 )
	{
	  if ( j -> excess > 0 )
	    {
              n_src++;
	      relabel ( j );
	      INSERT_TO_EXCESS_Q ( j );
	    }
	  total_excess += j_exc;
	}
      else
	total_excess -= df;

    }
  
  if (i -> excess <= 0)
    n_src--;
  if ( i -> excess <= 0 || flag_price ) break;

  relabel ( i );

  a = i -> current;
  j = a -> head;
}

i -> current = a;
} /* end of discharge */

/***************************************************** price_in *******/

int price_in ()

{
node     *i,                   /* current node */
         *j;

arc      *a,                   /* current arc from i */
         *a_stop,              /* first arc from the next node */
         *b,                   /* arc to be exchanged with suspended */
         *ra,                  /* opposite to  a  */
         *rb;                  /* opposite to  b  */

price_t   rc;                  /* reduced cost */

int      n_in_bad,             /* number of priced_in arcs with
				  negative reduced cost */
         bad_found;            /* if 1 we are at the second scan
                                  if 0 we are at the first scan */

excess_t  i_exc,                /* excess of  i  */
          df;                   /* an amount to increase flow */


bad_found = 0;
n_in_bad = 0;

 restart:

FOR_ALL_NODES_i 
  {
    for ( a = ( i -> first ) - 1, a_stop = ( i -> suspended ) - 1; 
	  a != a_stop; a -- )
      {
	rc = REDUCED_COST ( i, a -> head, a );
	
	if ( (rc < 0) && ( a -> r_cap > 0) )
	  { /* bad case */
	    if ( bad_found == 0 )
	      {
		bad_found = 1;
		UPDATE_CUT_OFF;
		goto restart;
		
	      }
	    df = a -> r_cap;
	    INCREASE_FLOW ( i, a -> head, a, df );
	    
	    ra = a -> sister;
	    j  = a -> head;
	    
	    b = -- ( i -> first );
	    EXCHANGE ( a, b );
	    
	    if ( SUSPENDED ( j, ra ) )
	      {
		rb = -- ( j -> first );
		EXCHANGE ( ra, rb );
	      }
	    
	    n_in_bad ++; 
	  }
	else
	  if ( ( rc < cut_on ) && ( rc > -cut_on ) )
	    {
	      i->first--;
	      b = i -> first;
	      EXCHANGE ( a, b );
	    }
      }
  }

if ( n_in_bad != 0 )
  {
    n_bad_pricein ++;

    /* recalculating excess queue */

    total_excess = 0;
    n_src=0;
    RESET_EXCESS_Q;

      FOR_ALL_NODES_i 
	{
	  i -> current = i -> first;
	  i_exc = i -> excess;
	  if ( i_exc > 0 )
	    { /* i  is a source */
	      total_excess += i_exc;
	      n_src++;
	      INSERT_TO_EXCESS_Q ( i );
	    }
	}

    INSERT_TO_EXCESS_Q ( dummy_node );
  }

if (time_for_price_in == TIME_FOR_PRICE_IN2)
  time_for_price_in = TIME_FOR_PRICE_IN3;

if (time_for_price_in == TIME_FOR_PRICE_IN1)
  time_for_price_in = TIME_FOR_PRICE_IN2;

return ( n_in_bad );

} /* end of price_in */

/************************************************** refine **************/

void refine () 

{
node     *i;      /* current node */
excess_t i_exc;   /* excess of  i  */

long   np, nr, ns;  /* variables for additional print */

int    pr_in_int;   /* current number of updates between price_in */

np = n_push; 
nr = n_relabel; 
ns = n_scan;

n_refine ++;
n_ref ++;
n_rel = 0;
pr_in_int = 0;

/* initialize */

total_excess = 0;
n_src=0;
RESET_EXCESS_Q

time_for_price_in = TIME_FOR_PRICE_IN1;

FOR_ALL_NODES_i 
  {
    i -> current = i -> first;
    i_exc = i -> excess;
    if ( i_exc > 0 )
      { /* i  is a source */
	total_excess += i_exc;
        n_src++;
	INSERT_TO_EXCESS_Q ( i )
      }
  }


if ( total_excess <= 0 ) return;

/* main loop */

while ( 1 )
  {
    if ( EMPTY_EXCESS_Q )
      {
	if ( n_ref > PRICE_OUT_START ) 
	  {
	    pr_in_int = 0;
	    price_in ();
	  }
	  
	if ( EMPTY_EXCESS_Q ) break;
      }

    REMOVE_FROM_EXCESS_Q ( i );

    /* push all excess out of i */

    if ( i -> excess > 0 )
     {
       discharge ( i );

       if ( TIME_FOR_UPDATE || flag_price )
	 {
	   if ( i -> excess > 0 )
	     {
	       INSERT_TO_EXCESS_Q ( i );
	     }

	   if ( flag_price && ( n_ref > PRICE_OUT_START ) )
	     {
	       pr_in_int = 0;
	       price_in ();
	       flag_price = 0;
	     }

	   price_update();

	   while ( flag_updt )
	     {
	       if ( n_ref == 1 )
		 {
		   err_end ( UNFEASIBLE );
		 }
	       else
		 {
		   flag_updt = 0;
		   UPDATE_CUT_OFF;
		   n_bad_relabel++;

		   pr_in_int = 0;
		   price_in ();

		   price_update ();
		 }
	     }

	   n_rel = 0;

	   if ( n_ref > PRICE_OUT_START && 
	       (pr_in_int ++ > time_for_price_in) 
	       )
	     {
	       pr_in_int = 0;
	       price_in ();
	     }

	 } /* time for update */
     }
  } /* end of main loop */

return;

} /*----- end of refine */


/*************************************************** price_refine **********/

int price_refine ()

{

node   *i,              /* current node */
       *j,              /* opposite node */
       *ir,             /* nodes for passing over the negative cycle */
       *is;
arc    *a,              /* arc (i,j) */
       *a_stop,         /* first arc from the next node */
       *ar;

long   bmax;            /* number of farest nonempty bucket */
long   i_rank,          /* rank of node i */
       j_rank,          /* rank of node j */
       j_new_rank;      /* new rank of node j */
bucket *b,              /* current bucket */
       *b_old,          /* old and new buckets of current node */
       *b_new;
price_t rc,              /* reduced cost of a */
       dr,              /* ranks difference */
       dp;
int    cc;              /* return code: 1 - flow is epsilon optimal
                                        0 - refine is needed        */
long   df;              /* cycle capacity */

int    nnc,             /* number of negative cycles cancelled during
			   one iteration */
       snc;             /* total number of negative cycle cancelled */

n_prefine ++;

cc=1;
snc=0;

snc_max = ( n_ref >= START_CYCLE_CANCEL ) 
          ? MAX_CYCLES_CANCELLED
          : 0;

/* main loop */

while ( 1 )
{ /* while negative cycle is found or eps-optimal solution is constructed */

nnc=0;

FOR_ALL_NODES_i 
  {
    i -> rank    = 0;
    i -> inp     = WHITE;
    i -> current = i -> first;
  }

RESET_STACKQ

FOR_ALL_NODES_i 
  {
    if ( i -> inp == BLACK ) continue;

    i -> b_next = NULL;

    /* deapth first search */
    while ( 1 )
      {
	i -> inp = GREY;

	/* scanning arcs from node i starting from current */
	FOR_CURRENT_ARCS_a_FROM_i 
	  {
	    if ( OPEN ( a ) )
	      {
		j = a -> head;
		if ( REDUCED_COST ( i, j, a ) < 0 )
		  {
		    if ( j -> inp == WHITE )
		      { /* fresh node  - step forward */
			i -> current = a;
			j -> b_next  = i;
			i = j;
			a = j -> current;
                        a_stop = (j+1) -> suspended;
			break;
		      }

		    if ( j -> inp == GREY )
		      { /* cycle detected */
			cc = 0;
			nnc++;

			i -> current = a;
			is = ir = i;
			df = MAX_32;

			while ( 1 )
			  {
			    ar = ir -> current;
			    if ( ar -> r_cap <= df )
			      {
				df = ar -> r_cap;
			        is = ir;
			      }
			    if ( ir == j ) break;
			    ir = ir -> b_next;
			  } 


			ir = i;

			while ( 1 )
			  {
			    ar = ir -> current;
 			    INCREASE_FLOW( ir, ar -> head, ar, df)

			    if ( ir == j ) break;
			    ir = ir -> b_next;
			  } 


			if ( is != i )
			  {
			    for ( ir = i; ir != is; ir = ir -> b_next )
			      ir -> inp = WHITE;
			    
			    i = is;
			    a = (is -> current) + 1;
                            a_stop = (is+1) -> suspended;
			    break;
			  }

		      }                     
		  }
		/* if j-color is BLACK - continue search from i */
	      }
	  } /* all arcs from i are scanned */

	if ( a == a_stop )
	  {
	    /* step back */
	    i -> inp = BLACK;
n_prscan1++;
	    j = i -> b_next;
	    STACKQ_PUSH ( i );

	    if ( j == NULL ) break;
	    i = j;
	    i -> current ++;
	  }

      } /* end of deapth first search */
  } /* all nodes are scanned */

/* no negative cycle */
/* computing longest paths with eps-precision */


snc += nnc;

if ( snc<snc_max ) cc = 1;

if ( cc == 0 ) break;

bmax = 0;

while ( NONEMPTY_STACKQ )
  {
n_prscan2++;
    STACKQ_POP ( i );
    i_rank = i -> rank;
    FOR_ACTIVE_ARCS_a_FROM_i 
      {
	if ( OPEN ( a ) )
	  {
	    j  = a -> head;
	    rc = REDUCED_COST ( i, j, a );


	    if ( rc < 0 ) /* admissible arc */
	      {
		dr = (price_t) (( - rc - 0.5 ) / epsilon);
		if (( j_rank = dr + i_rank ) < linf )
		  {
		    if ( j_rank > j -> rank )
		      j -> rank = j_rank;
		  }
	      }
	  }
      } /* all arcs from i are scanned */

    if ( i_rank > 0 )
      {
	if ( i_rank > bmax ) bmax = i_rank;
	b = buckets + i_rank;
	INSERT_TO_BUCKET ( i, b )
      }
  } /* end of while-cycle: all nodes are scanned
           - longest distancess are computed */


if ( bmax == 0 ) /* preflow is eps-optimal */
  { break; }

for ( b = buckets + bmax; b != buckets; b -- )
  {
    i_rank = b - buckets;
    dp     = i_rank * epsilon;

    while ( NONEMPTY_BUCKET( b ) )
      {
	GET_FROM_BUCKET ( i, b );

	n_prscan++;
	FOR_ACTIVE_ARCS_a_FROM_i 
	  {
	    if ( OPEN ( a ) )
	      {
		j = a -> head;
        	j_rank = j -> rank;
        	if ( j_rank < i_rank )
	          {
		    rc = REDUCED_COST ( i, j, a );
 
		    if ( rc < 0 ) 
		        j_new_rank = i_rank;
		    else
		      {
			dr = rc / epsilon;
			j_new_rank = ( dr < linf ) ? i_rank - ( (long)dr + 1 )
			                            : 0;
		      }
		    if ( j_rank < j_new_rank )
		      {
			if ( cc == 1 )
			  {
			    j -> rank = j_new_rank;

			    if ( j_rank > 0 )
			      {
				b_old = buckets + j_rank;
				REMOVE_FROM_BUCKET ( j, b_old )
				}

			    b_new = buckets + j_new_rank;
			    INSERT_TO_BUCKET ( j, b_new )  
			  }
			else
			  {
			   df = a -> r_cap;
			    INCREASE_FLOW ( i, j, a, df ) 
			  }
		      }
		  }
	      } /* end if opened arc */
	  } /* all arcs are scanned */

	    i -> price -= dp;

      } /* end of while-cycle: the bucket is scanned */
  } /* end of for-cycle: all buckets are scanned */

if ( cc == 0 ) break;

} /* end of main loop */

/* finish: */

/* if refine needed - saturate non-epsilon-optimal arcs */

if ( cc == 0 )
{ 
FOR_ALL_NODES_i 
  {
    FOR_ACTIVE_ARCS_a_FROM_i 
      {
	if ( REDUCED_COST ( i, a -> head, a ) < -epsilon )
	  {
	    if ( ( df = a -> r_cap ) > 0 )
	      {
		INCREASE_FLOW ( i, a -> head, a, df )
	      }
	  }

      }
  }
}


/*neg_cyc();*/

return ( cc );

} /* end of price_refine */


void compute_prices ()

{

node   *i,              /* current node */
       *j;              /* opposite node */
arc    *a,              /* arc (i,j) */
       *a_stop;         /* first arc from the next node */

long   bmax;            /* number of farest nonempty bucket */
long   i_rank,          /* rank of node i */
       j_rank,          /* rank of node j */
       j_new_rank;      /* new rank of node j */
bucket *b,              /* current bucket */
       *b_old,          /* old and new buckets of current node */
       *b_new;
price_t rc,              /* reduced cost of a */
       dr,              /* ranks difference */
       dp;
int    cc;              /* return code: 1 - flow is epsilon optimal
                                        0 - refine is needed        */


n_prefine ++;

cc=1;

/* main loop */

while ( 1 )
{ /* while negative cycle is found or eps-optimal solution is constructed */


FOR_ALL_NODES_i 
  {
    i -> rank    = 0;
    i -> inp     = WHITE;
    i -> current = i -> first;
  }

RESET_STACKQ

FOR_ALL_NODES_i 
  {
    if ( i -> inp == BLACK ) continue;

    i -> b_next = NULL;

    /* depth first search */
    while ( 1 )
      {
	i -> inp = GREY;

	/* scanning arcs from node i */
	FOR_ALL_ARCS_a_FROM_i 
	  {
	    if ( OPEN ( a ) )
	      {
		j = a -> head;
		if ( REDUCED_COST ( i, j, a ) < 0 )
		  {
		    if ( j -> inp == WHITE )
		      { /* fresh node  - step forward */
			i -> current = a;
			j -> b_next  = i;
			i = j;
			a = j -> current;
                        a_stop = (j+1) -> suspended;
			break;
		      }

		    if ( j -> inp == GREY )
		      { /* cycle detected; should not happen */
			cc = 0;
		      }                     
		  }
		/* if j-color is BLACK - continue search from i */
	      }
	  } /* all arcs from i are scanned */

	if ( a == a_stop )
	  {
	    /* step back */
	    i -> inp = BLACK;
	    n_prscan1++;
	    j = i -> b_next;
	    STACKQ_PUSH ( i );

	    if ( j == NULL ) break;
	    i = j;
	    i -> current ++;
	  }

      } /* end of deapth first search */
  } /* all nodes are scanned */

/* no negative cycle */
/* computing longest paths */

if ( cc == 0 ) break;

bmax = 0;

while ( NONEMPTY_STACKQ )
  {
    n_prscan2++;
    STACKQ_POP ( i );
    i_rank = i -> rank;
    FOR_ALL_ARCS_a_FROM_i 
      {
	if ( OPEN ( a ) )
	  {
	    j  = a -> head;
	    rc = REDUCED_COST ( i, j, a );


	    if ( rc < 0 ) /* admissible arc */
	      {
		dr = - rc;
		if (( j_rank = dr + i_rank ) < linf )
		  {
		    if ( j_rank > j -> rank )
		      j -> rank = j_rank;
		  }
	      }
	  }
      } /* all arcs from i are scanned */

    if ( i_rank > 0 )
      {
	if ( i_rank > bmax ) bmax = i_rank;
	b = buckets + i_rank;
	INSERT_TO_BUCKET ( i, b )
      }
  } /* end of while-cycle: all nodes are scanned
           - longest distancess are computed */


if ( bmax == 0 )
  { break; }

for ( b = buckets + bmax; b != buckets; b -- )
  {
    i_rank = b - buckets;
    dp     = i_rank;

    while ( NONEMPTY_BUCKET( b ) )
      {
	GET_FROM_BUCKET ( i, b )

	  n_prscan++;
	FOR_ALL_ARCS_a_FROM_i 
	  {
	    if ( OPEN ( a ) )
	      {
		j = a -> head;
        	j_rank = j -> rank;
        	if ( j_rank < i_rank )
	          {
		    rc = REDUCED_COST ( i, j, a );
 
		    if ( rc < 0 ) 
		        j_new_rank = i_rank;
		    else
		      {
			dr = rc;
			j_new_rank = ( dr < linf ) ? i_rank - ( (long)dr + 1 )
			                            : 0;
		      }
		    if ( j_rank < j_new_rank )
		      {
			if ( cc == 1 )
			  {
			    j -> rank = j_new_rank;

			    if ( j_rank > 0 )
			      {
				b_old = buckets + j_rank;
				REMOVE_FROM_BUCKET ( j, b_old )
				}

			    b_new = buckets + j_new_rank;
			    INSERT_TO_BUCKET ( j, b_new )  
			  }
		      }
		  }
	      } /* end if opened arc */
	  } /* all arcs are scanned */

	    i -> price -= dp;

      } /* end of while-cycle: the bucket is scanned */
  } /* end of for-cycle: all buckets are scanned */

if ( cc == 0 ) break;

} /* end of main loop */

} /* end of compute_prices */


/***************************************************** price_out ************/

void price_out ()

{
node     *i;                /* current node */

arc      *a,                /* current arc from i */
         *a_stop,           /* first arc from the next node */
         *b;                /* arc to be exchanged with suspended */

double   n_cut_off,         /* -cut_off */
         rc;                /* reduced cost */

n_cut_off = - cut_off;

FOR_ALL_NODES_i 
  {
    FOR_ACTIVE_ARCS_a_FROM_i 
      {
	rc = REDUCED_COST ( i, a -> head, a );

	if (((rc > cut_off) && (CLOSED(a -> sister)))
             ||
             ((rc < n_cut_off) && (CLOSED(a)))
           )
	  { /* suspend the arc */
	    b = i -> first;
	    i->first++;

	    EXCHANGE ( a, b );
	  }
      }
  }

} /* end of price_out */


/**************************************************** update_epsilon *******/
/*----- decrease epsilon after epsilon-optimal flow is constructed */

int update_epsilon()
{

if ( epsilon <= 1 ) return ( 1 );

epsilon = (price_t) (ceil ( (double) epsilon / f_scale ));

cut_off        = cut_off_factor * epsilon;
cut_on         = cut_off * CUT_OFF_GAP;

return ( 0 );
}


#ifdef CHECK_SOLUTION
int check_feas(node *nodes, arc *arp)

{
  node *i;
  arc *a, *a_stop;
  long fa;
  int ans = 1;

  FOR_ALL_NODES_i {
    FOR_ALL_ARCS_a_FROM_i {
      if (cap[N_ARC(a)] > 0) {
	fa = cap[N_ARC(a)] - a->r_cap;
	if (fa < 0) {
	  ans = 0;
       	  break;
	}
	node_balance [i - nodes] -= fa;
	node_balance [a->head - nodes] += fa;
      }
    }
  }

  FOR_ALL_NODES_i {
    if (node_balance [i-nodes] != 0) {
      ans = 0;
      break;
    }
  }

  return(ans);
}

#endif

/* check complimentary slackness */
int check_cs ()

{
  node *i;
  arc *a, *a_stop;

  FOR_ALL_NODES_i
    FOR_ALL_ARCS_a_FROM_i
    if (OPEN(a) && (REDUCED_COST(i, a->head, a) < 0)) {
      return(0);
    }

  return(1);
}

int check_eps_opt ()

{
  node *i;
  arc *a, *a_stop;

  FOR_ALL_NODES_i
    FOR_ALL_ARCS_a_FROM_i
    if (OPEN(a) && (REDUCED_COST(i, a->head, a) < -epsilon)) {
      return(0);
    }

  return(1);
}

/*************************************************** finishup ***********/
void finishup ( double *obj_ad )

{
arc   *a;            /* current arc */
long  na;            /* corresponding position in capacity array */
double  obj_internal;/* objective */
price_t cs;          /* actual arc cost */
long   flow;         /* flow through an arc */
node *i;

obj_internal = 0;

#ifdef NO_ZERO_CYCLES
for ( a = arcs ; a != sentinel_arc ; a ++ )
  if (a->cost == 1) {
    assert(a->sister->cost == -1);
    a->cost = 0;
    a->sister->cost = 0;
  }
#endif


for ( a = arcs, na = 0; a != sentinel_arc ; a ++, na ++ )
    {
      cs = a -> cost / dn;

      if ( cap[na]  > 0 && ( flow = cap[na] - (a -> r_cap) ) != 0 )
	obj_internal += (double) cs * (double) flow; 

      a -> cost = cs;
    }

FOR_ALL_NODES_i {
  i->price = (i->price / dn);
}

#ifdef COMP_DUALS
  compute_prices ();
#endif

*obj_ad = obj_internal;

}

/*********************************************** init_solution ***********/
void init_solution ( )


{
arc   *a;            /* current arc  (i,j) */
node  *i,            /* tail of  a  */
      *j;            /* head of  a  */
long  df;            /* residual capacity */

for ( a = arcs; a != sentinel_arc ; a ++ )
    {
      if ( a -> r_cap > 0 && a -> cost < 0 )
	{
	  df = a -> r_cap;
	  i  = ( a -> sister ) -> head;
          j  = a -> head;
	  INCREASE_FLOW ( i, j, a, df );
	}
    }
}

/************************************************* cs2 - main program ***/

void  cs2 (long n_p, long m_p, node *nodes_p, arc *arcs_p, 
	   long f_sc, price_t max_c, long *cap_p, double *obj_ad)

{

  int cc;             /* for storing return code */
  cs_init ( n_p, m_p, nodes_p, arcs_p, f_sc, max_c, cap_p );
  
  /*init_solution ( );*/
  printf ("c scale-factor: %8.0f     cut-off-factor: %6.1f\nc\n",
	  f_scale, cut_off_factor );
  
  cc = 0;
  update_epsilon ();
  
  do {  /* scaling loop */
    refine ();
    if ( n_ref >= PRICE_OUT_START )
      price_out ( );

    if ( update_epsilon () ) break;

    while (1) {
      if ( ! price_refine () ) break;

      if ( n_ref >= PRICE_OUT_START ) {
	if ( price_in () ) 
	  break; 
	if ((cc = update_epsilon ())) break;
      }
    }
  } while ( cc == 0 );

finishup ( obj_ad );

}



#ifdef COST_RESTART

void cs_cost_reinit ()

{
  node   *i;          /* current node */
  arc    *a;          /* current arc */
  bucket *b;          /* current bucket */
  price_t rc, minc, sum;
  arc *a_stop;

  for (b = buckets; b != l_bucket; b ++)
    RESET_BUCKET(b);

  rc = 0;
  FOR_ALL_NODES_i 
    {
      rc = MIN(rc, i -> price);
      i -> first = i -> suspended;
      i -> current = i -> first;
      i -> q_next = sentinel_node;
    }

  /* make prices nonnegative and multiply */
  FOR_ALL_NODES_i {
    i->price = (i->price - rc) * dn;
  }

  /* multiply arc costs */
  for (a = arcs ; a != sentinel_arc ; a ++)
    a -> cost *= dn;

/* for debugging only 
  for (a = arcs ; a != sentinel_arc ; a ++)
    if (a->cost >= 0) {
      a -> cost += n;
      a -> sister ->cost -= n;
    }
   */

  sum = 0;
  FOR_ALL_NODES_i {
    minc = 0;
    FOR_ACTIVE_ARCS_a_FROM_i {
      if ((OPEN(a) && ((rc = REDUCED_COST(i, a->head, a)) < 0)))
	minc = MAX(epsilon, -rc);
    }
    sum += minc;
  }

  epsilon = ceil (sum / dn);

  cut_off_factor = CUT_OFF_COEF * pow ((double) n, CUT_OFF_POWER);

  cut_off_factor = MAX ( cut_off_factor, CUT_OFF_MIN );

  n_ref = 0;

  n_refine = n_discharge = n_push=n_relabel = 0;
  n_update = n_scan = n_prefine = n_prscan = n_prscan1 = n_bad_pricein
           = n_bad_relabel = 0;

  flag_price = 0;

  excq_first = NULL;

  //  empty_push_bound = n * EMPTY_PUSH_COEF;

} /* end of reinitialization */

/******** restart after a cost update ***/

void cs2_cost_restart (double *obj_ad)


{
  int cc;             /* for storing return code */

  printf("c \nc ******************************\n");
  printf("c Restarting after a cost update\n");
  printf("c ******************************\nc\n");

  cs_cost_reinit ();
  
  printf ("c Init. epsilon = %6.0f\n", epsilon);
  cc = update_epsilon();
  
  if (cc != 0)
    printf("c Old solution is optimal\n");
  else {

     do {  /* scaling loop */

      while ( 1 )
	{
	  if ( ! price_refine () ) break;

	  if ( n_ref >= PRICE_OUT_START )
	    {
	      if ( price_in () ) 
		{ 
		  break; 
		}
	    }
	  if ((cc = update_epsilon ())) break;
	}

      if (cc) break;

      refine ();

      if ( n_ref >= PRICE_OUT_START )
	{
	  price_out ( );
	}

      if ( update_epsilon () ) break;
    } while (cc == 0);
  }

  finishup ( obj_ad );

}
#endif

#ifdef PRINT_ANS
void print_solution(node *ndp, arc *arp, long nmin, double *cost)
{
  node *i;
  arc *a;
  long ni;
  price_t cost2;

  printf ("c\ns %7.0f\n", *cost );
  for ( i = ndp; i < ndp + n; i ++ )
    {
      ni = N_NODE ( i );
      for ( a = i -> suspended; a != (i+1)->suspended; a ++ )
	{
	  if ( cap[ N_ARC (a) ]  > 0 ) {
	    printf ( "f %7ld %7ld %10ld\n",
		     ni, N_NODE( a -> head ), cap[ N_ARC (a) ] - ( a -> r_cap )
		     );
	  }
	}
    }

#ifdef COMP_DUALS
  /* find minimum price */
  cost2 = MAX_32;
  FOR_ALL_NODES_i {
    cost2 = MIN(*cost, i->price);
  }
  FOR_ALL_NODES_i {
    printf("p %7ld %7.2lld\n", N_NODE(i), i->price - cost2);
  }
#endif

  printf("c\n");
}
#endif

int main (int argc, char **argv)

{

  double t;
  arc *arp;
  node *ndp;
  long n, m, m2, nmin; 
  
  double cost;
  
  price_t c_max;
  long f_sc;
  long *cap;
  
  
  f_sc = (long) (( argc > 1 ) ? atoi( argv[1] ): SCALE_DEFAULT);
  
  printf ("c CS 4.6\n");
  printf ("c Commercial use requires a licence\n");
  printf ("c contact igsys@eclipse.net\nc\n");
  
  parse( &n, &m, &ndp, &arp, &nmin, &c_max, &cap );
  
  nodes = ndp;
  
#ifdef CHECK_SOLUTION
  node_balance = (long long int *) calloc (n+1, sizeof(long long int));
  node *i;
  for (i = ndp; i < ndp+n; i++) {
    node_balance[i-ndp] = i->excess;
  }
#endif
  
  
  m2 = 2 * m;
  printf ("c nodes: %15ld     arcs:  %15ld\n", n, m ); 
  
  t = timer();
  cs2 ( n, m2, ndp, arp, f_sc, c_max, cap, &cost );
  t = timer() - t;
  
  
  printf ("c time:  %15.2f     cost:  %15.0f\n", t, cost);
  
  printf ("c refines:    %10ld     discharges: %10ld\n",
	  n_refine, n_discharge);
  printf ("c pushes:     %10ld     relabels:   %10ld\n",
	  n_push, n_relabel);
  printf ("c updates:    %10ld     u-scans:    %10ld\n",
	  n_update, n_scan);
  printf ("c p-refines:  %10ld     r-scans:    %10ld\n",
	  n_prefine, n_prscan);
  printf ("c dfs-scans:  %10ld     bad-in:     %4ld  + %2ld\n",
	  n_prscan1, n_bad_pricein, n_bad_relabel);
  
#ifdef CHECK_SOLUTION
  printf("c checking feasibility...\n"); 
  if (check_feas(ndp, arp))
    printf("c ...OK\n");
  else
    printf("c ERROR: solution infeasible\n");
  printf("c computing prices and checking CS...\n");
  compute_prices ();
  if (check_cs())
    printf("c ...OK\n");
  else
    printf("ERROR: CS violation\n");
#endif
  
#ifdef PRINT_ANS
  print_solution(ndp, arp, nmin, &cost);
#endif

  free(cap);
  free(nodes - nmin);
  free(arcs);
  free(buckets);
#ifdef CHECK_SOLUTION
  free(node_balance);
#endif

 return 0;
}
