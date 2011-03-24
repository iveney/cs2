/*-----------------------------------------------------------
  parse (...) :                                                   
       1. Reads minimum-cost flow problem in  DIMACS format.   
       2. Prepares internal data representation.

   types: 'arc' and 'node' must be predefined

   type   node   must contain the field 'first': 

   typedef
     struct node_st
       {
          arc_st        *first;    ..  first outgoing arc 
          ....................
       }
    node;
--------------------------------------------------------------*/

int parse(long *n_ad, long *m_ad, node **nodes_ad, 
	  arc **arcs_ad, long *node_min_ad, price_t *m_c_ad, 
	  long **cap_ad )

{

#define MAXLINE       100	/* max line length in the input file */
#define ARC_FIELDS      5	/* no of fields in arc line  */
#define NODE_FIELDS     2	/* no of fields in node line  */
#define P_FIELDS        3       /* no of fields in problem line */
#define PROBLEM_TYPE "min"      /* name of problem type*/
#define ABS( x ) ( (x) >= 0 ) ? (x) : -(x)

long inf_cap = 0;
long    n,                      /* internal number of nodes */
        node_min,               /* minimal no of node  */
        node_max,               /* maximal no of nodes */
       *arc_first,              /* internal array for holding
                                     - node degree
                                     - position of the first outgoing arc */
       *arc_tail,               /* internal array: tails of the arcs */
        /* temporary variables carrying no of nodes */
        head, tail, i;

long    m,                      /* internal number of arcs */
        /* temporary variables carrying no of arcs */
        last, arc_num, arc_new_num;

node    *nodes,                 /* pointers to the node structure */
        *head_p,
        *ndp,
        *in,
        *jn;

arc     *arcs,                  /* pointers to the arc structure */
        *arc_current,
        *arc_new,
        *arc_tmp;

long    excess,                 /* supply/demand of the node */
        low,                    /* lowest flow through the arc */
        acap;                    /* capacity */

price_t cost,                   /* arc cost */
        m_c;                    /* maximum arc cost */

long    *cap;                   /* array of capacities */

excess_t total_p,                /* total supply */
        total_n,                /* total demand */
        cap_out,                /* sum of outgoing capacities */
        cap_in;                 /* sum of incoming capacities */

long    no_lines=0,             /* no of current input line */
        no_plines=0,            /* no of problem-lines */
        no_nlines=0,            /* no of node lines */
        no_alines=0,            /* no of arc-lines */
        pos_current=0;          /* 2*no_alines */

char    in_line[MAXLINE],       /* for reading input line */
        pr_type[3];             /* for reading type of the problem */

int     k,                      /* temporary */
        err_no;                 /* no of detected error */

/* -------------- error numbers & error messages ---------------- */
#define EN1   0
#define EN2   1
#define EN3   2
#define EN4   3
#define EN6   4
#define EN10  5
#define EN7   6
#define EN8   7
#define EN9   8
#define EN11  9
#define EN12 10
#define EN13 11
#define EN14 12
#define EN16 13
#define EN15 14
#define EN17 15
#define EN18 16
#define EN21 17
#define EN19 18
#define EN20 19
#define EN22 20
#define EN23 22

static char *err_message[] = 
  { 
/* 0*/    "more than one problem line",
/* 1*/    "wrong number of parameters in the problem line",
/* 2*/    "it is not a Min-cost problem line",
/* 3*/    "bad value of a parameter in the problem line",
/* 4*/    "can't obtain enough memory to solve this problem",
/* 5*/    "",
/* 6*/    "can't read problem name",
/* 7*/    "problem description must be before node description",
/* 8*/    "wrong capacity bounds",
/* 9*/    "wrong number of parameters in the node line",
/*10*/    "wrong value of parameters in the node line",
/*11*/    "unbalanced problem",
/*12*/    "node descriptions must be before arc descriptions",
/*13*/    "too many arcs in the input",
/*14*/    "wrong number of parameters in the arc line",
/*15*/    "wrong value of parameters in the arc line",
/*16*/    "unknown line type in the input",
/*17*/    "read error",
/*18*/    "not enough arcs in the input",
/*19*/    "warning: capacities too big - excess overflow possible",
/*20*/    "can't read anything from the input file",
/*21*/    "warning: infinite capacity replaced by BIGGEST_FLOW",
/*22*/    "error: node ids must start from 0 or 1"
  };
/* --------------------------------------------------------------- */

/* The main loop:
        -  reads the line of the input,
        -  analises its type,
        -  checks correctness of parameters,
        -  puts data to the arrays,
        -  does service functions
*/

while ( gets ( in_line ) != NULL )
  {
  no_lines ++;


  switch (in_line[0])
    {
      case 'c':                  /* skip lines with comments */
      case '\n':                 /* skip empty lines   */
      case '\0':                 /* skip empty lines at the end of file */
                break;

      case 'p':                  /* problem description      */
                if ( no_plines > 0 )
                   /* more than one problem line */
                   { err_no = EN1 ; goto error; }

                no_plines = 1;
   
                if (
        /* reading problem line: type of problem, no of nodes, no of arcs */
                    sscanf ( in_line, "%*c %3s %ld %ld", pr_type, &n, &m )
                != P_FIELDS
                   )
		    /*wrong number of parameters in the problem line*/
		    { err_no = EN2; goto error; }

                if ( strcmp ( pr_type, PROBLEM_TYPE ) )
		    /*wrong problem type*/
		    { err_no = EN3; goto error; }

                if ( n <= 0  || m <= 0 )
		    /*wrong value of no of arcs or nodes*/
		    { err_no = EN4; goto error; }


        /* allocating memory for  'nodes', 'arcs'  and internal arrays */
                nodes    = (node*) calloc ( n+2, sizeof(node) );
		arcs     = (arc*)  calloc ( 2*m+1, sizeof(arc) );
	        cap      = (long*) calloc ( 2*m,   sizeof(long) ); 
	        arc_tail = (long*) calloc ( 2*m,   sizeof(long) ); 
		arc_first= (long*) calloc ( n+2, sizeof(long) );
                /* arc_first [ 0 .. n+1 ] = 0 - initialized by calloc */

		for ( in = nodes; in <= nodes + n; in ++ )
		   in -> excess = 0;
		    

                if ( nodes == NULL || arcs == NULL || 
                     arc_first == NULL || arc_tail == NULL )
                    /* memory is not allocated */
		    { err_no = EN6; goto error; }
		     
		/* setting pointer to the first arc */
		arc_current = arcs;
                node_max = 0;
                node_min = n;
		m_c      = 0;
		total_p = total_n = 0;

		for ( ndp = nodes; ndp < nodes + n; ndp ++ )
		  ndp -> excess = 0;

                break;

      case 'n':		         /* node description */
	//		if ( no_alines > 0 ) 
	//                  /* there were arc descriptors before  */
	//                  { err_no = EN14; goto error; }

		if ( no_plines == 0 )
                  /* there was no problem line above */
                  { err_no = EN8; goto error; }

		no_nlines ++;

                /* reading node */
		k = sscanf ( in_line,"%*c %ld %ld", &i, &excess );
 
		if ( k < NODE_FIELDS )
                  /* node line is incorrect */
                  { err_no = EN11; goto error; }

		if ( i < 0 || i > n )
                  /* wrong number of the node */
                  { err_no = EN12; goto error; }

		( nodes + i ) -> excess = excess;
		if ( excess > 0 ) total_p += excess;
		if ( excess < 0 ) total_n -= excess;

		break;

      case 'a':                    /* arc description */

		if ( no_nlines == 0 ) 
                  /* there was no nodes  description above */
                  { err_no = EN14; goto error; }

		if ( no_alines >= m )
                  /*too many arcs on input*/
                  { err_no = EN16; goto error; }
		
		if (
                    /* reading an arc description */
                    sscanf ( in_line,"%*c %ld %ld %ld %ld %lld",
                                      &tail, &head, &low, &acap, &cost )
                    != ARC_FIELDS
                   ) 
                    /* arc description is not correct */
                    { err_no = EN15; goto error; }

		if ( tail < 0  ||  tail > n  ||
                     head < 0  ||  head > n  
		   )
                    /* wrong value of nodes */
		    { err_no = EN17; goto error; }

		if ( acap < 0 ) {
		  acap = MAX_32;
		  if (!inf_cap) {
		    inf_cap = 1;
		    fprintf ( stderr, "\n%s\n", err_message[21] );
		  }
		}

		if ( low < 0 || low > acap )
		  { err_no = EN9; goto error; }

               /* no of arcs incident to node i is placed in arc_first[i+1] */
		arc_first[tail + 1] ++; 
		arc_first[head + 1] ++;
		in    = nodes + tail;
		jn    = nodes + head;

                /* storing information about the arc */
		arc_tail[pos_current]        = tail;
		arc_tail[pos_current+1]      = head;
		arc_current       -> head    = jn;
		arc_current       -> r_cap   = acap - low;
		cap[pos_current]             = acap;
		arc_current       -> cost    = cost;
		arc_current       -> sister  = arc_current + 1;
	      ( arc_current + 1 ) -> head    = nodes + tail;
	      ( arc_current + 1 ) -> r_cap   = 0;
	        cap[pos_current+1]           = 0;
	      ( arc_current + 1 ) -> cost    = -cost;
	      ( arc_current + 1 ) -> sister  = arc_current;

		in -> excess -= low;
		jn -> excess += low;

		/* searching for minimum and maximum node */
                if ( head < node_min ) node_min = head;
                if ( tail < node_min ) node_min = tail;
                if ( head > node_max ) node_max = head;
                if ( tail > node_max ) node_max = tail;

		if ( cost < 0 ) cost = -cost;
		if ( cost > m_c && acap > 0 ) m_c = cost;

		no_alines   ++;
		arc_current += 2;
		pos_current += 2;

		break;

	default:
		/* unknown type of line */
		err_no = EN18; goto error;
		break;

    } /* end of switch */
}     /* end of input loop */

/* ----- all is red  or  error while reading ----- */ 

if ( feof (stdin) == 0 ) /* reading error */
  { err_no=EN21; goto error; } 

if ( no_lines == 0 ) /* empty input */
  { err_no = EN22; goto error; } 

if ( no_alines < m ) /* not enough arcs */
  { err_no = EN19; goto error; } 

if ( ABS( total_p - total_n ) > 0.5 ) /* unbalanced problem */
  { err_no = EN13; goto error; }

/********** ordering arcs - linear time algorithm ***********/

/* first arc from the first node */
( nodes + node_min ) -> first = arcs;

/* before below loop arc_first[i+1] is the number of arcs outgoing from i;
   after this loop arc_first[i] is the position of the first 
   outgoing from node i arcs after they would be ordered;
   this value is transformed to pointer and written to node.first[i]
   */
 
for ( i = node_min + 1; i <= node_max + 1; i ++ ) 
  {
    arc_first[i]          += arc_first[i-1];
    ( nodes + i ) -> first = arcs + arc_first[i];
  }


for ( i = node_min; i < node_max; i ++ ) /* scanning all the nodes  
                                            exept the last*/
  {

    last = ( ( nodes + i + 1 ) -> first ) - arcs;
                             /* arcs outgoing from i must be cited    
                              from position arc_first[i] to the position
                              equal to initial value of arc_first[i+1]-1  */

    for ( arc_num = arc_first[i]; arc_num < last; arc_num ++ )
      { tail = arc_tail[arc_num];

	while ( tail != i )
          /* the arc no  arc_num  is not in place because arc cited here
             must go out from i;
             we'll put it to its place and continue this process
             until an arc in this position would go out from i */

	  { arc_new_num  = arc_first[tail];
	    arc_current  = arcs + arc_num;
	    arc_new      = arcs + arc_new_num;
	    
	    /* arc_current must be cited in the position arc_new    
	       swapping these arcs:                                 */

	    head_p               = arc_new -> head;
	    arc_new -> head      = arc_current -> head;
	    arc_current -> head  = head_p;

	    acap                 = cap[arc_new_num];
	    cap[arc_new_num]     = cap[arc_num];
	    cap[arc_num]         = acap;

	    acap                 = arc_new -> r_cap;
	    arc_new -> r_cap     = arc_current -> r_cap;
	    arc_current -> r_cap = acap;

	    cost                = arc_new -> cost;
	    arc_new -> cost      = arc_current -> cost;
	    arc_current -> cost  = cost;

	    if ( arc_new != arc_current -> sister )
	      {
	        arc_tmp                = arc_new -> sister;
	        arc_new  -> sister     = arc_current -> sister;
	        arc_current -> sister  = arc_tmp;

                ( arc_current -> sister ) -> sister = arc_current;
		( arc_new     -> sister ) -> sister = arc_new;
	      }

	    arc_tail[arc_num] = arc_tail[arc_new_num];
	    arc_tail[arc_new_num] = tail;

	    /* we increase arc_first[tail]  */
	    arc_first[tail] ++ ;

            tail = arc_tail[arc_num];
	  }
      }
    /* all arcs outgoing from  i  are in place */
  }       

/* -----------------------  arcs are ordered  ------------------------- */

/*------------ testing network for possible excess overflow ---------*/

for ( ndp = nodes + node_min; ndp <= nodes + node_max; ndp ++ )
{
   cap_in  =   ( ndp -> excess );
   cap_out = - ( ndp -> excess );
   for ( arc_current = ndp -> first; arc_current != (ndp+1) -> first; 
         arc_current ++ )
      {
	arc_num = arc_current - arcs;
	if ( cap[arc_num] > 0 ) cap_out += cap[arc_num];
	if ( cap[arc_num] == 0 ) 
	  cap_in += cap[( arc_current -> sister )-arcs];
      }
}

if ((node_min < 0) || (node_min > 1)) /* unbalanced problem */
  { err_no = EN23; goto error; }

/* ----------- assigning output values ------------*/
*m_ad = m;
*n_ad = node_max - node_min + 1;
*node_min_ad = node_min;
*nodes_ad = nodes + node_min;
*arcs_ad = arcs;
*m_c_ad  = m_c;
*cap_ad   = cap;

/* free internal memory */
free ( arc_first ); free ( arc_tail );

/* Thanks God! All is done! */
return (0);

/* ---------------------------------- */
 error:  /* error found reading input */

fprintf ( stderr, "\nline %ld of input - %s\n", 
         no_lines, err_message[err_no] );

exit (1);

}
/* --------------------   end of parser  -------------------*/

