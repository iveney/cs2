#include <time.h>


/*********************************************************************/
/*                                                                   */
/* current processor time in seconds                                 */
/* difference between two calls is processor time spent by your code */
/* needs: <sys/types.h>, <sys/times.h>                               */
/* depends on compiler and OS                                        */
/*                                                                   */
/*********************************************************************/

static double timer ()
{
  double ans;

  ans = (double) clock();
  ans = ans / (double) CLOCKS_PER_SEC;

  return (ans);

}
