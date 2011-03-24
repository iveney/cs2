#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>


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
  struct rusage r;

  getrusage(0, &r);
  return (double)(r.ru_utime.tv_sec+r.ru_utime.tv_usec/(double)1000000);
}
