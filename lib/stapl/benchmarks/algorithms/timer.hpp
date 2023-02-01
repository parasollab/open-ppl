#include <sys/time.h>
#include <unistd.h>


struct timeval start_timer()
{
  struct timeval  tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);
  return tv;
}

double stop_timer(struct timeval const& start)
{
  struct timeval  stop;
  struct timezone tz;
  gettimeofday(&stop, &tz);

  const double CLOCK_RATIO_S  = 1000000.0;   // 10^6

  return stop.tv_usec < start.tv_usec ?
    (stop.tv_sec - 1 - start.tv_sec)*1.0 +
    (CLOCK_RATIO_S + stop.tv_usec - start.tv_usec)/CLOCK_RATIO_S :
    (stop.tv_sec - start.tv_sec)*1.0 +
    (stop.tv_usec - start.tv_usec)/CLOCK_RATIO_S;
}
