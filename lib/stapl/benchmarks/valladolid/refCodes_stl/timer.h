#include <time.h>
#include <sys/time.h>

struct timespec start;
struct timespec stop;
const clockid_t clk_id = CLOCK_MONOTONIC_RAW;

void start_timer()
{
  start = timespec();
  clock_gettime(clk_id, &start);
}

double stop_timer()
{
  stop = timespec();
  clock_gettime(clk_id, &stop);
  const timespec diff = { (stop.tv_sec - start.tv_sec),
                          (stop.tv_nsec - start.tv_nsec) };
  return (double(diff.tv_sec) + double(diff.tv_nsec)*1.0E-9);
}
