#include <sys/time.h>
#define get_sec(tv) tv.tv_sec+tv.tv_usec/1000000.0
extern struct timeval tp;
#define seconds(tm) gettimeofday(&tp,(struct timezone *)0);tm=get_sec(tp);
#include <time.h>
