// $Id$

#include <iostream.h>
#include <time.h>
#include <string.h>

///Modified for VC
#if defined(_WIN32)
///////////////////////////////////////////////////////////////////////
//Time information for ms windows
#include <sys/timeb.h>
_timeb timebuffer;
#else

///////////////////////////////////////////////////////////////////////
//Time information for unix
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
struct rusage buf;
#endif

#include "Clock_Class.h"


/////////////////////////////////////////////////////////////////////
//
//  Clock_Class.c
//
//  General Description
//      This class provides methods to handle clocks to time events.
//
//  Created
//      1/27/99 Chris Jones
//
/////////////////////////////////////////////////////////////////////

//----------------------------------------
// Clock_Class constructor
//----------------------------------------
Clock_Class::
Clock_Class() {
  s_time=0;
  u_time=0;
  s_utime=0;
  u_utime=0;
};

//----------------------------------------
// Clock_Class Destructor
//----------------------------------------
Clock_Class::
~Clock_Class() {
};

//----------------------------------------
// Clear the clock
//----------------------------------------
int
Clock_Class::
ClearClock() {
  s_time=0;
  u_time=0;
  s_utime=0;
  u_utime=0;
  return(1);
};

//----------------------------------------
// Start a clock and give it a name of Name
//----------------------------------------
int
Clock_Class::

StartClock(char *Name) {

///Modified for VC
#if defined(_WIN32)
  _ftime( &timebuffer );
  s_utime=timebuffer.millitm;	//millisecond
  s_time=timebuffer.time;		//second
#else
  getrusage(RUSAGE_SELF, &buf);
  s_utime = buf.ru_utime.tv_usec;
  s_time = buf.ru_utime.tv_sec;
#endif

  strcpy(ClockName,Name);
  return(1);
};

//----------------------------------------
// Stop the clock
//----------------------------------------
int
Clock_Class::
StopClock() {

///Modified for VC
#if defined(_WIN32)
  _ftime( &timebuffer );
  u_utime=timebuffer.millitm - s_utime;	//millisecond
  u_time=timebuffer.time - s_time;		//second
#else
  getrusage(RUSAGE_SELF, &buf);
  u_utime = buf.ru_utime.tv_usec - s_utime;
  u_time = buf.ru_utime.tv_sec - s_time;
#endif

  return(1);
};

//----------------------------------------
// Stop the clock and Print its current value
//----------------------------------------
int
Clock_Class::
StopPrintClock() {
  StopClock();
  PrintClock();
  return(1);
};

//----------------------------------------
// Print the current value of the clock
//---------------------------------------- 
void
Clock_Class::
PrintClock() {
    cout << ClockName << ": " << u_time << " sec" <<
 endl;
};

//----------------------------------------
// Retrieve the current value of the clock
//---------------------------------------- 
int
Clock_Class::
GetClock() {
  return(u_time);
};

//----------------------------------------
// Print the name of the clock provided in the
// call to StartClock
//----------------------------------------
void
Clock_Class::
PrintName() {
    cout << ClockName ;
};

//----------------------------------------
// Retrieve the seconds on the clock
//----------------------------------------
double
Clock_Class::
GetClock_SEC() {

///Modified for VC
#if defined(_WIN32)
  return (double)u_utime/1e3+u_time;
#else
  return (double)u_utime/1e6+u_time;
#endif
};

//----------------------------------------
// Retrieve the u seconds on the clock
//----------------------------------------
int
Clock_Class::
GetClock_USEC() {

///Modified for VC
#if defined(_WIN32)
  return u_time*1e6+u_utime*1e3; //here u_utime is millisecond
#else
  return u_time*1e6+u_utime;
#endif
};

