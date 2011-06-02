// $Id$

//#include "Defines.h"
#include <iostream>

///Modified for VC
#if defined(_WIN32)
///////////////////////////////////////////////////////////////////////
//Time information for ms windows
#include <sys/timeb.h>
#else

///////////////////////////////////////////////////////////////////////
//Time information for unix
#include <sys/time.h>
#endif

#include "Clock_Elapsed.h"


/////////////////////////////////////////////////////////////////////
//
//  Clock_Elapsed.c
//
//  General Description
//      This class provides methods to handle clocks to time events.
//
//  Created
//      11/20/03 Shawna Thomas
//
/////////////////////////////////////////////////////////////////////

//----------------------------------------
// Clock_Elapsed constructor
//----------------------------------------
Clock_Elapsed::
Clock_Elapsed() {
  timerclear(&s_time);
  timerclear(&u_time);
  timerclear(&elapsed);
};

//----------------------------------------
// Clock_Elapsed Destructor
//----------------------------------------
Clock_Elapsed::
~Clock_Elapsed() {
};

//----------------------------------------
// Clear the clock
//----------------------------------------
int
Clock_Elapsed::
ClearClock() {
  timerclear(&s_time);
  timerclear(&u_time);
  timerclear(&elapsed);
  return 1;
};

//----------------------------------------
// Start a clock and give it a name of Name
//----------------------------------------
int
Clock_Elapsed::

StartClock(string Name) {
  struct timezone tz;
  gettimeofday(&s_time, &tz);
  ClockName = Name;
  return 1;
};

//----------------------------------------
// Stop the clock
//----------------------------------------
int
Clock_Elapsed::
StopClock() {
  struct timezone tz;
  gettimeofday(&u_time, &tz);

  struct timeval diff;
  if(s_time.tv_usec > u_time.tv_usec) {
    u_time.tv_usec += 1000000;
    u_time.tv_sec--;
  }

  diff.tv_usec = u_time.tv_usec - s_time.tv_usec;
  diff.tv_sec = u_time.tv_sec - s_time.tv_sec;

  elapsed.tv_usec += diff.tv_usec;
  elapsed.tv_sec += diff.tv_sec;
  if(elapsed.tv_usec > 1000000) {
    elapsed.tv_usec -= 1000000;
    elapsed.tv_sec++;
  }

  return 1;
};

//----------------------------------------
// Stop the clock and Print its current value
//----------------------------------------
int
Clock_Elapsed::
StopPrintClock() {
  StopClock();
  PrintClock();
  return 1;
};

//----------------------------------------
// Print the current value of the clock
//---------------------------------------- 
void
Clock_Elapsed::
PrintClock() {
  cout << ClockName << ": " << GetClock_SEC() << " sec" <<endl;
};

//----------------------------------------
// Retrieve the current value of the clock
//---------------------------------------- 
int
Clock_Elapsed::
GetClock() {
  return elapsed.tv_sec;
};

//----------------------------------------
// Print the name of the clock provided in the
// call to StartClock
//----------------------------------------
void
Clock_Elapsed::
PrintName() {
    cout << ClockName;
};

//----------------------------------------
// Retrieve the seconds on the clock
//----------------------------------------
double
Clock_Elapsed::
GetClock_SEC() {
  return (double)(elapsed.tv_sec + elapsed.tv_usec/1e6);
};

//----------------------------------------
// Retrieve the u seconds on the clock
//----------------------------------------
int
Clock_Elapsed::
GetClock_USEC() {
  return (int)(elapsed.tv_sec*1e6 + elapsed.tv_usec);
};

