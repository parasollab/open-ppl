// $Id$

#include <iostream.h>
#include "Clock_Class.h"
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>




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

struct rusage buf;

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
   // brc changed to RUSAGE 
  //getrusage((int) 0, &buf);
  getrusage(RUSAGE_SELF, &buf);
  s_utime = buf.ru_utime.tv_usec;
  s_time = buf.ru_utime.tv_sec;
  strcpy(ClockName,Name);
  return(1);
};

//----------------------------------------
// Stop the clock
//----------------------------------------
int
Clock_Class::
StopClock() {
  // brc changed to RUSAGE_SELF
  //getrusage(0, &buf);
  getrusage(RUSAGE_SELF, &buf);
  u_utime = buf.ru_utime.tv_usec - s_utime;
  u_time = buf.ru_utime.tv_sec - s_time;
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
  return (double)u_utime/1e6+u_time;
};

//----------------------------------------
// Retrieve the u seconds on the clock
//----------------------------------------
int
Clock_Class::
GetClock_USEC() {
  return         u_time*1e6+u_utime;
};

