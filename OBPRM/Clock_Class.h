// $Id$

#include <time.h>
#ifdef BSD
#include <sys/time.h>
#endif
#include <sys/resource.h>

// brc added extern 
//extern rusage buf;


/////////////////////////////////////////////////////////////////////
//
//  Clock_Class.h
//
//  General Description
//	This class provides methods to handle clocks to time events.
//
//  Created
//	1/27/99	Chris Jones
//
///////////////////////////////////////////////////////////////////// 
class Clock_Class {
public:
  Clock_Class();
  ~Clock_Class();

  int ClearClock();
  int StartClock( char *Name );
  int StopClock();
  int StopPrintClock();
  void PrintClock();
  int GetClock();

  void PrintName();
  double GetClock_SEC();
  int GetClock_USEC();
 
private:
  int s_time, u_time;
  int s_utime, u_utime;
  char ClockName[50];
};

