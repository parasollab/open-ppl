#include <iostream>

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

struct rusage buf;

#include "MetricUtils.h"
#include "GraphAlgo.h"
/////////////////////////////////////////////////////////////////////
//
//  Stat_Class.c
//
//  General Description
//      This class lets you keep statistics on various aspects of
//      the program (ie. number of collision detection calls,
//      number of calls to each local planner, etc).
//
//  Created
//     1/27/99  Chris Jones
//
/////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
// Static Data Member init

const int Stat_Class::ALL= -1;

// Stat_Class Class Constructor
Stat_Class::
Stat_Class() {
  ClearStats();
};

// Stat_Class Class Deconstructor
Stat_Class::
~Stat_Class() {
};

//----------------------------------------
//  Clear all stats
//----------------------------------------
void
Stat_Class::
ClearStats() {

  // initialize the number of collision detection calls to 0
  m_numCollDetCalls.clear();

  // initialize each local planners successful connections, attempts,
  // and collision detection calls to 0
  m_lpConnections.clear();
  m_lpAttempts.clear();
  m_lpCollDetCalls.clear();

  Connections_Attempted = 0;
  Connections_Made = 0;
  Nodes_Attempted = 0;
  Nodes_Generated = 0;
  cc_number = 0;

  avg_min_intracc_dist = 0;
  avg_max_intracc_dist = 0;
  avg_mean_intracc_dist = 0;
  avg_sigma_intracc_dist = 0;

  avg_min_intracc_edge_s = 0;
  avg_max_intracc_edge_s = 0;
  avg_mean_intracc_edge_s = 0;
  avg_sigma_intracc_edge_s = 0;

  avg_max_intracc_dist_to_cm = 0;
  avg_min_intracc_dist_to_cm = 0;
  avg_mean_intracc_dist_to_cm = 0;
  avg_sigma_intracc_dist_to_cm = 0;

  max_intercc_dist = 0.0;
  avg_intercc_dist = 0.0;
  sigma_intercc_dist = 0.0;
  min_intercc_dist = 100000.0;

  max_cc_size = 0.0;
  min_cc_size = 0.0;
  avg_cc_size = 0.0;
  sigma_cc_size = 0.0;

  m_collDetCountByName.clear();

  m_isCollByName.clear();
  m_isCollTotal = 0;
};

//----------------------------------------
// Increment the number of collision detection
// calls for the method by the name of CDName
//----------------------------------------
int
Stat_Class::
IncNumCollDetCalls( string _cdName, string *_pCallName){
  m_numCollDetCalls[_cdName]++;

  // If a caller's name was provided
  // then increment the verification counter
  // with that name.
         
  if( _pCallName )
  { m_collDetCountByName[*_pCallName]++; }

  return m_numCollDetCalls[_cdName];
};

//----------------------------------------
// Increment the number of Cfg::isCollision
// calls 
//---------------------------------------- 
void
Stat_Class::
IncCfgIsColl( string *_pCallName) {

  if( _pCallName )
  { m_isCollByName[*_pCallName]++; }
  else { m_isCollByName["UNKNOWN"]++; }

  m_isCollTotal++;

};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName
//----------------------------------------
int
Stat_Class::
IncLPConnections( string _lpName ) {
  m_lpConnections[_lpName]++;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName by incr
//----------------------------------------
int
Stat_Class::
IncLPConnections( string _lpName ,int _incr) {
  m_lpConnections[_lpName] += _incr;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Decrement the number of connections made
// by local planning method named LPName
//----------------------------------------
int
Stat_Class::
DecLPConnections(string _lpName) {
  m_lpConnections[_lpName]--;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName by decr
//----------------------------------------
int
Stat_Class::
DecLPConnections(string _lpName, int _decr) {
  m_lpConnections[_lpName] -= _decr;
  return m_lpConnections[_lpName];
};


//----------------------------------------
// Set the number of connections made by local
// planning method LPName to Connections
//----------------------------------------
int
Stat_Class::
SetLPConnections(string _lpName, int _connections) {
  m_lpConnections[_lpName]=_connections;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Increment the number attempts made by the
// local planning method named LPName by incr
//----------------------------------------
int
Stat_Class::
IncLPAttempts(string _lpName , int _incr) {
  m_lpAttempts[_lpName] += _incr;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Increment the number attempts made by the
// local planning method named LPName
//----------------------------------------
int
Stat_Class::
IncLPAttempts(string _lpName ) {
  m_lpAttempts[_lpName]++;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Decrement the number attempts made by the
// local planning method named LPName
//----------------------------------------
int
Stat_Class::
DecLPAttempts(string _lpName) {
  m_lpAttempts[_lpName]--;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Decrement the number attempts made by the
// local planning method named LPName by decr
//----------------------------------------
int
Stat_Class::
DecLPAttempts(string _lpName ,int _decr) {
  m_lpAttempts[_lpName] -= _decr;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Set the number of attempts made by local
// planning method LPName to Attempts
//----------------------------------------
int
Stat_Class::
SetLPAttempts(string _lpName, int _attempts) {
  m_lpAttempts[_lpName]=_attempts;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Increment the number of collision detection
// calls made by local planning method named
// LPName
//----------------------------------------
int
Stat_Class::
IncLPCollDetCalls(string _lpName) {
  m_lpCollDetCalls[_lpName]++;
  return m_lpCollDetCalls[_lpName];
};

//----------------------------------------
// Increment the number of collision detection
// calls made by local planning method named
// LPName by incr
//----------------------------------------
int
Stat_Class::
IncLPCollDetCalls(string _lpName, int _incr) {
  m_lpCollDetCalls[_lpName] += _incr;
  return m_lpCollDetCalls[_lpName];
};

//----------------------------------------
// Decrement the number of collision detection
// calls made by local planning method named
// LPName
//----------------------------------------
int
Stat_Class::
DecLPCollDetCalls(string _lpName) {
  m_lpCollDetCalls[_lpName]--;
  return m_lpCollDetCalls[_lpName];
};

//----------------------------------------
// Decrement the number of collision detection
// calls made by local planning method named
// LPName by decr
//----------------------------------------
int
Stat_Class::
DecLPCollDetCalls(string _lpName, int _decr) {
  m_lpCollDetCalls[_lpName] -= _decr;
  return m_lpCollDetCalls[_lpName];
};

void
Stat_Class::
PrintFeatures() {
  cout << "General features:" << endl;
  //cout << "\tNodes_Attempted: " << Nodes_Attempted << endl;
  //cout << "\tNodes_Generated: " << Nodes_Generated << endl;
  //cout << "\tpct_free_nodes: " << ((double)Nodes_Generated)/Nodes_Attempted << endl;
  cout << "\tcc_number: " << cc_number << endl;
  cout << "\tConnections_Attempted: " << Connections_Attempted << endl;
  cout << "\tConnections_Made: " << Connections_Made << endl;
  cout << "\tpct_succesful_connections: " << ((double)Connections_Made)/Connections_Attempted << endl;

  cout << "Intra-cc features:" << endl;
  cout << "\tavg_min_intracc_dist: " << avg_min_intracc_dist << endl;
  cout << "\tavg_max_intracc_dist: " << avg_max_intracc_dist << endl;
  cout << "\tavg_mean_intracc_dist: " << avg_mean_intracc_dist << endl;
  cout << "\tavg_sigma_intracc_dist: " << avg_sigma_intracc_dist << endl;
  cout << "\tavg_min_intracc_edge_s: " << avg_min_intracc_edge_s << endl;
  cout << "\tavg_max_intracc_edge_s: " << avg_max_intracc_edge_s << endl;
  cout << "\tavg_mean_intracc_edge_s: " << avg_mean_intracc_edge_s << endl;
  cout << "\tavg_sigma_intracc_edge_s: " << avg_sigma_intracc_edge_s << endl;

  cout << "\tavg_max_intracc_dist_to_cm: " << avg_max_intracc_dist_to_cm << endl;
  cout << "\tavg_min_intracc_dist_to_cm: " << avg_min_intracc_dist_to_cm << endl;
  cout << "\tavg_mean_intracc_dist_to_cm: " << avg_mean_intracc_dist_to_cm << endl;
  cout << "\tavg_sigma_intracc_dist_to_cm: " << avg_sigma_intracc_dist_to_cm << endl;

  cout << "Inter-cc features:" << endl;
  cout << "\tmax_intercc_dist: " << max_intercc_dist << endl;
  cout << "\tmin_intercc_dist: " << min_intercc_dist << endl;
  cout << "\tavg_intercc_dist: " << avg_intercc_dist << endl;
  cout << "\tsigma_intercc_dist: " << sigma_intercc_dist <<endl<<endl;

  cout << "\tmin_cc_size: " << min_cc_size << endl;
  cout << "\tmax_cc_size: " << max_cc_size << endl;
  cout << "\tavg_cc_size: " << avg_cc_size << endl;
  cout << "\tsigma_cc_size: " << sigma_cc_size << endl;
}

void
Stat_Class::
IncNodes_Generated(){
  Nodes_Generated++;
};

void
Stat_Class::
IncNodes_Attempted(){
  Nodes_Attempted++;
};
void
Stat_Class::
IncConnections_Attempted(){
  Connections_Attempted++;
};

void
Stat_Class::
IncConnections_Made(){
  Connections_Made++;
};

/////////////////////////////////////////////////////////////////////
//
//  ClockClass
//
//  General Description
//      This class provides methods to handle clocks to time events.
//
/////////////////////////////////////////////////////////////////////

//----------------------------------------
// ClockClass constructor
//----------------------------------------
ClockClass::
ClockClass() {
  timerclear(&m_sTime);
  timerclear(&m_uTime);
  timerclear(&m_elapsed);
};

//----------------------------------------
// ClockClass Destructor
//----------------------------------------
ClockClass::
~ClockClass() {
};

//----------------------------------------
// Clear the clock
//----------------------------------------
int
ClockClass::
ClearClock() {
  timerclear(&m_sTime);
  timerclear(&m_uTime);
  timerclear(&m_elapsed);
  return 1;
};

//----------------------------------------
// Start a clock and give it a name of Name
//----------------------------------------
int
ClockClass::
StartClock(string _name) {
  struct timezone tz;
  gettimeofday(&m_sTime, &tz);
  m_clockName = _name;
  return 1;
};

//----------------------------------------
// Stop the clock
//----------------------------------------
int
ClockClass::
StopClock() {
  struct timezone tz;
  gettimeofday(&m_uTime, &tz);

  struct timeval diff;
  if(m_sTime.tv_usec > m_uTime.tv_usec) {
    m_uTime.tv_usec += 1000000;
    m_uTime.tv_sec--;
  }

  diff.tv_usec = m_uTime.tv_usec - m_sTime.tv_usec;
  diff.tv_sec = m_uTime.tv_sec - m_sTime.tv_sec;

  m_elapsed.tv_usec += diff.tv_usec;
  m_elapsed.tv_sec += diff.tv_sec;
  if(m_elapsed.tv_usec > 1000000) {
    m_elapsed.tv_usec -= 1000000;
    m_elapsed.tv_sec++;
  }

  return 1;
};

//----------------------------------------
// Stop the clock and Print its current value
//----------------------------------------
int
ClockClass::
StopPrintClock() {
  StopClock();
  PrintClock();
  return 1;
};

//----------------------------------------
// Print the current value of the clock
//----------------------------------------
void
ClockClass::
PrintClock() {
  cout << m_clockName << ": " << GetSeconds() << " sec" <<endl;
};

//----------------------------------------
// Retrieve the current value of the clock
//----------------------------------------
int
ClockClass::
GetClock() {
  return m_elapsed.tv_sec;
};

//----------------------------------------
// Print the name of the clock provided in the
// call to StartClock
//----------------------------------------
void
ClockClass::
PrintName() {
    cout << m_clockName;
};

//----------------------------------------
// Retrieve the seconds on the clock
//----------------------------------------
double
ClockClass::
GetSeconds() {
  return (double)(m_elapsed.tv_sec + m_elapsed.tv_usec/1e6);
};

//----------------------------------------
// Retrieve the u seconds on the clock
//----------------------------------------
int
ClockClass::
GetUSeconds() {
  return (int)(m_elapsed.tv_sec*1e6 + m_elapsed.tv_usec);
};

