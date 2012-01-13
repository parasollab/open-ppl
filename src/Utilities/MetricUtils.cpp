#include <iostream>

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

struct rusage buf;

#include "MetricUtils.h"
#include "GraphAlgo.h"
/////////////////////////////////////////////////////////////////////
//
//  StatClass.c
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

const int StatClass::ALL= -1;

// StatClass Class Constructor
StatClass::
StatClass() {
  ClearStats();
};

// StatClass Class Deconstructor
StatClass::
~StatClass() {
};

//----------------------------------------
//  Clear all stats
//----------------------------------------
void
StatClass::
ClearStats() {

  // initialize the number of collision detection calls to 0
  m_numCollDetCalls.clear();

  // initialize each local planners successful connections, attempts,
  // and collision detection calls to 0
  m_lpConnections.clear();
  m_lpAttempts.clear();
  m_lpCollDetCalls.clear();

  m_connectionsAttempted = 0;
  m_connectionsMade = 0;
  m_nodesAttempted = 0;
  m_nodesGenerated = 0;
  m_ccNumber = 0;

  m_avgMinIntraCCDist = 0;
  m_avgMaxIntraCCDist = 0;
  m_avgMeanIntraCCDist = 0;
  m_avgSigmaIntraCCDist = 0;

  m_avgMinIntraCCEdge = 0;
  m_avgMaxIntraCCEdge = 0;
  m_avgMeanIntraCCEdge = 0;
  m_avgSigmaIntraCCEdge = 0;

  m_avgMaxIntraCCDistToCm = 0;
  m_avgMinIntraCCDistToCm = 0;
  m_avgMeanIntraCCDistToCm = 0;
  m_avgSigmaIntraCCDistToCm = 0;

  m_maxInterCCDist = 0.0;
  m_avgInterCCDist = 0.0;
  m_sigmaInterCCDist = 0.0;
  m_minInterCCDist = 100000.0;

  m_maxCCSize = 0.0;
  m_minCCSize = 0.0;
  m_avgCCSize = 0.0;
  m_sigmaCCSize = 0.0;

  m_collDetCountByName.clear();

  m_isCollByName.clear();
  m_isCollTotal = 0;
};

//----------------------------------------
// Increment the number of collision detection
// calls for the method by the name of CDName
//----------------------------------------
int
StatClass::
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
StatClass::
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
StatClass::
IncLPConnections( string _lpName ) {
  m_lpConnections[_lpName]++;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName by incr
//----------------------------------------
int
StatClass::
IncLPConnections( string _lpName ,int _incr) {
  m_lpConnections[_lpName] += _incr;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Decrement the number of connections made
// by local planning method named LPName
//----------------------------------------
int
StatClass::
DecLPConnections(string _lpName) {
  m_lpConnections[_lpName]--;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName by decr
//----------------------------------------
int
StatClass::
DecLPConnections(string _lpName, int _decr) {
  m_lpConnections[_lpName] -= _decr;
  return m_lpConnections[_lpName];
};


//----------------------------------------
// Set the number of connections made by local
// planning method LPName to Connections
//----------------------------------------
int
StatClass::
SetLPConnections(string _lpName, int _connections) {
  m_lpConnections[_lpName]=_connections;
  return m_lpConnections[_lpName];
};

//----------------------------------------
// Increment the number attempts made by the
// local planning method named LPName by incr
//----------------------------------------
int
StatClass::
IncLPAttempts(string _lpName , int _incr) {
  m_lpAttempts[_lpName] += _incr;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Increment the number attempts made by the
// local planning method named LPName
//----------------------------------------
int
StatClass::
IncLPAttempts(string _lpName ) {
  m_lpAttempts[_lpName]++;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Decrement the number attempts made by the
// local planning method named LPName
//----------------------------------------
int
StatClass::
DecLPAttempts(string _lpName) {
  m_lpAttempts[_lpName]--;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Decrement the number attempts made by the
// local planning method named LPName by decr
//----------------------------------------
int
StatClass::
DecLPAttempts(string _lpName ,int _decr) {
  m_lpAttempts[_lpName] -= _decr;
  return m_lpAttempts[_lpName];
};

//----------------------------------------
// Set the number of attempts made by local
// planning method LPName to Attempts
//----------------------------------------
int
StatClass::
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
StatClass::
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
StatClass::
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
StatClass::
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
StatClass::
DecLPCollDetCalls(string _lpName, int _decr) {
  m_lpCollDetCalls[_lpName] -= _decr;
  return m_lpCollDetCalls[_lpName];
};

void
StatClass::
PrintFeatures(ostream& _os) {
  _os << "General features:" << endl;
  _os << "\tm_ccNumber: " << m_ccNumber << endl;
  _os << "\tm_connectionsAttempted: " << m_connectionsAttempted << endl;
  _os << "\tm_connectionsMade: " << m_connectionsMade << endl;
  _os << "\tpct_succesful_connections: " << ((double)m_connectionsMade)/m_connectionsAttempted << endl;

  _os << "General features:" << endl;
  _os << "Intra-cc features:" << endl;
  _os << "\tm_avgMinIntraCCDist: " << m_avgMinIntraCCDist << endl;
  _os << "\tm_avgMaxIntraCCDist: " << m_avgMaxIntraCCDist << endl;
  _os << "\tm_avgMeanIntraCCDist: " << m_avgMeanIntraCCDist << endl;
  _os << "\tm_avgSigmaIntraCCDist: " << m_avgSigmaIntraCCDist << endl;
  _os << "\tm_avgMinIntraCCEdge: " << m_avgMinIntraCCEdge << endl;
  _os << "\tm_avgMaxIntraCCEdge: " << m_avgMaxIntraCCEdge << endl;
  _os << "\tm_avgMeanIntraCCEdge: " << m_avgMeanIntraCCEdge << endl;
  _os << "\tm_avgSigmaIntraCCEdge: " << m_avgSigmaIntraCCEdge << endl;
  _os << "\tm_avgMaxIntraCCDistToCm: " << m_avgMaxIntraCCDistToCm << endl;
  _os << "\tm_avgMinIntraCCDistToCm: " << m_avgMinIntraCCDistToCm << endl;
  _os << "\tm_avgMeanIntraCCDistToCm: " << m_avgMeanIntraCCDistToCm << endl;
  _os << "\tm_avgSigmaIntraCCDistToCm: " << m_avgSigmaIntraCCDistToCm << endl;

  _os << "Inter-cc features:" << endl;
  _os << "\tm_maxInterCCDist: " << m_maxInterCCDist << endl;
  _os << "\tm_minInterCCDist: " << m_minInterCCDist << endl;
  _os << "\tm_avgInterCCDist: " << m_avgInterCCDist << endl;
  _os << "\tm_sigmaInterCCDist: " << m_sigmaInterCCDist << endl << endl;
  _os << "\tm_minCCSize: " << m_minCCSize << endl;
  _os << "\tm_maxCCSize: " << m_maxCCSize << endl;
  _os << "\tm_avgCCSize: " << m_avgCCSize << endl;
  _os << "\tm_sigmaCCSize: " << m_sigmaCCSize << endl;
}

void
StatClass::
IncNodes_Generated(){
  m_nodesGenerated++;
};

void
StatClass::
IncNodes_Attempted(){
  m_nodesAttempted++;
};
void
StatClass::
IncConnections_Attempted(){
  m_connectionsAttempted++;
};

void
StatClass::
IncConnections_Made(){
  m_connectionsMade++;
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
  ClearClock();
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
void
ClockClass::
ClearClock() {
  m_sTime=0;
  m_uTime=0;
  m_suTime=0;
  m_uuTime=0;
};

//----------------------------------------
// Start a clock and give it a name of Name
//----------------------------------------
void
ClockClass::
StartClock(string _name) {
  getrusage(RUSAGE_SELF, &buf);
  m_suTime = buf.ru_utime.tv_usec;
  m_sTime = buf.ru_utime.tv_sec;
  m_clockName = _name;
};

//----------------------------------------
// Stop the clock
//----------------------------------------
void
ClockClass::
StopClock() {
  getrusage(RUSAGE_SELF, &buf);
  m_uuTime = buf.ru_utime.tv_usec - m_suTime;
  m_uTime = buf.ru_utime.tv_sec - m_sTime;
};

//----------------------------------------
// Stop the clock and Print its current value
//----------------------------------------
void
ClockClass::
StopPrintClock() {
  StopClock();
  PrintClock();
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
  return m_uTime;
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
  return (double)m_uuTime/1e6+m_uTime;
};

//----------------------------------------
// Retrieve the u seconds on the clock
//----------------------------------------
int
ClockClass::
GetUSeconds() {
  return (int)(m_uTime*1e6+m_uuTime);
};

