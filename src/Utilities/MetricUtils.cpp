#include <iostream>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include "GraphAlgo.h"
#include "MetricUtils.h"

struct rusage buf;
/////////////////////////////////////////////////////////////////////
//
//  StatClass.cpp
//
//  General Description
//      This class lets you keep statistics on various aspects of
//      the program (ie. number of collision detection calls,
//      number of calls to each local planner, etc).
/////////////////////////////////////////////////////////////////////

const int StatClass::ALL= -1;

StatClass::StatClass() {
  ClearStats();
}

StatClass::~StatClass() {}

//----------------------------------------
//  Clear all stats
//----------------------------------------
void
StatClass::ClearStats() {

  // initialize the number of collision detection calls to 0
  m_numCollDetCalls.clear();

  // initialize each local planners successful connections, attempts,
  // and collision detection calls to 0
  m_lpInfo.clear();

  m_samplerInfo.clear();

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
}

//----------------------------------------
// Increment the number of collision detection
// calls for the method by the name of CDName
//----------------------------------------
int
StatClass::IncNumCollDetCalls(string _cdName, string* _callName) {
  m_numCollDetCalls[_cdName]++;

  // If a caller's name was provided
  // then increment the verification counter
  // with that name.

  if( _callName )
  { m_collDetCountByName[*_callName]++; }

  return m_numCollDetCalls[_cdName];
}

//----------------------------------------
// Increment the number of Cfg::isCollision
// calls 
//---------------------------------------- 
void
StatClass::IncCfgIsColl(string* _callName) {
  if(_callName )
    m_isCollByName[*_callName]++; 
  else
    m_isCollByName["UNKNOWN"]++; 

  m_isCollTotal++;
}

//----------------------------------------
// Increment the number of connections made
// by local planning method named LPName by incr
//----------------------------------------
int
StatClass::IncLPConnections(string _lpName, int _incr) {
  return (m_lpInfo[_lpName].get<1>() += _incr);
}

//----------------------------------------
// Increment the number attempts made by the
// local planning method named LPName by incr
//----------------------------------------
int
StatClass::IncLPAttempts(string _lpName, int _incr) {
  return (m_lpInfo[_lpName].get<0>() += _incr);
}

//----------------------------------------
// Increment the number of collision detection
// calls made by local planning method named
// LPName by incr
//----------------------------------------
int
StatClass::IncLPCollDetCalls(string _lpName, int _incr) {
  return (m_lpInfo[_lpName].get<2>() += _incr);
}

void
StatClass::PrintFeatures(ostream& _os) {
  unsigned long int connectionsAttempted = 0, connectionsMade = 0;
  std::map<std::string, boost::tuple<unsigned long int,unsigned long int, unsigned long int> >::iterator sumIter;
  for (sumIter=m_lpInfo.begin();sumIter!=m_lpInfo.end();sumIter++) {
    connectionsAttempted += sumIter->second.get<0>();
    connectionsMade += sumIter->second.get<1>();
  }

  _os << "General features:" << endl;
  _os << "\tccNumber: " << m_ccNumber << endl;
  _os << "\tconnectionsAttempted: " << connectionsAttempted << endl;
  _os << "\tconnectionsMade: " << connectionsMade << endl;
  _os << "\tpct_succesful_connections: ";

  if (connectionsAttempted == 0)
    _os << 0.0 << endl;
  else 
    _os << ((double)connectionsMade)/connectionsAttempted << endl;

  _os << "General features:" << endl;
  _os << "Intra-cc features:" << endl;
  _os << "\tavgMinIntraCCDist: " << m_avgMinIntraCCDist << endl;
  _os << "\tavgMaxIntraCCDist: " << m_avgMaxIntraCCDist << endl;
  _os << "\tavgMeanIntraCCDist: " << m_avgMeanIntraCCDist << endl;
  _os << "\tavgSigmaIntraCCDist: " << m_avgSigmaIntraCCDist << endl;
  _os << "\tavgMinIntraCCEdge: " << m_avgMinIntraCCEdge << endl;
  _os << "\tavgMaxIntraCCEdge: " << m_avgMaxIntraCCEdge << endl;
  _os << "\tavgMeanIntraCCEdge: " << m_avgMeanIntraCCEdge << endl;
  _os << "\tavgSigmaIntraCCEdge: " << m_avgSigmaIntraCCEdge << endl;
  _os << "\tavgMaxIntraCCDistToCm: " << m_avgMaxIntraCCDistToCm << endl;
  _os << "\tavgMinIntraCCDistToCm: " << m_avgMinIntraCCDistToCm << endl;
  _os << "\tavgMeanIntraCCDistToCm: " << m_avgMeanIntraCCDistToCm << endl;
  _os << "\tavgSigmaIntraCCDistToCm: " << m_avgSigmaIntraCCDistToCm << endl;

  _os << "Inter-cc features:" << endl;
  _os << "\tmaxInterCCDist: " << m_maxInterCCDist << endl;
  _os << "\tminInterCCDist: " << m_minInterCCDist << endl;
  _os << "\tavgInterCCDist: " << m_avgInterCCDist << endl;
  _os << "\tsigmaInterCCDist: " << m_sigmaInterCCDist << endl << endl;
  _os << "\tminCCSize: " << m_minCCSize << endl;
  _os << "\tmaxCCSize: " << m_maxCCSize << endl;
  _os << "\tavgCCSize: " << m_avgCCSize << endl;
  _os << "\tsigmaCCSize: " << m_sigmaCCSize << endl;
}

int
StatClass::IncNodesGenerated(string _samplerName, int _incr){
  return (m_samplerInfo[_samplerName].second += _incr);
}

int
StatClass::IncNodesAttempted(string _samplerName, int _incr){
  return (m_samplerInfo[_samplerName].first += _incr);
}

void
StatClass::StartClock(string _name) {
  if(_name != "") {
    if(m_clockMap.find(_name)==m_clockMap.end())
      m_clockMap[_name].SetName(_name);
    m_clockMap[_name].StartClock();
  }
  else  
    cerr<<"Error::Attempting to start a non-existing clock"<< endl; 
}

void
StatClass::StopClock(string _name) {
  if(_name != "")
    m_clockMap[_name].StopClock(); 
  else 
    cerr<<"Error::Attempting to stop a non-existing clock"<< _name << endl; 
}

void 
StatClass::StopPrintClock(string _name, ostream& _os) {
  if(_name != "")
    m_clockMap[_name].StopPrintClock(_os);
  else 
    cerr<<"Error::Attempting to stop and print a non-existing clock"<< _name << endl; 
}

void 
StatClass::PrintClock(string _name, ostream& _os) {
  if(_name !="")
    m_clockMap[_name].PrintClock(_os);
  else 
    cerr<<"Error::Attempting to print a non-existing clock"<< _name << endl; 
}

void
StatClass::ClearClock(string _name) {
  if(_name != "")
    m_clockMap[_name].ClearClock();
  else 
    cerr<<"Error::Attempting to clear a non-existing clock"<< _name << endl; 
}

double
StatClass::GetSeconds(string _name) {
  if(_name !="")
    return m_clockMap[_name].GetSeconds();
  else {
    cerr<<"Attempting to GetSeconds for a non-existing clock:: "<< _name << endl; 
    return 0;
  }
}

int 
StatClass::GetUSeconds(string _name) {
  if(_name != "")
    return m_clockMap[_name].GetUSeconds();
  else {
    cerr<<"Attempting to GetUSeconds for a non-existing clock::"<< _name << endl; 
    return 0;
  }
}

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
ClockClass::ClockClass() {
  ClearClock();
}

//----------------------------------------
// ClockClass Destructor
//----------------------------------------
ClockClass::
~ClockClass() {}

void 
ClockClass::SetName(string _name){
  m_clockName = _name;
}

//----------------------------------------
// Clear the clock
//----------------------------------------
void
ClockClass::ClearClock() {
  m_sTime=m_uTime=m_suTime=m_uuTime=0;
}

//----------------------------------------
// Start a clock and give it a name of Name
//----------------------------------------
void
ClockClass::StartClock() {
  getrusage(RUSAGE_SELF, &buf);
  m_suTime = buf.ru_utime.tv_usec;
  m_sTime = buf.ru_utime.tv_sec;
}

//----------------------------------------
// Stop the clock
//----------------------------------------
void
ClockClass::StopClock() {
  getrusage(RUSAGE_SELF, &buf);
  m_uuTime += buf.ru_utime.tv_usec - m_suTime;
  m_uTime += buf.ru_utime.tv_sec - m_sTime;
}

//----------------------------------------
// Stop the clock and Print its current value
//----------------------------------------
void
ClockClass::StopPrintClock(ostream& _os) {
  StopClock();
  PrintClock(_os);
}

//----------------------------------------
// Print the current value of the clock
//----------------------------------------
void
ClockClass::PrintClock(ostream& _os) {
  _os << m_clockName << ": " << GetSeconds() << " sec" <<endl;
}

//----------------------------------------
// Retrieve the seconds on the clock
//----------------------------------------
double
ClockClass::GetSeconds() {
  return (double)m_uuTime/1e6+m_uTime;
}

//----------------------------------------
// Retrieve the u seconds on the clock
//----------------------------------------
int
ClockClass::GetUSeconds() {
  return (int)(m_uTime*1e6+m_uuTime);
}

//Roadmap Clearance returns a RoadmapClearanceStats structure that includes information regarding the clearance of a
//roadmap
/*RoadmapClearanceStats
RoadmapClearance(Roadmap<CfgType, WeightType> _g, const ClearanceParams& _cParams){
  RoadmapClearanceStats output;
  ///temp until edge iterator & operator != in new stapl dynamic graph is fixed
  #ifndef _PARALLEL
  double minClearance = 1e6;
  double runningTotal = 0;
  RoadmapGraph<CfgType, WeightType>* graph = _g.m_pRoadmap;
  vector<double> clearanceVec;
  for(RoadmapGraph<CfgType, WeightType>::edge_iterator it = graph->edges_begin(); it != graph->edges_end(); it++){
    double currentClearance = MinEdgeClearance((*graph->find_vertex((*it).source())).property(),
      (*graph->find_vertex((*it).target())).property(), (*it).property(), _cParams);
    clearanceVec.push_back(currentClearance);//Save this value for variance computation later
    runningTotal+=currentClearance;
    if(currentClearance < minClearance){//Did we find a new minimum clearance value?
      minClearance = currentClearance;
    }
  }
  output.m_minClearance = minClearance;
  double average = runningTotal / graph->get_num_edges();
  output.m_avgClearance = average;
  double varSum = 0;
  for(vector<double>::iterator it = clearanceVec.begin(); it != clearanceVec.end(); it++){
    varSum+=pow(((*it) - average), 2);
  }
  output.m_clearanceVariance = varSum / clearanceVec.size();
  #endif
  return output;
}

double
MinEdgeClearance(const CfgType& _c1, const CfgType& _c2, const WeightType& _weight, const ClearanceParams& _cParams){
  MPProblem* mp = _cParams.GetMPProblem();
  Environment* env = mp->GetEnvironment();
  double minClearance = 1e6;
  shared_ptr<DistanceMetricMethod> dummy; //Currently not used
  //Reconstruct the path given the two nodes
  vector<CfgType> intermediates = _weight.GetIntermediates();
  LocalPlanners<CfgType, WeightType>* lp = mp->GetMPStrategy()->GetLocalPlanners();
  vector<CfgType> reconEdge = lp->GetMethod(_weight.GetLPLabel())->ReconstructPath(env, dummy, _c1, _c2, intermediates, env->GetPositionRes(), env->GetOrientationRes());  
  for(vector<CfgType>::iterator it = reconEdge.begin(); it != reconEdge.end(); it++){
    CDInfo collInfo;
    StatClass dummyStats; //Not used
    CfgType clrCfg;
    //Decide which collision info function to use
    double currentClearance;
    if(_cParams.m_exactClearance){
      GetExactCollisionInfo(*it, dummyStats, collInfo, _cParams);
      currentClearance = collInfo.m_minDist;
    }
    else{
      GetApproxCollisionInfo(*it, clrCfg, dummyStats, collInfo, _cParams);
      currentClearance = collInfo.m_minDist;
    }
    //If newly computed clearance is less than the previous minimum, it becomes the minimum
    if(currentClearance < minClearance){
      minClearance = currentClearance;
    }
  }
  return minClearance;
}
*/
