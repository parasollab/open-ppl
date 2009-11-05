#ifndef _NEIGHBORHOOD_FINDER_METHOD_H_
#define _NEIGHBORHOOD_FINDER_METHOD_H_

#include <string>
#include <iostream>
#include "LabeledObject.h"

class NeighborhoodFinderMethod : public LabeledObject  {

public:
  NeighborhoodFinderMethod(std::string in_strLabel) : 
        LabeledObject(in_strLabel), m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0) { }
  NeighborhoodFinderMethod() : m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0) { }
  virtual const std::string GetName () const = 0;
  virtual void PrintOptions(std::ostream& out_os) const = 0;
  
  double GetQueryTime() const { return m_query_time; }
  double GetTotalTime() const { return m_total_time; }
  double GetConstructionTime() const { return m_construction_time; }
  int GetNumQueries() const { return m_num_queries; }
  
  
/*

The following methods are not in the base class because we want 
them templated & therefore they cannot be virtual.  However, 
these MUST be implemeted for the code to properly compile.  
The PMPL_container_base will be used to "dispatch" calls to these
functions.

.....

*/
protected:
  double m_total_time, m_query_time, m_construction_time;
  int m_num_queries;
  Clock_Elapsed m_clock_total, m_clock_query, m_clock_cons;
  void StartTotalTime(){  
    m_clock_total.ClearClock();
    m_clock_total.StartClock("");    
  }
  
  void EndTotalTime(){
    m_clock_total.StopClock();
    m_total_time += m_clock_total.GetClock_SEC();
  }
  
  void StartQueryTime(){
    m_clock_query.ClearClock();
    m_clock_query.StartClock(""); 
  }
  
  void EndQueryTime(){
    m_clock_query.StopClock();
    m_query_time += m_clock_query.GetClock_SEC();
  }

  void StartConstructionTime(){  
    m_clock_cons.ClearClock();
    m_clock_cons.StartClock("");    
  }
  
  void EndConstructionTime(){
    m_clock_cons.StopClock();
    m_construction_time += m_clock_cons.GetClock_SEC();
  }
  
  void IncrementNumQueries() { ++m_num_queries; }

};

#endif //end #ifndef _NEIGHBORHOOD_FINDER_METHOD_H_
