#ifndef _NEIGHBORHOOD_FINDER_METHOD_H_
#define _NEIGHBORHOOD_FINDER_METHOD_H_

#include <string>
#include <iostream>
#include "MPUtils.h"
#include "MPProblem.h"


class NeighborhoodFinderMethod : public MPBaseObject  {

public:
  NeighborhoodFinderMethod(shared_ptr<DistanceMetricMethod> dm, string _label="",  MPProblem* _problem = NULL);
  NeighborhoodFinderMethod(XMLNodeReader& _node, MPProblem* _problem);
  NeighborhoodFinderMethod();
  virtual ~NeighborhoodFinderMethod() {}
  virtual const std::string GetName () const = 0;
  virtual void PrintOptions(std::ostream& out_os) const = 0;
  
  double GetQueryTime() const { return m_query_time; }
  double GetTotalTime() const { return m_total_time; }
  double GetConstructionTime() const { return m_construction_time; }
  int GetNumQueries() const { return m_num_queries; }

  shared_ptr<DistanceMetricMethod> GetDMMethod() { return dmm; }
 
/*

The following methods are not in the base class because we want 
them templated & therefore they cannot be virtual.  However, 
these MUST be implemeted for the code to properly compile.  
The PMPL_container_base will be used to "dispatch" calls to these
functions.

.....

*/
private:
  void Init();
protected:
  double m_total_time, m_query_time, m_construction_time;
  int m_num_queries;

  void StartTotalTime();
  void EndTotalTime();
  void StartQueryTime();
  void EndQueryTime();
  void StartConstructionTime();
  void EndConstructionTime();

  void IncrementNumQueries() { ++m_num_queries; }

  shared_ptr<DistanceMetricMethod> dmm;
};


template <typename T, typename U>
class compare_second : public binary_function<const pair<T, U>, const pair<T, U>, bool>
{
 public:
  bool operator()(const pair<T, U>& _cc1, const pair<T, U>& _cc2) const
  {
    return _cc1.second < _cc2.second;
  }
};


#endif //end #ifndef _NEIGHBORHOOD_FINDER_METHOD_H_
