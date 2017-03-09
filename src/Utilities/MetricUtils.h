#ifndef METRIC_UTILS_H_
#define METRIC_UTILS_H_

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#ifndef _PARALLEL
#include <containers/sequential/graph/algorithms/connected_components.h>
#endif


////////////////////////////////////////////////////////////////////////////////
/// This class is used to measure the running time between @c StartClock and
/// @c StopClock. Client side could provide clock name.
///
/// @ingroup MetricUtils
////////////////////////////////////////////////////////////////////////////////
class ClockClass {

  public:

    ClockClass();

    /// Set the clock name.
    /// @param _name The name to use.
    void SetName(const std::string& _name);

    /// Reset the clock.
    void ClearClock();

    /// Start the clock and the name is identity of this clock.
    void StartClock();

    /// Stop the clock and calculate the total running time.
    void StopClock();

    /// Call StopClock and PrintClock.
    void StopPrintClock(std::ostream& _os);

    /// Print clock name and time in seconds to an outstream.
    /// @param _os The outstream to print to.
    void PrintClock(std::ostream& _os);

    /// Get the recorded time in seconds.
    double GetSeconds();

    /// Get the recorded time in microseconds.
    int GetUSeconds();

  private:

    ///@name Internal State
    ///@{

    int m_sTime, m_uTime;
    int m_suTime, m_uuTime;

    std::string m_clockName;

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// The StatClass is a storage hub of all statistics to be tracked in PMPL,
/// including but not limited to timing, success/fail attempts,
/// collision detection calls.
///
/// @ingroup MetricUtils
////////////////////////////////////////////////////////////////////////////////
class StatClass final {

  public:

    StatClass();

    void ClearStats();

    int IncNumCollDetCalls(const std::string& _cdName,
        const std::string& _callName);
    size_t GetIsCollTotal();
    void IncCfgIsColl(const std::string& _callName);

    int IncLPConnections(const std::string& _lpName , int _incr = 1);
    int IncLPAttempts(const std::string& _lpName, int _incr = 1);
    int IncLPCollDetCalls(const std::string& _lpName, int _incr = 1);

    template<class RoadmapType>
    void PrintAllStats(std::ostream& _os, RoadmapType* _rmap);

    void IncNodesGenerated(const std::string& _samplerName, size_t _incr = 1);
    void IncNodesAttempted(const std::string& _samplerName, size_t _incr = 1);

    //Clock Accessors
    void ClearClock(const std::string& _name);
    void StartClock(const std::string& _name);
    void StopClock(const std::string& _name);
    void StopPrintClock(const std::string& _name, std::ostream& _os);
    void PrintClock(const std::string& _name, std::ostream& _os);
    double GetSeconds(const std::string& _name);
    int GetUSeconds(const std::string& _name);

    // Statistics Accessors/Modifiers
    double GetStat(const std::string& _s);
    void SetStat(const std::string& _s, const double _v);
    void IncStat(const std::string& _s, const double _v = 1);

    // Histories
    std::vector<double>& GetHistory(const std::string& _s);
    void AddToHistory(const std::string& _s, double _v);
    void SetAuxDest(const std::string& _s);

    //help
    template<class GraphType>
    void DisplayCCStats(std::ostream& _os, GraphType&);

    // m_lpInfo represents information about the Local Planners, referenced by
    // name
    // m_lpInfo.first is the name of the Local Planner
    // m_lpInfo.second.get<0>() is the # of LP attempts
    // m_lpInfo.second.get<1>() is the # of LP connections (successes)
    // m_lpInfo.second.get<2>() is the # of LP collision detection calls
    std::map<std::string, std::tuple<size_t, size_t, size_t> > m_lpInfo;
    std::map<std::string, size_t> m_collDetCountByName;

    std::map<std::string, ClockClass> m_clockMap;

    ///IsColl simply counts the number of times a Cfg is tested for Collision.
    ///\see Cfg::isCollision
    std::map<std::string, size_t> m_isCollByName;
    size_t m_isCollTotal;


    //m_samplerInfo represents sampler nodes attempted and generated
    //  map<string, pair<int, int> > represents a mapping between the sampler name
    //  and first the # of attempted samples, then the number of generated samples
    //  that is, pair.first = attempts, pair.second = generated (successful attempts)
    std::map<std::string, std::pair<size_t, size_t>> m_samplerInfo;

  private:

    ///@name Internal State
    ///@{

    std::map<std::string, size_t> m_numCollDetCalls;
    std::map<std::string, double> m_stats;
    std::map<std::string, std::vector<double>> m_histories;
    std::string m_auxFileDest;

    ///@}

};


template <typename RoadmapType>
void
StatClass::
PrintAllStats(std::ostream& _os, RoadmapType* _rmap) {
#ifndef _PARALLEL

  // Output roadmap statistics.
  _os << "Roadmap Statistics:\n"
      << "\n  Number of Nodes: " << _rmap->GetGraph()->get_num_vertices()
      << "\n  Number of Edges: " << _rmap->GetGraph()->get_num_edges()
      << std::endl;
  DisplayCCStats(_os, *_rmap->GetGraph());

  // Output sampler statistics.
  if(!m_samplerInfo.empty()) {
    size_t totalAttempts = 0, totalGenerated = 0;
    _os << "\n\n"
        << std::setw(60) << std::left  << "Sampler Statistics"
        << std::setw(10) << std::right << "Attempts"
        << std::setw(10) << std::right << "Successes"
        << "\n\n";

    for(const auto& info : m_samplerInfo) {
      _os << "  "
          << std::setw(58) << std::left  << info.first
          << std::setw(10) << std::right << info.second.first
          << std::setw(10) << std::right << info.second.second
          << std::endl;
      totalAttempts += info.second.first;
      totalGenerated += info.second.second;
    }

    if(totalAttempts > 0)
      _os << "  "
          << std::setw(58) << std::left << "Total"
          << std::setw(10) << std::right << totalAttempts
          << std::setw(10) << std::right << totalGenerated
          << "\n  Success Rate: "
          << std::setprecision(3) << totalGenerated * 100.0 / totalAttempts << "%"
          << std::endl;
  }

  // Output local planner statistics.
  if(!m_lpInfo.empty()) {
    _os << "\n\n"
        << std::setw(44) << std::left  << "Local Planner Statistics"
        << std::setw(12) << std::right << "Attempts"
        << std::setw(12) << std::right << "Connections"
        << std::setw(12) << std::right << "CD Calls"
        << "\n\n";

    for(const auto& info : m_lpInfo)
      _os << "  "
          << std::setw(42) << std::left  << info.first
          << std::setw(12) << std::right << std::get<0>(info.second)
          << std::setw(12) << std::right << std::get<1>(info.second)
          << std::setw(12) << std::right << std::get<2>(info.second)
          << std::endl;
  }

  // Ouput CD statistics.
  if(!m_isCollByName.empty()) {
    _os << "\n\n"
        << std::setw(70) << std::left  << "Collision Detection Calls"
        << std::setw(10) << std::right << "Count"
        << "\n\n";
    for(const auto& info : m_isCollByName)
      _os << "  "
          << std::setw(68) << std::left  << info.first
          << std::setw(10) << std::right << info.second
          << std::endl;
    _os << "  "
        << std::setw(68) << std::left  << "Total"
        << std::setw(10) << std::right << m_isCollTotal
        << std::endl;
  }

  // Output other statistics.
  if(!m_stats.empty()) {
    _os << "\n\n"
        << std::setw(70) << std::left  << "Other Statistics"
        << std::setw(10) << std::right << "Value"
        << "\n\n";
    for(const auto& stat : m_stats)
      _os << "  "
          << std::setw(68) << std::left  << stat.first
          << std::setw(10) << std::right << stat.second
          << std::endl;
  }

  // Output clocks.
  _os << "\n\n"
      << std::setw(66) << std::left  << "Clock Name"
      << std::setw(14) << std::right << "Time (Seconds)"
      << "\n\n";
  for(auto& clock : m_clockMap)
    _os << "  "
        << std::setw(68) << std::left << clock.first
        << std::setw(10) << std::right << clock.second.GetSeconds()
        << std::endl;

  // Write out history files.
  for(const auto& hist : m_histories) {
    std::ofstream ofs(m_auxFileDest + "." + hist.first + ".hist");
    for(const auto& item : hist.second)
      ofs << item << std::endl;
  }

#endif
}


template <class GraphType>
void
StatClass::
DisplayCCStats(std::ostream& _os, GraphType& _g)  {
  #ifndef _PARALLEL
  // Compute the CC information.
  typedef typename GraphType::vertex_descriptor VID;

  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  std::vector<std::pair<size_t, VID>> ccs;

  stapl::sequential::get_cc_stats(_g, cMap, ccs);

  size_t ccnum = 0;
  _os << "\n  There are " << ccs.size() << " connected components:";
  for(auto cc : ccs)
    _os << "\n    CC[" << ccnum++ << "]: " << cc.first
        << " (vid " << cc.second << ")";
  _os << std::endl;

  #else
    _os << "WARNING: CCs not computed, called sequential implementation of CC stats \n" ;
  #endif
}

#endif
