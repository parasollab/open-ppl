#ifndef PMPL_PATH_H_
#define PMPL_PATH_H_

#include <algorithm>

#include "Behaviors/Agents/BatteryBreak.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/PMPLExceptions.h"


////////////////////////////////////////////////////////////////////////////////
/// A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PathType final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename RoadmapType::VID        VID;

    ///@}
    ///@name Construction
    ///@{

    /// Construct an empty path.
    /// @param _r The roadmap used by this path.
    PathType(RoadmapType* const _r = nullptr);

    ///@}
    ///@name Path Interface
    ///@{

    /// Get the robot which travels this path.
    Robot* GetRobot() const noexcept;

    /// Get the roadmap used by this path.
    RoadmapType* GetRoadmap() const noexcept;

    /// Get the number of cfgs in the path.
    size_t Size() const noexcept;

    /// Check if the path is empty.
    bool Empty() const noexcept;

    /// Get the total edge weight.
    double Length() const;

    /// Get the VIDs in the path.
    const std::vector<VID>& VIDs() const noexcept;

    /// Get a copy of the Cfgs in the path.
    /// @warning If the cfgs in the roadmap are later altered (i.e., if the DOF
    ///          values or labels are edited), this copy will be out-of-date.
    const std::vector<CfgType>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param _lib The planning library to use.
    /// @param _lp  The local planner label to use when connecting cfgs.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<CfgType> FullCfgs(MPLibrary* const _lib,
        const string& _lp = "") const;

    /// Find the furthest place and time in a path that an agent can travel to
    /// before being required to return to a charger.
    /// @params _batteryLevel Current battery level of the agent.
    /// @params _rate Rate at which battery levels decrease.
    /// @params _threshold Lowest level battery can reach before charging.
    /// @params _currentTime current time for the path to start calculating
    ///         from.
    /// @params _timeRes Resolution of a single timestep.
    template <typename MPLibrary>
    BatteryBreak FindBatteryBreak(double _batteryLevel, double _rate,
        double _theshold, double _currentTime, double _timeRes,
        MPLibrary* const _lib);

    /// Append another path to the end of this one.
    /// @param _p The path to append.
    PathType& operator+=(const PathType& _p);

    /// Add another path to the end of this one and return the result.
    /// @param _p The path to add.
    PathType operator+(const PathType& _p) const;

    /// Append a new set of VIDs to the end of this path.
    /// @param _vids The VIDs to append.
    PathType& operator+=(const std::vector<VID>& _vids);

    /// Add a new set of VIDs to the end of this path and return the result.
    /// @param _vids The VIDs to add.
    PathType operator+(const std::vector<VID>& _vids) const;

    /// Copy assignment operator.
    PathType& operator=(const PathType& _p);

    /// Clear all data in the path.
    void Clear();

    /// Clear cached data, but leave the VIDs.
    void FlushCache();

    ///@}

  private:

    ///@name Helpers
    ///@{

    void AssertSameMap(const PathType& _p) const;

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* const m_roadmap;       ///< The roadmap.
    std::vector<VID> m_vids;            ///< The vids of the path configurations.

    std::vector<CfgType> m_cfgs;        ///< The path configurations.
    mutable bool m_cfgsCached{false};   ///< Are the current cfgs correct?

    double m_length{0};                 ///< The path length.
    mutable bool m_lengthCached{false}; ///< Is the current path length correct?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
PathType<MPTraits>::
PathType(RoadmapType* const _r) : m_roadmap(_r) { }

/*------------------------------ Path Interface ------------------------------*/

template <typename MPTraits>
Robot*
PathType<MPTraits>::
GetRobot() const noexcept {
  return m_roadmap->GetRobot();
}


template <typename MPTraits>
typename MPTraits::RoadmapType*
PathType<MPTraits>::
GetRoadmap() const noexcept {
  return m_roadmap;
}


template <typename MPTraits>
size_t
PathType<MPTraits>::
Size() const noexcept {
  return m_vids.size();
}


template <typename MPTraits>
bool
PathType<MPTraits>::
Empty() const noexcept {
  return m_vids.empty();
}


template <typename MPTraits>
double
PathType<MPTraits>::
Length() const {
  if(!m_lengthCached) {
    double& length = const_cast<double&>(m_length);
    length = 0;
    for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
      if(*start == *(start + 1))
        continue;  // Skip repeated vertices.
      const auto& edge = m_roadmap->GetEdge(*start, *(start+1));
      length += edge.GetWeight();
    }
    m_lengthCached = true;
  }
  return m_length;
}


template <typename MPTraits>
const std::vector<typename PathType<MPTraits>::VID>&
PathType<MPTraits>::
VIDs() const noexcept {
  return m_vids;
}


template <typename MPTraits>
const std::vector<typename MPTraits::CfgType>&
PathType<MPTraits>::
Cfgs() const {
  if(!m_cfgsCached) {
    std::vector<CfgType>& cfgs = const_cast<std::vector<CfgType>&>(m_cfgs);
    cfgs.clear();
    cfgs.reserve(m_vids.size());
    for(const auto& vid : m_vids)
      cfgs.push_back(m_roadmap->GetVertex(vid));
    m_cfgsCached = true;
  }
  return m_cfgs;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::CfgType>
PathType<MPTraits>::
FullCfgs(MPLibrary* const _lib, const string& _lp) const {
  if(m_vids.empty())
    return std::vector<CfgType>();
  // Inserting first vertex cfg
  std::vector<CfgType> out = {m_roadmap->GetVertex(m_vids.front())};

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Inserting intermediates between vertices
    std::vector<CfgType> edge = _lib->ReconstructEdge(m_roadmap, 
      *it, *(it+1));
    out.insert(out.end(), edge.begin(), edge.end());
    //Inserting next vertex cfg
    out.push_back(m_roadmap->GetVertex(*(it+1)));
  }
  return out;
}


template <typename MPTraits>
template <typename MPLibrary>
BatteryBreak
PathType<MPTraits>::
FindBatteryBreak(double _batteryLevel, double _rate, double _threshold,
                 double _currentTime, double _timeRes, MPLibrary* const _lib){
  std::vector<CfgType> fullPath = FullCfgs(_lib);
  /*std::cout << "Battery Level: " << _batteryLevel << "\nRate: " << _rate <<
            "\nThreshold: " << _threshold << std::endl;
  std::cout << "Printing full path" << std::endl;
  for(auto cfg : fullPath){
    std::cout << cfg << std::endl;
  }*/
  /*
  //TODO: Figure out if the abstracted path will ever be necessary
  std::cout << "Printing abstracted path" << std::endl;
  std::vector<CfgType> path = Cfgs();
  for(auto cfg : path){
    std::cout << cfg << std::endl;
  }*/
  size_t cfgIt = 0; //keeps track of last path cfg reached before battery break
  for(auto cfg : fullPath){
    _batteryLevel -= _rate;
    //std::cout << "BatteryLevel: " << _batteryLevel << std::endl;
    if(_batteryLevel <= _threshold)
      break;
    _currentTime += _timeRes;
    cfgIt++;
  }
  return BatteryBreak(fullPath[cfgIt], _currentTime);
}


template <typename MPTraits>
PathType<MPTraits>&
PathType<MPTraits>::
operator+=(const PathType& _p) {
  AssertSameMap(_p);
  return *this += _p.m_vids;
}


template <typename MPTraits>
PathType<MPTraits>
PathType<MPTraits>::
operator+(const PathType& _p) const {
  AssertSameMap(_p);
  return *this + _p.m_vids;
}


template <typename MPTraits>
PathType<MPTraits>&
PathType<MPTraits>::
operator+=(const std::vector<VID>& _vids) {
  if(_vids.size()) {
    std::copy(_vids.begin(), _vids.end(), back_inserter(m_vids));
    m_lengthCached = false;
    m_cfgsCached = false;
  }
  return *this;
}


template <typename MPTraits>
PathType<MPTraits>
PathType<MPTraits>::
operator+(const std::vector<VID>& _vids) const {
  PathType out(*this);
  out += _vids;
  return out;
}


template <typename MPTraits>
PathType<MPTraits>&
PathType<MPTraits>::
operator=(const PathType& _p) {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE, "Can't assign path from another roadmap");

  m_vids         = _p.m_vids;
  m_cfgs         = _p.m_cfgs;
  m_cfgsCached   = _p.m_cfgsCached;
  m_length       = _p.m_length;
  m_lengthCached = _p.m_lengthCached;

  return *this;
}


template <typename MPTraits>
void
PathType<MPTraits>::
Clear() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
  m_vids.clear();
}


template <typename MPTraits>
void
PathType<MPTraits>::
FlushCache() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
PathType<MPTraits>::
AssertSameMap(const PathType& _p) const {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE, "Can't add paths from different roadmaps!");
}

/*----------------------------------------------------------------------------*/

#endif
