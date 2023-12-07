#include "GroupPath.h"

#include <algorithm>
#include <vector>

/*------------------------------- Construction -------------------------------*/

GroupPath::
GroupPath(GroupRoadmapType* const _r) : m_roadmap(_r) { }

/*------------------------------ Path Interface ------------------------------*/

typename GroupPath::GroupRoadmapType*
GroupPath::
GetRoadmap() const noexcept {
  return m_roadmap;
}


size_t
GroupPath::
Size() const noexcept {
  return m_vids.size();
}


bool
GroupPath::
Empty() const noexcept {
  return m_vids.empty();
}


double
GroupPath::
Length() const {
  // If the length is cached, we don't need to recompute.
  if(m_lengthCached)
    return m_length;
  m_lengthCached = true;

  // Recompute the length by summing the edge weights.
  m_length = 0;
  for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
    // Skip repeated vertices.
    /// @todo This will be an error if we allow self-edges.
    if(*start == *(start + 1))
      continue;

    // Add this edge's weight to the sum.
    const auto& edge = m_roadmap->GetEdge(*start, *(start + 1));
    m_length += edge.GetWeight();
  }

  return m_length;
}


size_t
GroupPath::
TimeSteps() const {
  // If the length is cached, we don't need to recompute.
  if(m_timestepsCached)
    return m_timesteps;
  m_timestepsCached = true;

  // Recompute the length by summing the edge weights.
  m_timesteps = 0;
  for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
    // Skip repeated vertices.
    /// @todo This will be an error if we allow self-edges.
    if(*start == *(start + 1))
      continue;

    // Add this edge's weight to the sum.
    const auto& edge = m_roadmap->GetEdge(*start, *(start + 1));
    m_timesteps += edge.GetTimeSteps();
  }

  for(auto w : m_waitingTimesteps) {
    m_timesteps += w;
  }

  return m_timesteps;
}


const std::vector<typename GroupPath::VID>&
GroupPath::
VIDs() const noexcept {
  return m_vids;
}


const std::vector<typename GroupPath::GroupCfgType>&
GroupPath::
Cfgs() const {
  // If the cfgs are cached, we don't need to recompute.
  if(m_cfgsCached)
    return m_cfgs;
  m_cfgsCached = true;

  m_cfgs.clear();
  m_cfgs.reserve(m_vids.size());
  for(const auto& vid : m_vids)
    m_cfgs.push_back(m_roadmap->GetVertex(vid));

  return m_cfgs;
}


GroupPath&
GroupPath::
operator+=(const GroupPath& _p) {
  AssertSameMap(_p);
  return *this += _p.m_vids;
}


GroupPath
GroupPath::
operator+(const GroupPath& _p) const {
  AssertSameMap(_p);
  return *this + _p.m_vids;
}


GroupPath&
GroupPath::
operator+=(const std::vector<VID>& _vids) {
  if(_vids.size()) {
    FlushCache();
    std::copy(_vids.begin(), _vids.end(), std::back_inserter(m_vids));
  }
  return *this;
}


GroupPath
GroupPath::
operator+(const std::vector<VID>& _vids) const {
  GroupPath out(*this);
  out += _vids;
  return out;
}


GroupPath&
GroupPath::
operator=(const GroupPath& _p) {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't assign path from another roadmap";

  m_vids         = _p.m_vids;
  m_cfgs         = _p.m_cfgs;
  m_cfgsCached   = _p.m_cfgsCached;
  m_length       = _p.m_length;
  m_lengthCached = _p.m_lengthCached;
  m_timestepsCached = _p.m_timestepsCached;
  m_waitingTimesteps = _p.m_waitingTimesteps;

  return *this;
}


void
GroupPath::
Clear() {
  FlushCache();
  m_vids.clear();
  m_waitingTimesteps.clear();
}


void
GroupPath::
FlushCache() {
  m_lengthCached = false;
  m_timestepsCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
}


void
GroupPath::
SetWaitTimes(std::vector<size_t> _waitTimes) {
  m_waitingTimesteps = _waitTimes;

  // Check to make sure that size matches path length
  if(m_waitingTimesteps.size() != m_vids.size()) {
    throw RunTimeException(WHERE) << "Size of timestep vector does not "
                                  << "match path length";
  }
}


std::vector<size_t>
GroupPath::
GetWaitTimes() {
  return m_waitingTimesteps;
}


std::pair<size_t,size_t>
GroupPath::
GetEdgeAtTimestep(size_t _timestep) {
 
  size_t step = 0;
  for(size_t i = 0; i + 1 < m_vids.size(); i++) {

    if(!m_waitingTimesteps.empty())
      step += m_waitingTimesteps[i];
    
    if(_timestep <= step)
      return std::make_pair(m_vids[i],m_vids[i]);

    auto duration = m_roadmap->GetEdge(m_vids[i],m_vids[i+1]).GetTimeSteps();

    step += duration;

    if(_timestep <= step)
      return std::make_pair(m_vids[i],m_vids[i+1]);
  }

  return std::make_pair(m_vids.back(),m_vids.back());
}

/*--------------------------------- Helpers ----------------------------------*/

void
GroupPath::
AssertSameMap(const GroupPath& _p) const {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't add paths from different roadmaps "
                                  << "(source = " << _p.m_roadmap << ","
                                  << " target = " << m_roadmap << ").";
}
