#include "Path.h"

#include "MPLibrary/MPLibrary.h"

#include <algorithm>
#include <vector>

/*------------------------------- Construction -------------------------------*/

Path::
Path(RoadmapType* const _r) : m_roadmap(_r) { }

/*------------------------------ Path Interface ------------------------------*/

Robot*
Path::
GetRobot() const noexcept {
  return m_roadmap->GetRobot();
}


typename Path::RoadmapType*
Path::
GetRoadmap() const noexcept {
  return m_roadmap;
}


size_t
Path::
Size() const noexcept {
  return m_vids.size();
}


bool
Path::
Empty() const noexcept {
  return m_vids.empty();
}


double
Path::
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


const std::vector<typename Path::VID>&
Path::
VIDs() const noexcept {
  return m_vids;
}


const std::pair<std::vector<typename Path::VID>,std::vector<size_t>>
Path::
VIDsWaiting() const noexcept {
	return std::make_pair(m_vids,m_waitingTimesteps);
}


const std::vector<Cfg>&
Path::
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


size_t
Path::
TimeSteps() const {

  //if(m_timestepsCached)
  //  return m_timesteps;

  m_timestepsCached = true;

  if(m_vids.empty())
    return 0;

	m_timesteps = 0;

  for(size_t i = 0; i < m_vids.size()-1; i++) {
		if(m_vids[i] == m_vids[i+1]) {
			m_timesteps++;
			continue;
		}

    if(!m_waitingTimesteps.empty())
      m_timesteps += m_waitingTimesteps[i];

		auto edge = m_roadmap->GetEdge(m_vids[i], m_vids[i+1]);
		m_timesteps += edge.GetTimeSteps();
  }

  if(!m_waitingTimesteps.empty())
    m_timesteps += m_waitingTimesteps.back();

  return m_timesteps;
}


void
Path::
SetTimeSteps(size_t _timesteps) {
  m_timestepsCached = true;
  m_timesteps = _timesteps;
}


void
Path::
ResetTimeSteps() {
  m_timestepsCached = false;
}


Path&
Path::
operator+=(const Path& _p) {
  AssertSameMap(_p);
  return *this += _p.m_vids;
}


Path
Path::
operator+(const Path& _p) const {
  AssertSameMap(_p);
  return *this + _p.m_vids;
}


Path&
Path::
operator+=(const std::vector<VID>& _vids) {
  if(_vids.size()) {
    FlushCache();
    std::copy(_vids.begin(), _vids.end(), std::back_inserter(m_vids));
  }
  return *this;
}


Path
Path::
operator+(const std::vector<VID>& _vids) const {
  Path out(*this);
  out += _vids;
  return out;
}


Path&
Path::
operator=(const Path& _p) {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't assign path from another roadmap";

  m_vids             = _p.m_vids;
  m_waitingTimesteps = _p.m_waitingTimesteps;
  m_cfgs             = _p.m_cfgs;
  m_cfgsCached       = _p.m_cfgsCached;
  m_length           = _p.m_length;
  m_lengthCached     = _p.m_lengthCached;

  return *this;
}


void
Path::
Clear() {
  FlushCache();
  m_vids.clear();
  m_waitingTimesteps.clear();
}


void
Path::
FlushCache() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
}


void
Path::
SetFinalWaitTimeSteps(const size_t& _timeSteps) {
	m_finalWaitTimeSteps = _timeSteps;
}


const size_t
Path::
GetFinalWaitTimeSteps() const {
	return m_finalWaitTimeSteps;
}


void
Path::
SetWaitTimes(std::vector<size_t> _waitTimes) {
  m_waitingTimesteps = _waitTimes;

  // Check to make sure that size matches path length
  if(m_waitingTimesteps.size() != m_vids.size()) {
    throw RunTimeException(WHERE) << "Size of timestep vector does not "
                                  << "match path length";
  }
}


std::vector<size_t>
Path::
GetWaitTimes() {
  return m_waitingTimesteps;
}


std::pair<std::pair<size_t,size_t>,std::pair<size_t,size_t>>
Path::
GetEdgeAtTimestep(size_t _timestep) {

  size_t step = 0;

  for(size_t i = 0; i + 1 < m_vids.size(); i++) {
    
    if(!m_waitingTimesteps.empty())
      step += m_waitingTimesteps[i];

    if(_timestep <= step) {
      auto vertex = std::make_pair(m_vids[i],m_vids[i]);
      auto interval = std::make_pair(step-m_waitingTimesteps[i],step);
      return std::make_pair(vertex,interval);
    }

    auto duration = m_roadmap->GetEdge(m_vids[i],m_vids[i+1]).GetTimeSteps();

    step += duration;

    if(_timestep <= step) { 
      auto edge = std::make_pair(m_vids[i],m_vids[i+1]);
      auto interval = std::make_pair(step-duration,step);
      return std::make_pair(edge,interval);
    }
  }

  auto vertex = std::make_pair(m_vids.back(),m_vids.back());
  auto interval = std::make_pair(step,std::numeric_limits<size_t>::infinity());
  return std::make_pair(vertex,interval);
}


const std::vector<Cfg>
Path::
FullCfgs(MPLibrary* const _lib) const {
  if(m_vids.empty())
    return std::vector<Cfg>();

  // Insert the first vertex.
  std::vector<Cfg> out = {m_roadmap->GetVertex(m_vids.front())};

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Insert intermediates between vertices.
    std::vector<Cfg> edge = _lib->ReconstructEdge(m_roadmap, *it, *(it+1));
    out.insert(out.end(), edge.begin(), edge.end());

    //Insert the next vertex.
    out.push_back(m_roadmap->GetVertex(*(it+1)));
  }
	auto last = out.back();
	for(size_t i = 0; i < m_finalWaitTimeSteps; i++) {
		out.push_back(last);
	}
  return out;
}


const std::vector<Cfg>
Path::
FullCfgsWithWait(MPLibrary* const _lib) const {
  if(m_vids.empty())
    return std::vector<Cfg>();

  if(m_waitingTimesteps.empty())
    return FullCfgs(_lib);

  // Insert the first vertex.
  size_t vid = m_vids.front();
  Cfg vertex = m_roadmap->GetVertex(vid);
  std::vector<Cfg> out = {vertex};

  // Insert first vertex however many timesteps it waits
  //for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
  for(size_t i = 0; i < m_waitingTimesteps[0]; ++i) {
    out.push_back(vertex);
  }

  auto cnt = 1;
  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    // Insert intermediates between vertices.

    std::vector<Cfg> edge = _lib->ReconstructEdge(m_roadmap, *it, *(it+1));
    out.insert(out.end(), edge.begin(), edge.end());

    // Insert the next vertex.
    vid = *(it+1);
    vertex = m_roadmap->GetVertex(vid);
    out.push_back(vertex);
    for(size_t i = 0; i < m_waitingTimesteps[cnt]; ++i) {
      out.push_back(vertex);
    }

    cnt++;
  }

  auto last = out.back();
  for(size_t i = 0; i < m_finalWaitTimeSteps; i++) {
    out.push_back(last);
  }
  return out;
}

/*--------------------------------- Helpers ----------------------------------*/

inline
void
Path::
AssertSameMap(const Path& _p) const noexcept {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't add paths from different roadmaps "
                                  << "(source = " << _p.m_roadmap << ","
                                  << " target = " << m_roadmap << ").";
}

/*----------------------------------------------------------------------------*/

