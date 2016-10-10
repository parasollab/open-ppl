#ifndef ROADMAP_VCS_H_
#define ROADMAP_VCS_H_

#include "RoadmapChangeEvent.h"

#include <vector>
using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Roadmap
/// @brief Roadmap event version tracker
///
/// This class is like a version control system for roadmaps. It aids in
/// tracking changes to the roadmap.
////////////////////////////////////////////////////////////////////////////////
template<typename GraphType>
class RoadmapVCS {
  public:
    typedef RoadmapChangeEvent<GraphType> Event;
    typedef pair<size_t, Event> ChangeEvent;
    typedef vector<ChangeEvent> ChangeEvents;
    typedef typename ChangeEvents::const_iterator const_iterator;

    RoadmapVCS() {}

    const_iterator begin() const {return m_changes.cbegin();}
    const_iterator end() const {return m_changes.cend();}
    const_iterator IteratorAt(size_t _versionNumber) const {return begin() + _versionNumber;}

    void AddEvent(Event&& _event) {m_changes.emplace_back(++m_versionNumber, _event);}

    size_t GetVersionNumber() const {return m_versionNumber;}

  private:
    ChangeEvents m_changes;
    size_t m_versionNumber{(size_t)-1};
};

#endif
