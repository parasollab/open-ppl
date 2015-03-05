/////////////////////////////////////////////////////////////////////
/**@file RoadmapVCS.h
  *
  * General Description
  *    This class represents a set of events concerning adding,
  *    removing, and editing RoadmapGraph information.
  *
  * @date 4/17/08
  * @author Bryan Boyd
  */
/////////////////////////////////////////////////////////////////////

#ifndef RoadmapVCS_h
#define RoadmapVCS_h

#include "RoadmapChangeEvent.h"

#include <vector>
//#include <pair>

using namespace std;

template<typename CFG, typename WEIGHT>
class RoadmapVCS {

  public:
    typedef RoadmapChangeEvent<CFG, WEIGHT> ChangeEvent;
    typedef vector< pair<int, ChangeEvent> > ChangeVector;
    typedef typename ChangeVector::const_iterator cce_iter; // constant event iterator

    RoadmapVCS();

    cce_iter begin();
    cce_iter end();
    cce_iter iter_at(int version_number);

    void addEvent(ChangeEvent);

    int get_version_number();

  protected:
    // NOTE: not really using the first int in pair for now...
    vector< pair<int, ChangeEvent > > change_list;
    int version_number;
};

template<typename CFG, typename WEIGHT>
RoadmapVCS<CFG, WEIGHT>::
RoadmapVCS()
{
  version_number = -1;
}

template<typename CFG, typename WEIGHT>
void
RoadmapVCS<CFG, WEIGHT>::
addEvent(ChangeEvent _event)
{
  version_number++;

  pair<int, ChangeEvent> temp(version_number, _event);
  change_list.push_back(temp);

  //cout << "Event #" << version_number << endl;
}

template<typename CFG, typename WEIGHT>
typename RoadmapVCS<CFG, WEIGHT>::cce_iter
RoadmapVCS<CFG, WEIGHT>::
begin()
{
  return change_list.begin();
}

template<typename CFG, typename WEIGHT>
typename RoadmapVCS<CFG, WEIGHT>::cce_iter
RoadmapVCS<CFG, WEIGHT>::
end()
{
  return change_list.end();
}

template<typename CFG, typename WEIGHT>
typename RoadmapVCS<CFG, WEIGHT>::cce_iter
RoadmapVCS<CFG, WEIGHT>::
iter_at(int _version_number)
{
  return change_list.begin() + _version_number;
}

template<typename CFG, typename WEIGHT>
int
RoadmapVCS<CFG, WEIGHT>::
get_version_number()
{
  return version_number;
}

#endif
