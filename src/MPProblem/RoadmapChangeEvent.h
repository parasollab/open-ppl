#ifndef RoadmapChangeEvent_h
#define RoadmapChangeEvent_h

#include <iostream>
#include <memory>
#include <vector>
using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Roadmap
/// @brief Stores types events concerning roadmap changes.
/// @tparam CFG Configuration type
/// @tparam WEIGHT Weight type
///
/// This class represents a set of events concerning adding, removing, and
/// editing RoadmapGraph information.
////////////////////////////////////////////////////////////////////////////////
template<typename CFG, typename WEIGHT>
class RoadmapChangeEvent {

  public:
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CFG, WEIGHT>::vertex_descriptor VID;
    enum ChangeType { ADD_VERTEX, REMOVE_VERTEX };

    class BaseChangeEvent {
      public:
        virtual ~BaseChangeEvent() {}
        virtual ChangeType GetType() const = 0;

      protected:
        ChangeType item_type;
    };

    class AddVertexEvent : public BaseChangeEvent {
      public:
        AddVertexEvent(const CFG& _cfg, VID _vid) {
          cfg = _cfg;
          vid = _vid;
          this->item_type = ADD_VERTEX;
        }
        virtual ~AddVertexEvent() {}

        ChangeType GetType() const { return this->item_type; }
        const CFG& GetCFG() const { return cfg; }
        VID GetVID() const { return vid; }

      protected:
        CFG cfg;
        VID vid;
        //ChangeType item_type;
    };

    class RemoveVertexEvent : public BaseChangeEvent
    {
      public:
        RemoveVertexEvent(VID _vid)
        {
          vid = _vid;
          this->item_type = REMOVE_VERTEX;
        }

        ChangeType GetType() const { return this->item_type; }
        VID GetVID() const { return vid; }

      protected:
        int vid;
        //ChangeType item_type;
    };

    // create a constructor for each event...

    // default constructor
    RoadmapChangeEvent();

    // AddVertexEvent constructor
    RoadmapChangeEvent(ChangeType type, const CFG& _cfg, VID _vid);

    // RemoveVertexEvent constructor
    RoadmapChangeEvent(ChangeType type, VID _vid);

    //destructor
    ~RoadmapChangeEvent();

    bool IsTypeAddVertex() const;
    bool IsTypeRemoveVertex() const;

    const BaseChangeEvent* GetEvent() const;
    const AddVertexEvent* GetAddVertexEvent() const;
    const RemoveVertexEvent* GetRemoveVertexEvent() const;

  protected:
    shared_ptr<BaseChangeEvent> event;
};

template<typename CFG, typename WEIGHT>
RoadmapChangeEvent<CFG, WEIGHT>::RoadmapChangeEvent() {
}

template<typename CFG, typename WEIGHT>
RoadmapChangeEvent<CFG, WEIGHT>::RoadmapChangeEvent(ChangeType type, const CFG& _cfg, VID _vid) {
  if (type == ADD_VERTEX)
    event = shared_ptr<BaseChangeEvent>(new AddVertexEvent(_cfg, _vid));
}

template<typename CFG, typename WEIGHT>
RoadmapChangeEvent<CFG, WEIGHT>::RoadmapChangeEvent(ChangeType type, VID _vid) {
  if (type == REMOVE_VERTEX)
    event = shared_ptr<BaseChangeEvent>(new RemoveVertexEvent(_vid));
}

template<typename CFG, typename WEIGHT>
RoadmapChangeEvent<CFG, WEIGHT>::~RoadmapChangeEvent() {}

template<typename CFG, typename WEIGHT>
bool
RoadmapChangeEvent<CFG, WEIGHT>::
IsTypeAddVertex() const {
  if (event->GetType() == ADD_VERTEX)
    return true;
  else
    return false;
}

template<typename CFG, typename WEIGHT>
bool
RoadmapChangeEvent<CFG, WEIGHT>::IsTypeRemoveVertex() const {
  if (event->GetType() == REMOVE_VERTEX)
    return true;
  else
    return false;
}

template<typename CFG, typename WEIGHT>
const typename RoadmapChangeEvent<CFG, WEIGHT>::BaseChangeEvent*
RoadmapChangeEvent<CFG, WEIGHT>::
GetEvent() const {
  return event;
}

template<typename CFG, typename WEIGHT>
const typename RoadmapChangeEvent<CFG, WEIGHT>::AddVertexEvent*
RoadmapChangeEvent<CFG, WEIGHT>::
GetAddVertexEvent() const {
  if (this->IsTypeAddVertex())
    return (AddVertexEvent*)event;
  else {
    cout << "Could not cast to AddVertexEvent... invalid type" << endl;
    return NULL;
  }
}

template<typename CFG, typename WEIGHT>
const typename RoadmapChangeEvent<CFG, WEIGHT>::RemoveVertexEvent*
RoadmapChangeEvent<CFG, WEIGHT>::
GetRemoveVertexEvent() const {
  if (this->IsTypeRemoveVertex())
    return (RemoveVertexEvent*)event;
  else {
    cout << "Could not cast to RemoveVertexEvent... invalid type" << endl;
    return NULL;
  }
}

#endif
