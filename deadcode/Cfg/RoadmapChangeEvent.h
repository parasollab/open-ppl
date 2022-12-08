#ifndef RoadmapChangeEvent_h
#define RoadmapChangeEvent_h

#include <memory>

#include "Utilities/PMPLExceptions.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Roadmap
/// @brief Stores types events concerning roadmap changes.
/// @tparam CfgType Configuration type
/// @tparam WeightType Weight type
///
/// This class represents a set of events concerning adding, removing, and
/// editing RoadmapGraph information.
////////////////////////////////////////////////////////////////////////////////
template<typename GraphType>
class RoadmapChangeEvent {

  public:
    typedef typename GraphType::vertex_property CfgType;
    typedef typename GraphType::edge_property WeightType;
    typedef typename GraphType::vertex_descriptor VID;

    enum class ChangeType { ADD_VERTEX, REMOVE_VERTEX };

    class BaseChangeEvent {
      public:
        virtual ~BaseChangeEvent() {}

        ChangeType GetType() const {return this->m_type;}

      protected:
        ChangeType m_type;
    };

    class AddVertexEvent : public BaseChangeEvent {
      public:
        AddVertexEvent(const CfgType& _cfg, VID _vid) {
          m_cfg = _cfg;
          m_vid = _vid;
          this->m_type = ChangeType::ADD_VERTEX;
        }

        const CfgType& GetCfg() const { return m_cfg; }
        VID GetVID() const { return m_vid; }

      protected:
        CfgType m_cfg;
        VID m_vid;
    };

    class RemoveVertexEvent : public BaseChangeEvent {
      public:
        RemoveVertexEvent(VID _vid) {
          m_vid = _vid;
          this->m_type = ChangeType::REMOVE_VERTEX;
        }

        VID GetVID() const { return m_vid; }

      protected:
        VID m_vid;
    };

    // AddVertexEvent constructor
    RoadmapChangeEvent(const CfgType& _cfg, VID _vid);

    // RemoveVertexEvent constructor
    RoadmapChangeEvent(VID _vid);

    bool IsTypeAddVertex() const {
      return m_event->GetType() == ChangeType::ADD_VERTEX;
    }
    bool IsTypeRemoveVertex() const {
      return m_event->GetType() == ChangeType::REMOVE_VERTEX;
    }

    shared_ptr<const AddVertexEvent> GetAddVertexEvent() const;
    shared_ptr<const RemoveVertexEvent> GetRemoveVertexEvent() const;

  protected:
    shared_ptr<BaseChangeEvent> m_event;
};

template<typename GraphType>
RoadmapChangeEvent<GraphType>::
RoadmapChangeEvent(const CfgType& _cfg, VID _vid) :
  m_event(new AddVertexEvent(_cfg, _vid)) {
  }

template<typename GraphType>
RoadmapChangeEvent<GraphType>::
RoadmapChangeEvent(VID _vid) :
  m_event(new RemoveVertexEvent(_vid)) {
  }

template<typename GraphType>
shared_ptr<const typename RoadmapChangeEvent<GraphType>::AddVertexEvent>
RoadmapChangeEvent<GraphType>::
GetAddVertexEvent() const {
  if(IsTypeAddVertex())
    return static_pointer_cast<const AddVertexEvent>(m_event);
  else
    throw RunTimeException(WHERE, "Cannot cast ChangeEvent to AddVertexEvent.");
}

template<typename GraphType>
shared_ptr<const typename RoadmapChangeEvent<GraphType>::RemoveVertexEvent>
RoadmapChangeEvent<GraphType>::
GetRemoveVertexEvent() const {
  if(IsTypeRemoveVertex())
    return static_pointer_cast<const RemoveVertexEvent>(m_event);
  else
    throw RunTimeException(WHERE, "Cannot cast ChangeEvent to RemoveVertexEvent.");
}

#endif
