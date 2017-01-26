#ifndef ROADMAP_SET_H
#define ROADMAP_SET_H

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// @TODO This class is an artifact of old code. If anyone actually wants to use
///       it, please take responsibility for re-designing it intelligently.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RoadmapSet final {

  public:

    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::VPI         Iterator;
    typedef typename GraphType::CVPI        ConstIterator;

    RoadmapSet() = default;

    RoadmapSet(XMLNode& _node) {
      m_filename = _node.Read("filename", true, "", "filename containing "
          "witness samples");
    }

    ~RoadmapSet() noexcept {
      delete m_roadmap;
    }

    /// Returns the number of nodes in m_roadmap
    size_t size() const {
      CheckMap();
      return m_roadmap->GetGraph()->get_num_vertices();
    }

    /// Returns a cfg iterator from m_roadmap begin
    Iterator begin() {
      CheckMap();
      return m_roadmap->GetGraph()->begin();
    }

    /// Returns a cfg iterator from m_roadmap end
    Iterator end() {
      CheckMap();
      return m_roadmap->GetGraph()->end();
    }

    /// Returns a const cfg iterator from m_roadmap begin
    ConstIterator begin() const {
      CheckMap();
      return m_roadmap->GetGraph()->begin();
    }

    /// Returns a const cfg iterator from m_roadmap end
    ConstIterator end() const {
      CheckMap();
      return m_roadmap->GetGraph()->end();
    }

    static string GetName() { return "RoadmapSet"; }

    void CheckMap() const {
      if(!m_roadmap)
        throw RunTimeException(WHERE, "Need to call ReadMap first.");
    }

    void ReadMap(Robot* const _r) {
      if(m_roadmap)
        return;
      m_roadmap = new RoadmapType(_r);
      m_roadmap->Read(m_filename);
    }

  private:

    RoadmapType* m_roadmap{nullptr};
    std::string m_filename;
};

#endif
