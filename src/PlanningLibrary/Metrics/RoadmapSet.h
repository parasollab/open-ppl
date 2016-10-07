#ifndef ROADMAPSET_H
#define ROADMAPSET_H

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RoadmapSet {
  public:

    typedef typename MPTraits::MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::VPI Iterator;
    typedef typename GraphType::CVPI ConstIterator;

    RoadmapSet() {}

    RoadmapSet(const RoadmapType& _roadmap) :
      m_roadmap(_roadmap) {}

    RoadmapSet(XMLNode& _node) {
      string filename = _node.Read("filename", true, "", "filename containing witness samples");
      m_roadmap.Read(filename.c_str());
    }

    size_t size() const {
      //returns the number of nodes in m_roadmap
      return m_roadmap.GetGraph()->get_num_vertices();
    }

    Iterator begin() {
      //returns a cfg iterator from m_roadmap begin
      return m_roadmap.GetGraph()->begin();
    }

    Iterator end() {
      //returns a cfg iterator from m_roadmap end
      return m_roadmap.GetGraph()->end();
    }

    ConstIterator begin() const {
      //returns a const cfg iterator from m_roadmap begin
      return m_roadmap.GetGraph()->begin();
    }

    ConstIterator end() const {
      //returns a const cfg iterator from m_roadmap end
      return m_roadmap.GetGraph()->end();
    }

    static string GetName() { return "RoadmapSet"; }

  private:

    RoadmapType m_roadmap;
};

#endif
