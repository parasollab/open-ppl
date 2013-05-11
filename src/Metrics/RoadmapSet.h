#ifndef ROADMAPSET_H
#define ROADMAPSET_H

template<class MPTraits>
class RoadmapSet {
  public:

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::MPProblemType::RoadmapType RoadmapType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef RoadmapGraph<CfgType, WeightType> GraphType;
    typedef typename GraphType::VPI iterator;
    typedef typename GraphType::CVPI const_iterator;

    RoadmapSet() {}

    RoadmapSet(shared_ptr<RoadmapType> _roadmap) :
      m_roadmap(_roadmap) {}

    RoadmapSet(MPProblemType* _problem, XMLNodeReader& _node) {
      //reads in m_roadmap from a filename if supplied
      //otherwise points to the MPProblem roadmap
      string filename = _node.stringXMLParameter("filename", false, "", "filename containing witness samples");
      if (filename == "")
        m_roadmap = shared_ptr<RoadmapType>(_problem->GetRoadmap());
      else {
        m_roadmap = shared_ptr<RoadmapType>(new RoadmapType());
        m_roadmap->Read(filename.c_str());
      }
    }

    size_t size() const {
      //returns the number of nodes in m_roadmap
      return m_roadmap->GetGraph()->get_num_vertices();
    }

    iterator begin() {
      //returns a cfg iterator from m_roadmap begin
      return iterator(m_roadmap->GetGraph()->begin());
    }

    iterator end() {
      //returns a cfg iterator from m_roadmap end
      return iterator(m_roadmap->GetGraph()->end());
    }

    const_iterator begin() const {
      //returns a const cfg iterator from m_roadmap begin
      return const_iterator(m_roadmap->GetGraph()->begin());
    }

    const_iterator end() const {
      //returns a const cfg iterator from m_roadmap end
      return const_iterator(m_roadmap->GetGraph()->end());
    }

    static string GetName() { return "RoadmapSet"; }

  private:

    shared_ptr<RoadmapType> m_roadmap;
};

#endif
