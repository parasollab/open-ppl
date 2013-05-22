#ifndef ROADMAPSET_H
#define ROADMAPSET_H

template<class MPTraits>
class RoadmapSet {
  public:

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::MPProblemType::RoadmapType RoadmapType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::VPI Iterator;
    typedef typename GraphType::CVPI ConstIterator;

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

    Iterator begin() {
      //returns a cfg iterator from m_roadmap begin
      return m_roadmap->GetGraph()->begin();
    }

    Iterator end() {
      //returns a cfg iterator from m_roadmap end
      return m_roadmap->GetGraph()->end();
    }

    ConstIterator begin() const {
      //returns a const cfg iterator from m_roadmap begin
      return m_roadmap->GetGraph()->begin();
    }

    ConstIterator end() const {
      //returns a const cfg iterator from m_roadmap end
      return m_roadmap->GetGraph()->end();
    }

    static string GetName() { return "RoadmapSet"; }

  private:

    shared_ptr<RoadmapType> m_roadmap;
};

#endif
