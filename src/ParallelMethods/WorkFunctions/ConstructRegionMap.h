#ifndef CONSTRUCTREGIONMAP_H_
#define CONSTRUCTREGIONMAP_H_

#include "Environment/BoundingBox.h"
#include "ParallelMethods/ParallelSBMPHeader.h"

using namespace psbmp;
using namespace stapl;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// General Framework for roadmap construction, calls MPStrategy method (i.e.
/// BasicPRM, Basic RRT) directly to be called if region migration is not
/// needed.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ConstructRoadmap {
  public:
    typedef void result_type;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;

    ConstructRoadmap(MPStrategyPointer _mpsm ): m_strategyMethod(_mpsm){ }
    ConstructRoadmap(const ConstructRoadmap& _wf, std::size_t _offset)  {}
    void define_type(stapl::typer& _t){ }

    template<typename View, typename bbView>
    void operator()(View _view,  bbView _bbview) const;

  private:
    MPStrategyPointer m_strategyMethod;
};

template<class MPTraits>
template<typename View, typename BBView>
void
ConstructRoadmap<MPTraits>::
operator() (View _view, BBView _bbView) const {
  BoundingBox bb = _bbView;
  shared_ptr<BoundingBox> boundary = shared_ptr<BoundingBox>(new BoundingBox(bb));
  vector<VID> dummy;
  Region<BoundingBox, MPTraits> bbInfo(boundary,dummy);
  _view.property() = bbInfo;

  m_strategyMethod->SetBoundary(boundary);
  m_strategyMethod->Run();
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NodeGenerator {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::SamplerPointer NGM;
    typedef void result_type;

    NodeGenerator(MPProblemType* _problem, NGM _ngm, int _num): m_attempts(_num) {
      m_problem = _problem;
      m_sp = _ngm;
    }

    void define_type(stapl::typer& _t) {
    }

    template <typename BBView, typename RGView>
      void operator()(BBView _v1, RGView _v2) const;

  private:
    MPProblemType* m_problem;
    NGM m_sp;
    int m_attempts;
};

template <class MPTraits>
template <typename BBView, typename RGView>
void
NodeGenerator<MPTraits>::
operator() (BBView _v1, RGView _v2) const {
  vector<CfgType> outNodes;
  vector<VID> regionVIDs;
  string callee("Generator");
  BoundingBox bb = _v1;
  shared_ptr<BoundingBox> boundary = shared_ptr<BoundingBox>(new BoundingBox(bb));

  m_sp->Sample(m_attempts, 10, boundary, back_inserter(outNodes));

  typedef typename vector<CfgType>::iterator VIT;
  for(VIT vit = outNodes.begin(); vit  != outNodes.end(); ++vit) {

    CfgType tmp = *vit;
    //TODO: Pass validity checker label as string
    if(m_problem->GetValidityChecker("rapid")->IsValid(tmp, callee)) {
      VID vid = m_problem->GetRoadmap()->GetGraph()->add_vertex(tmp);
      regionVIDs.push_back(vid);
    }
  }
  Region<BoundingBox, MPTraits> bbInfo(boundary,regionVIDs);
  _v2.property() = bbInfo;

}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NodeConnector {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::ConnectorPointer NCP;
    typedef void result_type;

    NodeConnector(MPProblemType* _problem, NCP _ncp) {
      m_problem = _problem;
      m_ncp = _ncp;
    }

    void define_type(stapl::typer& _t) {}

    template<typename regionView>
    void operator()(regionView _view) const;

  private:
    MPProblemType* m_problem;
    NCP m_ncp;
};

template <class MPTraits>
template <typename RegionView>
void
NodeConnector<MPTraits>::
operator() (RegionView _view) const {
  vector<VID> regionVIDs = _view.property().RegionVIDs();
  m_ncp->Connect(m_problem->GetRoadmap(),
      regionVIDs.begin(), regionVIDs.end(),
      regionVIDs.begin(), regionVIDs.end());
}

#endif
