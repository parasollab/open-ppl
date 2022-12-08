/////////////////////////////////////
//HEADER ConstructiveAdaptiveRegion.h
/////////////////////////////////////
#ifndef CONSTRUCTADAPTIVEREGION_H_
#define CONSTRUCTADAPTIVEREGION_H_

#include "ParallelSBMPHeader.h"
#include "AdaptiveRegionDecomposition.h"
#include "Region.h"

typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Region, WeightType> RRGraph;
typedef shared_ptr<DistanceMetricMethod> DMM;
typedef shared_ptr<ValidityCheckerMethod> VCM;

using namespace psbmp;
using namespace stapl;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class ConstructAdaptiveRegion {

private:
  typedef Sampler<CfgType>::SamplerPointer NGM;
  typedef MPRegion<CfgType,WeightType>  MPR;
  typedef std::tr1::tuple<double,int, int> MTuple;


  MPR* m_region;
  Environment* m_env;
  DMM m_dmm;
  NGM m_ngm;
  VCM m_vcm;
  MTuple m_constr, m_classifier;


public:

  ConstructAdaptiveRegion(MPR* _mpr, Environment* _env,DMM _dmm,NGM _ngm,
    VCM _vcm, MTuple _coparam, MTuple _clparam) {

    m_region = _mpr;
    m_env = _env;
    m_dmm = _dmm;
    m_ngm = _ngm;
    m_vcm = _vcm;
    m_constr = _coparam ;
    m_classifier = _clparam;

  }

  void define_type(stapl::typer& _t) {
    _t.member(m_region);
  }

  template<typename bbView>
  void operator()(bbView _bview) const {

    Environment* penv = const_cast<Environment*>(m_env);
    BoundingBox bb = _bview;
    shared_ptr<BoundingBox> bbox = shared_ptr<BoundingBox>(new BoundingBox(bb));
    AdaptiveRegionDecomposition<CfgType> innerRegion;
    innerRegion.ConstructRegions(penv, bbox, m_dmm, m_region,
      m_ngm, m_vcm, m_constr, m_classifier);
    innerRegion.AddRegionEdges(penv, m_dmm,m_region);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class ConstructInnerRegionMap {
private:
  typedef MPRegion<CfgType,WeightType>  MPR;

  MPR* m_region;
  Environment* m_env;
  MPStrategyMethod* m_strategyMethod;
  VCM m_vcm;
  bool m_useOuterBB;

public:

  ConstructInnerRegionMap(MPR* _mpr, Environment* _env, MPStrategyMethod* _mpsm, VCM _vcm, bool _useOuterBB) {
    m_region = _mpr;
    m_env = _env;
    m_strategyMethod = _mpsm;
    m_vcm = _vcm;
    m_useOuterBB = _useOuterBB;
  }

  void define_type(stapl::typer& _t)  {
    _t.member(m_region);
  }

  template<typename regionView>
  void operator()(regionView _l0view) const{
    //PrintValue("Inner Region descriptor" , _l0view.descriptor());
    double tSetup,tRun;
    stapl::counter<stapl::default_timer> bt1,bt2;
    double  setup_timer=0.0, run_timer = 0.0;
    bt1.start();
    vector<VID> regionCand;
    string callee("InnerRegion");

    string sampler = _l0view.property().GetSamplerString();

    shared_ptr<BoundingBox> outerBB = _l0view.property().GetOuterBoundary();
    int outerDOF = outerBB->GetDOFs();
    int outerPosDOF = outerBB->GetPosDOFs();

    CfgType cfg1 = _l0view.property().GetCenter();
    CfgType cfg2 = _l0view.property().GetRadiusCFG();
    double rgRadius = _l0view.property().GetRadius();
    vector<double> cfg1Pos = cfg1.GetPosition();
    double robotRadius = m_env->GetMultiBody(m_env->GetRobotIndex())->GetMaxAxisRange();
    shared_ptr<BoundingBox> bb = shared_ptr<BoundingBox>(new BoundingBox(outerDOF, outerPosDOF));

    for(int i = 0; i <outerPosDOF; i++){
      bb->SetParameter(i, cfg1Pos[i]-rgRadius-2*robotRadius, cfg1Pos[i]+rgRadius+2*robotRadius);
    }
    //bb->TranslationalScale(2*robotRadius);
    //bb->Print(cout);

    ///Add region center and radius to rmap if valid
    if(m_vcm->IsValid(cfg1, &callee)){
      VID cvid = m_region->GetRoadmap()->m_pRoadmap->add_vertex(cfg1);
      //PrintValue("Inner Region center vid: ", cvid);
      regionCand.push_back(cvid);
    }

    if(m_vcm->IsValid(cfg2, &callee)){
      VID rvid = m_region->GetRoadmap()->m_pRoadmap->add_vertex(cfg2);
      //PrintValue("Inner Region radius vid: ", rvid);
      regionCand.push_back(rvid);
    }

    _l0view.property().SetVIDs(regionCand);
    //PrintValue("WF sampler is " , sampler);
    pair<string,vector<VID> > regionPar(sampler,regionCand);
    m_strategyMethod->SetSampler(regionPar);
    if(m_useOuterBB) {
      m_strategyMethod->SetBoundary(outerBB);
    }else{
      m_strategyMethod->SetBoundary(bb);
    }
    setup_timer = bt1.stop();
    bt2.start();
    m_strategyMethod->Run(0);
    run_timer = bt2.stop();

    tSetup += setup_timer;
    tRun += run_timer;

    psbmp::PrintValue("CONSTRUCT- Setup = ", tSetup);
    psbmp::PrintValue("CONSTRUCT- Run = ", tRun);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class ConstructOuterRegion {
private:
  typedef MPRegion<CfgType,WeightType>  MPR;


  MPR* m_region;
  Environment* m_env;
  MPStrategyMethod* m_strategyMethod;
  VCM m_vcm;
  bool m_useOuterBB;

public:

  ConstructOuterRegion(MPR* _mpr,Environment* _env, MPStrategyMethod* _mpsm,VCM _vcm, bool _useOuterBB) {
    m_region = _mpr;
    m_env = _env;
    m_strategyMethod = _mpsm;
    m_vcm = _vcm;
    m_useOuterBB = _useOuterBB;
  }

  void define_type(stapl::typer& _t)  {
    _t.member(m_region);
  }

  template<typename regionView>
  void operator()(regionView _l1view) const {

    typedef typename regionView::property_type pType;
    typedef typename pType::vertex_iterator VIT;
    pType p = _l1view.property();
    //std::cout << "print_h1 p size " << p.level() << endl;
    //std::cout << stapl::get_location_id() <<"> " << (_l1view).descriptor() << "/" << p.size() << "/ {" << p.domain() << "}   (";
    ConstructInnerRegionMap innerRegionMap(m_region, m_env, m_strategyMethod, m_vcm, m_useOuterBB);
    //std::for_each(p.begin(), p.end(), innerRegionMap);
    new_algorithms::for_each(p, innerRegionMap); // problem with proxy
    //std::cout << "),  \t";
  }
};

#endif

