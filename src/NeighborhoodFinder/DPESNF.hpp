#ifndef _DPES_NEIGHBORHOOD_FINDER_H_
#define _DPES_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.hpp"
#include "util.h"
#include "MPProblem.h"
#include "DPES.h"

#include <vector>
#include <functional>
using namespace std;


template<typename CFG, typename WEIGHT>
class VID_DPES_proxy{
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  VID_DPES_proxy(VID _v, Roadmap<CFG,WEIGHT>* _rmp){
    m_vid = _v;
    //if (m_rmp == NULL)   
      m_rmp = _rmp;
  }

  CFG GetData() { return (*(m_rmp->m_pRoadmap->find_vertex(m_vid))).property();}
  const CFG GetData() const { return (*(m_rmp->m_pRoadmap->find_vertex(m_vid))).property();}  
   bool operator==(const VID_DPES_proxy<CFG,WEIGHT>& _p) {
    return (GetData() ==  _p.GetData());
 }

private:
  VID m_vid;
   Roadmap<CFG,WEIGHT>* m_rmp;
  
};

template<typename CFG, typename WEIGHT>
class CFG_DPES_Pivot_proxy{
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  CFG_DPES_Pivot_proxy(VID_DPES_proxy<CFG,WEIGHT> _vdp) { m_cfg = _vdp.GetData(); }
  CFG_DPES_Pivot_proxy(const CFG& _cfg) { m_cfg = _cfg; }

  CFG GetData() { return m_cfg; }
  const CFG GetData() const { return m_cfg; }
  bool operator==(const CFG_DPES_Pivot_proxy<CFG,WEIGHT>& _p) {
    return (GetData() ==  _p.GetData());
  }
  bool operator==(const VID_DPES_proxy<CFG,WEIGHT>& _p) {
    return (GetData() ==  _p.GetData());
  }
  
private:
  CFG m_cfg;

};

template<typename CFG, typename WEIGHT>
class DistanceMetric_DPES_proxy {
public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  DistanceMetric_DPES_proxy(shared_ptr<DistanceMetricMethod> dm, Environment* env) { 
    //if(m_dm == NULL)
      m_dm = dm; 
    //if(m_env == NULL)
      m_env = env;
  }

  double operator()(const VID_DPES_proxy<CFG,WEIGHT>& _p1, const VID_DPES_proxy<CFG,WEIGHT>& _p2) const  {  
    return m_dm->Distance(m_env, _p1.GetData(), _p2.GetData());
  }

  double operator()(const VID_DPES_proxy<CFG,WEIGHT>& _p1, const CFG_DPES_Pivot_proxy<CFG,WEIGHT>& _cfg) const {
    return m_dm->Distance(m_env, _p1.GetData(), _cfg.GetData());
  }
  
  double operator()(const CFG_DPES_Pivot_proxy<CFG,WEIGHT>& _cfg, const VID_DPES_proxy<CFG,WEIGHT>& _p1) const {
    return m_dm->Distance(m_env, _cfg.GetData(), _p1.GetData());
  }
  
  double operator()(const CFG_DPES_Pivot_proxy<CFG,WEIGHT>& _cfg1, const CFG_DPES_Pivot_proxy<CFG,WEIGHT>& _cfg2) const {
    return m_dm->Distance(m_env, _cfg1.GetData(), _cfg2.GetData());
  }


private:
   shared_ptr<DistanceMetricMethod> m_dm;
   Environment* m_env;
}; 
 
 
template<typename CFG, typename WEIGHT>
class DPESNF: public NeighborhoodFinderMethod {
  
typedef DistanceMetric_DPES_proxy<CFG,WEIGHT> DM_PROXY;
  
typedef DPES<VID_DPES_proxy<CFG,WEIGHT>, 
       CFG_DPES_Pivot_proxy<CFG,WEIGHT>, 
       DistanceMetric_DPES_proxy<CFG,WEIGHT>, 
       DistanceMetric_DPES_proxy<CFG,WEIGHT> > DPES_TYPE;
       
typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;       
       
public:
  DPESNF(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    NeighborhoodFinderMethod(ParseLabelXML(in_Node),in_Node, in_pProblem), m_dprox(NULL), m_dpes(NULL) {
  
    
    m_m = in_Node.numberXMLParameter("m", false, int(3),
                                                  int(1), int(6),
                                                  "m value for DPES");
    m_l = in_Node.numberXMLParameter("l", false, int(50),
                                                  int(5), int(1000),
                                                  "l value for DPES");
    m_cur_roadmap_version= -1;
  }

  DPESNF(shared_ptr<DistanceMetricMethod> _dmm, std::string _strLabel) :
    NeighborhoodFinderMethod(_strLabel), m_dprox(NULL), m_dpes(NULL) {
    dmm = _dmm;
    m_cur_roadmap_version = -1;
    m_m = 3;
    m_l = 50;
  }

  virtual ~DPESNF() {}

  virtual const std::string GetName () const {
    return DPESNF::GetClassName();
  }
  static const std::string GetClassName() {
    return "DPESNF";
  }
  virtual void PrintOptions(std::ostream& out_os) const {
    out_os << this->GetClassName() << ":: m = " << m_m << "  l = " << m_l << std::endl;
  }


  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int k,
    OutputIterator _out);
  
  // do the work here, and have the function above obtain the CFG and call this one
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k,
    OutputIterator _out);
  
  
  // KClosest that operate over the entire roadmap to find the kclosest to a VID or CFG
  //
  // NOTE: These are the prefered methods for kClosest computations
  template <typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    VID _v, int k, OutputIterator _out);
  
  template <typename OutputIterator>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, int k, OutputIterator _out);
  

  // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
  // represent the kclosest pairs of VIDs between the two ranges.
  template <typename InputIterator, typename OutputIterator>
  OutputIterator
  KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
    InputIterator _in1_first, InputIterator _in1_last, 
    InputIterator _in2_first, InputIterator _in2_last, 
    int k, OutputIterator _out);
    
    int getM(size_t _size) const { return m_m; } 
    int getL(size_t _k)  const {return m_l; }

private:
  DM_PROXY* m_dprox;//(dmm, _rmp->GetEnvironment());
  DPES_TYPE* m_dpes;//(dprox, dprox);
  int m_cur_roadmap_version;
  void UpdateInternalModel( Roadmap<CFG,WEIGHT>* _rmp );  
  int m_m; ///< Number of pivots
  int m_l; ///< Number of DPES space neighbors, l > k

  //dpes model here
  //int roadmap version
  //void UpdateInternalModel(Roadmap<CFG,WEIGHT>* _rmp) {
    //1) checks roadmap version with internal version #
    //2) updates any changes since last version.
  //}
};

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
DPESNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int k,
    OutputIterator _out) {
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property();
  return KClosest(_rmp, _input_first, _input_last, _v_cfg, k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
DPESNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k,
  OutputIterator _out) {

  StartTotalTime();
  IncrementNumQueries();

  int l= getL( k);
  int m = getM(100);//_rmp.size());
  
  DM_PROXY dprox(dmm, _rmp->GetEnvironment());
  DPES_TYPE dpes(dprox, dprox);

// Creat S       
  InputIterator vi;
  for (vi = _input_first; vi != _input_last; ++vi) { 
      dpes.AddNode(VID_DPES_proxy<CFG,WEIGHT>(*vi, _rmp));
  }     

//Creat Pivots
  dpes.CreatPivots(m);

//Project to VS
  dpes.UpdateProjected();

  vector< pair<int, double> > closest(k);

  StartQueryTime();
  dpes.KClosestPIVOT(CFG_DPES_Pivot_proxy<CFG,WEIGHT>(_cfg), closest.begin(), k, l);
  EndQueryTime();

  for(vector< pair<int, double> >::iterator iter = closest.begin(); iter != closest.end(); ++iter) {
		*_out = iter->first;
		++_out;
    //cout << "\t" << iter->first << "\t" << iter->second << endl;
  //  cout << iter->second << " - index = " << iter->first << endl;

	}
  //cout << endl;

  // 1) Make VID_DPES_proxy
  // 2) Make CFG_DPES_pivot_proxy
  // 3) Make DistanceMetric_proxy
  //    -- compare VID_DPES_proxy VID_DPES_proxy
  //    -- compare CFG_DPES_pivot_proxy VID_DPES_proxy
  //    -- use shared_ptr<DistanceMetricMethod> dmm received from constructor
   //return _input_first;
   EndTotalTime();
   return _out;
}


template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
DPESNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  VID _v, int k, OutputIterator _out) {

  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = (*(pMap->find_vertex(_v))).property(); 
  return KClosest(_rmp, _v_cfg, k, _out);
}


template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator
DPESNF<CFG,WEIGHT>::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
  CFG _cfg, int k, OutputIterator _out) {
//  cout << "DPESNF::KClosest() - For entire roadmap" << endl;
  StartTotalTime();
  
  StartConstructionTime();
  UpdateInternalModel(_rmp);
  EndConstructionTime();

  IncrementNumQueries();

  int l= getL( k);

  vector< pair<int, double> > closest(k);

  StartQueryTime();
  m_dpes->KClosestPIVOT(CFG_DPES_Pivot_proxy<CFG,WEIGHT>(_cfg), closest.begin(), k, l);
  EndQueryTime();
	  
  for(vector< pair<int, double> >::iterator iter = closest.begin(); iter != closest.end(); ++iter) {
		*_out = iter->first;
		++_out;
    //cout << "\t" << iter->first << "\t" << iter->second << endl;
  //  cout << iter->second << " - index = " << iter->first << endl;

	}
 // cout << "m = " << m_m << " l = " << l << endl;
  EndTotalTime();
  return _out;

}


template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator
DPESNF<CFG,WEIGHT>::
KClosestPairs( Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last, 
  int k, OutputIterator _out) {

  // need to provide an implementation of this
  return _out;
}



template<typename CFG, typename WEIGHT>
void
DPESNF<CFG,WEIGHT>::
UpdateInternalModel( Roadmap<CFG,WEIGHT>* _rmp )
{
  if(m_dpes == NULL) { //create the model for the first time
    m_dprox = new DM_PROXY(dmm, _rmp->GetEnvironment());
    m_dpes = new DPES_TYPE(*m_dprox, *m_dprox);
  //  m_cur_roadmap_version = 0;
  }
  
  int new_version = _rmp->m_pRoadmap->roadmapVCS.get_version_number();
  if(m_cur_roadmap_version != new_version) {
    //redo everytghing.... clear m_dpes.
    if(m_dpes != NULL) {
      //delete m_dpes;
      m_dpes->Clear();
    }
    m_dpes = new DPES_TYPE(*m_dprox, *m_dprox);    
    // Creat S
    vector< VID > roadmap_vids;
    _rmp->m_pRoadmap->GetVerticesVID(roadmap_vids);
    typename vector<VID>::iterator vi;
    for (vi = roadmap_vids.begin(); vi != roadmap_vids.end(); ++vi) { 
        m_dpes->AddNode(VID_DPES_proxy<CFG,WEIGHT>(*vi, _rmp));
  }  
  //Creat Pivots
  //int m = 3;//getM(_rmp.size());
  m_dpes->CreatPivots(m_m);

  //Project to VS
  m_dpes->UpdateProjected();
  m_cur_roadmap_version = new_version;
  }
  /*
  RoadmapChangeLog::cce_iter start = _rmp->m_pRoadmap->roadmapChangeLog->iter_at(latest_version);
  RoadmapChangeLog::cce_iter end = _rmp->m_pRoadmap->roadmapChangeLog->end();
  RoadmapChangeLog::cce_iter iter;
  CFG temp_cfg;
  VID _v = 0;
  for(iter = start; iter != end; iter++) {
    if((*iter).second.IsTypeAddVertex()) {
        //m_dpes->Add....
      //this->AddPoint(NULL, (*iter).second->GetEvent()->GetVID());
//      tree.insert(Point_d(temp_cfg.DOF(), _v, (*iter).second->GetEvent()->GetCFG().GetData().begin(), 
//                                              (*iter).second->GetEvent()->GetCFG().GetData().end()));

//need to update S and VS.
    }
  }
  */
}


#endif //end #ifndef _DPES_NEIGHBORHOOD_FINDER_H_
