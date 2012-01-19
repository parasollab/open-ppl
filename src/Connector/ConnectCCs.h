#ifndef ConnectCCs_h
#define ConnectCCs_h

#include "ComponentConnectionMethod.h"


//ConnectCCs
   /**Try to connect different connected components of the roadmap.
    *We try to connect all pairs of connected components. If both
    *components are small (less than "smallcc" nodes), then we try to 
    *connect all pairs of nodes.  If at least one of the components is 
    *large, we try to connect the "kpairs" closest pairs of nodes.
    *
    *@see WeightedGraph ::Get CCStats, WeightedGraph ::Get CC, 
    *WeightedGraph ::Is SameCC for information about connected 
    *component in graph,and ConnectSmallCCs, ConnectBigCCs for 
    *connecting between connected component.
    */


#define KPAIRS        4        // default for connectCCs
#define SMALL_CC      3        // default for connectCCs: smallcc size


template <class CFG, class WEIGHT>
class ConnectCCs: public ComponentConnectionMethod<CFG,WEIGHT> {
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  //////////////////////
  // Constructors and Destructor
  ConnectCCs();
  ConnectCCs(int k, size_t cc, string nf);
  ConnectCCs(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ConnectCCs(Roadmap<CFG,WEIGHT>*);
  virtual ~ConnectCCs();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods

  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);
  virtual ComponentConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection method
  /**Connect two small connected components.
   *Algorithm:
   *   -# for every Cfg, cfg1, in cc1
   *       -# for every Cfg. cgf2, in cc2
   *           -# if cfg1 and cfg2 could be connected.
   *           -# then add this edge to roadmap, _rm.
   *              and return.
   *           -# else if info.addPartialEdge
   *               -# then add failed path to roadmap
   *                  (cfg1->failed_point)
   *           -# end if
   *       -# end for
   *   -# end for
   *
   *@param _cc1id a node id which defines the first connected component.
        *@param _cc2id a node id which defines the second connected component.
        */
   template<typename OutputIterator>
  void ConnectSmallCCs(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
		       vector<VID>& cc1vec, 
		       vector<VID>& cc2vec,
		       bool addPartialEdge,
		       bool addAllEdges,
                       OutputIterator collision);  
  /**Connect two big connected components.
   *Algorithm:
   *   -# find k pairs of closest Cfgs from cc1 to cc2.
   *   -# for every pair, p, in k pairs.
   *       -# if p.first and p.second could be connected.
   *          (p.first is from cc1 and p.second is from cc2)
   *       -# then add this edge to roadmap, _rm.
   *          and return.
   *       -# else if info.addPartialEdge
   *           -# then add failed path to roadmap
   *              (cfg1->failed_point)
   *       -# end if
   *   -# end for
   *
   *In step 1, FindKClosestPairs is used to find first k
   *closest pairs from the first connected component to 
   *the second connected component.
   *
   *@param _cc1id a node id which defines the first connected component.
   *@param _cc2id a node id which defines the second connected component.
   *@param _cn _cn.GetKPairs define k above.
   */
   template<typename OutputIterator>
  void ConnectBigCCs(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
		     vector<VID>& cc1vec, 
		     vector<VID>& cc2vec,
		     bool addPartialEdge, bool addAllEdges,
                     OutputIterator collision);

  //@}
   template<typename OutputIterator>
  void Connect(Roadmap<CFG, WEIGHT>*, StatClass& Stats,
		 bool addPartialEdge,
		 bool addAllEdges, OutputIterator collision);
	
	template <typename InputIterator, typename OutputIterator>
  void Connect(Roadmap<CFG, WEIGHT>*, StatClass& Stats,
		 bool addPartialEdge,
		 bool addAllEdges,
		 InputIterator _itr1_first, InputIterator _itr1_last,
                 InputIterator _itr2_first, InputIterator _itr2_last, OutputIterator collision);

// private:
  //////////////////////
  // Data

  int kpairs;
  size_t smallcc;
  string nf_label;
  string m_lp;
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  ConnectCCs
template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::ConnectCCs():
  ComponentConnectionMethod<CFG,WEIGHT>() { 
  this->SetName("components"); 

  SetDefault();
}

template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::
ConnectCCs(int k, size_t cc, string nf) : kpairs(k), smallcc(cc), nf_label(nf) {
SetDefault();
}

template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::
ConnectCCs(XMLNodeReader& in_Node, MPProblem* in_pProblem):
  ComponentConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  this->SetName("components"); 
  SetDefault();

  nf_label = in_Node.stringXMLParameter("nf", true, "", "nf");
  m_lp = in_Node.stringXMLParameter("lp_method", true, "default", "LocalPlanner");
  kpairs = in_Node.numberXMLParameter("kpairs", true, 5,1,1000, "kpairs value"); 
  smallcc = in_Node.numberXMLParameter("smallcc", true, 5,1,1000,  "smallcc value"); 
}


template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::ConnectCCs(Roadmap<CFG,WEIGHT> * rdmp):
  ComponentConnectionMethod<CFG,WEIGHT>(rdmp) {
  this->SetName("components");

  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::~ConnectCCs() { 
}


template <class CFG, class WEIGHT>
void ConnectCCs<CFG,WEIGHT>::SetDefault() {
  kpairs = KPAIRS;
  smallcc = SMALL_CC;

}


template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << this->GetName() << " ";
  _os << "\tINTEGER INTEGER (default kpairs:" << KPAIRS << ", smallcc:" << SMALL_CC << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName() << " kpairs = ";
  _os << kpairs << "smallcc = " << smallcc ;
  _os << endl;
}


template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  out_os << "    " << this->GetName() << "::  kpairs = ";
  out_os << kpairs << "  smallcc = " << smallcc ;
  out_os << endl;
}


template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>* 
ConnectCCs<CFG,WEIGHT>::
CreateCopy() {
  ComponentConnectionMethod<CFG,WEIGHT>* _copy = 
           new ConnectCCs<CFG,WEIGHT>(*this);
  return _copy;
}


//
// try to connect all pairs of cfgs in the two CCs
//
template <class CFG, class WEIGHT>
template <typename OutputIterator>
void
ConnectCCs<CFG, WEIGHT>::
ConnectSmallCCs(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
		vector<typename RoadmapGraph<CFG, WEIGHT>::VID>& cc1vec, vector<typename RoadmapGraph<CFG, WEIGHT>::VID>& cc2vec,
		bool addPartialEdge, bool addAllEdges, OutputIterator collision) {
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  
  // created a temporary variable since Get CC requires &
  LPOutput<CFG,WEIGHT> lpOutput;
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(nf_label)->GetDMMethod();
  for (size_t c1 = 0; c1 < cc1vec.size(); c1++){
    for (size_t c2 = 0; c2 < cc2vec.size(); c2++){
      if(_rm->IsCached(cc1vec[c1], cc2vec[c2]) && !_rm->GetCache(cc1vec[c1], cc2vec[c2])) {
        continue;
      }
      CfgType _col;
      if (!_rm->m_pRoadmap->IsEdge(cc1vec[c1],cc2vec[c2]) 
          && this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp)->
               IsConnected(_rm->GetEnvironment(),Stats,dm,
                           (*(pMap->find_vertex(cc1vec[c1]))).property(),
                           (*(pMap->find_vertex(cc2vec[c2]))).property(),
                           _col, &lpOutput, this->connectionPosRes, 
                           this->connectionOriRes, (!addAllEdges)) ) {
	pMap->AddEdge(cc1vec[c1], cc2vec[c2], lpOutput.edge);
	return;
      }
      else if(addPartialEdge) {
	typename vector<pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > >::iterator I;
	for(I=lpOutput.savedEdge.begin(); I!=lpOutput.savedEdge.end(); I++) {
	  CFG tmp = I->first.second;
	  if(!tmp.AlmostEqual((*(pMap->find_vertex(cc1vec[c1]))).property())) {
	    VID tmpVID = pMap->AddVertex(tmp);
	    pMap->AddEdge(cc1vec[c1], tmpVID, I->second);
	  }
	}
      }
      if(_col != CfgType())
         *collision++ = _col;
    }//end for c2
  }//end for c1
}


//
// try to connect kclosest pairs of cfgs in the two CCs
//
template <class CFG, class WEIGHT>
template <typename OutputIterator>
void
ConnectCCs<CFG, WEIGHT>::
ConnectBigCCs(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
	      vector<typename RoadmapGraph<CFG, WEIGHT>::VID>& cc1vec, vector<typename RoadmapGraph<CFG, WEIGHT>::VID>& cc2vec,
	      bool addPartialEdge, bool addAllEdges, OutputIterator collision) { 
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  
  size_t k = min(kpairs, cc2vec.size());

  vector<pair<VID,VID> > kp(k);
  typename vector<pair<VID,VID> >::iterator kp_iter = kp.begin();
  this->GetMPProblem()->GetNeighborhoodFinder()->KClosestPairs(
                this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(nf_label),
                _rm, 
                cc1vec.begin(), cc1vec.end(), 
                cc2vec.begin(),cc2vec.end(), 
                k, kp_iter);
  
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(nf_label)->GetDMMethod();
  LPOutput<CFG,WEIGHT> lpOutput;
  for (size_t i = 0; i < kp.size(); i++) {
    if(_rm->IsCached(kp[i].first, kp[i].second) && !_rm->GetCache(kp[i].first, kp[i].second)) {
      continue;
    }
    CfgType _col;
    if(!_rm->m_pRoadmap->IsEdge(kp[i].first,kp[i].second) 
       && this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp)->
            IsConnected(_rm->GetEnvironment(),Stats,dm,
                        (*(pMap->find_vertex(kp[i].first))).property(),
                        (*(pMap->find_vertex(kp[i].second))).property(),
                        _col, &lpOutput, this->connectionPosRes, 
                        this->connectionOriRes, (!addAllEdges)) ) {
      pMap->AddEdge(kp[i].first, kp[i].second, lpOutput.edge); 
      return;
    }
    else if(addPartialEdge) {
      typename vector<pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > >::iterator I;
      for(I=lpOutput.savedEdge.begin(); I!=lpOutput.savedEdge.end(); I++) {
	CFG tmp = I->first.second;
	if(!tmp.AlmostEqual((*(pMap->find_vertex(kp[i].first))).property())) {
	  VID tmpVID = pMap->AddVertex(tmp);
	  pMap->AddEdge(kp[i].first, tmpVID, I->second);
	}
      }
    }
    if(_col != CfgType())
         *collision++ = _col;
  }//end for c1
}



template <class CFG, class WEIGHT>
template <typename OutputIterator>
void ConnectCCs<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
          bool addPartialEdge,
	  bool addAllEdges, OutputIterator collision) {
  vector< pair<size_t,VID> > ccs1;
  stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  get_cc_stats(*(_rm->m_pRoadmap),cmap,ccs1);
  
  cout << "components(kpairs="<< kpairs ;
  cout << ", smallcc="<<smallcc <<"): "<<flush;
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  if(ccs1.size()>1){
     // process components from smallest to biggest  
     typename vector<pair<size_t,VID> >::reverse_iterator C1, C2;
     for(C1 = ccs1.rbegin(); C1+1 != ccs1.rend(); ++C1) {
        for(C2 = C1+1; C2 != ccs1.rend(); ++C2) {      
           // if cc1 & cc2 not already connected, try to connect them 
           cmap.reset();
           if ( !is_same_cc(*pMap,cmap,C1->second,C2->second) ) {
              vector<VID> cc1;
              cmap.reset();
              get_cc(*pMap,cmap,C1->second,cc1);
              
              vector<VID> cc2;
              cmap.reset();
              get_cc(*pMap,cmap,C2->second,cc2);
              
              if(cc1.size() < smallcc && cc2.size() < smallcc ) {
                ConnectSmallCCs(_rm,Stats, cc1,cc2,addPartialEdge,addAllEdges, collision);
              } else {
                 if(cc1.size() <= cc2.size())
                   ConnectBigCCs(_rm,Stats, cc1,cc2,addPartialEdge,addAllEdges, collision);
                 else
                   ConnectBigCCs(_rm,Stats, cc2,cc1,addPartialEdge,addAllEdges, collision);
              }
           } 
        }/*endfor cc2*/
     }/*endfor cc1*/
  }
}


template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void ConnectCCs<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
          bool addPartialEdge,
          bool addAllEdges,
          InputIterator _itr1_first, InputIterator _itr1_last,
          InputIterator _itr2_first, InputIterator _itr2_last, OutputIterator collision) {
//	  vector<VID> & vids1, vector<VID> & vids2) {
	  //vector<typename RoadmapGraph<CFG, WEIGHT>::VID> & vids1, vector<typename RoadmapGraph<CFG, WEIGHT>::VID> & vids2) {
  cout << "components(kpairs="<< kpairs ;
  cout << ", smallcc="<<smallcc <<"): "<<flush;
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  //DisplayCCStats(*pMap); cout << endl;

  for (InputIterator itr1 = _itr1_last; itr1 != _itr1_first; --itr1) {
    for (InputIterator itr2 = _itr2_last; itr2 != _itr2_first; --itr2) {
      // if V1 & V2 not already connected, try to connect them 
      cmap.reset();
      if ( !is_same_cc(*pMap,cmap,*itr1,*itr2) ) {
        vector<VID> cc1;
	      cmap.reset();
        get_cc(*pMap,cmap,*itr1,cc1);

        vector<VID> cc2;
	      cmap.reset();
        get_cc(*pMap,cmap,*itr2,cc2);

        if(cc1.size() < smallcc && cc2.size() < smallcc ) {
          ConnectSmallCCs(_rm,Stats,cc1,cc2,addPartialEdge,addAllEdges, collision);
        } else {
          if(cc1.size() <= cc2.size())
            ConnectBigCCs(_rm,Stats,cc1,cc2,addPartialEdge,addAllEdges, collision);
          else
            ConnectBigCCs(_rm,Stats,cc2,cc1,addPartialEdge,addAllEdges, collision);
        }
      } 

    }/*endfor V2*/ 
  }/*endfor V1*/

  //DisplayCCStats(*pMap); cout << endl;
}

#endif
