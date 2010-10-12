#ifndef ConnectkCCs_h
#define ConnectkCCs_h

#include "ComponentConnectionMethod.h"
#include "Closest.h"
#include "NeighborhoodConnection.h"

//ConnectkCCs
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


#define K1_CLOSEST 5         // k1 closest nodes between two CC
#define K2_CLOSEST 2         // k2 closest CCs

template <class CFG, class WEIGHT>
class ConnectkCCs: public ComponentConnectionMethod<CFG,WEIGHT> {
 public:
 typedef typename RoadmapGraph<CFG,WEIGHT>::vertex_descriptor VID;
  //////////////////////
  // Constructors and Destructor
  ConnectkCCs();
  ConnectkCCs(Roadmap<CFG,WEIGHT>*, 
              DistanceMetricMethod*, LocalPlanners<CFG,WEIGHT>*);
  virtual ~ConnectkCCs();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual ComponentConnectionMethod<CFG, WEIGHT>* CreateCopy();

  //Connect
  void Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
         DistanceMetricMethod* dm,
         LocalPlanners<CFG,WEIGHT>* lp ,
         bool addPartialEdge,
         bool addAllEdges);

  template <typename InputIterator>
  void Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
         DistanceMetricMethod* dm,
         LocalPlanners<CFG,WEIGHT>* lp,
         bool addPartialEdge,
         bool addAllEdges,
         InputIterator _itr1_first, InputIterator _itr1_last,
         InputIterator _itr2_first, InputIterator _itr2_last);

protected:

    // compute all pair distance between ccs.
    // approximated using coms of ccs
    template <typename InputIterator>
    void compute_AllPairs_CCDist_com(Roadmap<CFG, WEIGHT>* _rm,
                                                   DistanceMetricMethod * dm,
                                                   InputIterator _ccs1_first, InputIterator _ccs1_last,
                                                   InputIterator _ccs2_first, InputIterator _ccs2_last);
    // compute all pair distance between ccs.
    // shortest dist betweem ccs
    void compute_AllPairs_CCDist_closest(Roadmap<CFG, WEIGHT>* _rm,
                                                       DistanceMetricMethod * dm,
                                                       vector<VID>& ccs1,vector<VID>& ccs2);

    //get k2 closest pairs
    void get_K2_Pairs(int ccid,vector<int>&k2_ccid);
    //void getCCData(RoadmapGraph<CFG, WEIGHT> *rmapG,VID vid,vector<CFG>& ccdata);
    CFG CC_com(RoadmapGraph<CFG, WEIGHT> *rmapG,VID vid);

    // get closest dist between two given CCs
    double closestInterCCDist( Roadmap<CFG, WEIGHT>* _rm, DistanceMetricMethod * dm,
                                        vector<VID>& cc1, vector<VID>& cc2);
 private:
  //////////////////////
  // Data
  int k1;  // k  closest nodes
  int k2;  // k  closest cc
  vector< vector<double> > ccDist;
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  ConnectkCCs
template <class CFG, class WEIGHT>
ConnectkCCs<CFG,WEIGHT>::ConnectkCCs():
  ComponentConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "k-components"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectkCCs<CFG,WEIGHT>::ConnectkCCs(Roadmap<CFG,WEIGHT> * rdmp, DistanceMetricMethod* dm, LocalPlanners<CFG,WEIGHT>* lp):
  ComponentConnectionMethod<CFG,WEIGHT>(rdmp, dm, lp) {
  this->element_name = string("k-components");

  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectkCCs<CFG,WEIGHT>::~ConnectkCCs() { 
}

template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::SetDefault() {
  k1 = K1_CLOSEST;
  k2 = K2_CLOSEST;
}


template <class CFG, class WEIGHT>
void
ConnectkCCs<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  _os <<"\n"<<this->GetName()<<" ";
  _os <<"\tINTEGER INTEGER (default k1:"<<K1_CLOSEST<<", k2:"<<K2_CLOSEST<<")";
  _os <<endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ConnectkCCs<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName() << " k1 = "<< k1 << "k2 = " << k2 ;
  _os << endl;
}


template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>* 
ConnectkCCs<CFG,WEIGHT>::
CreateCopy() {
  ComponentConnectionMethod<CFG,WEIGHT>* _copy = 
           new ConnectkCCs<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
          DistanceMetricMethod * dm,
          LocalPlanners<CFG,WEIGHT>* lp,
          bool addPartialEdge,
      bool addAllEdges) {
  
#ifndef QUIET
  cout << this->GetName()<<" (k1="<<k1<<", k2="<<k2<<"): "<<flush;
#endif
  Closest<CFG,WEIGHT> cl(k1);
  cl.cdInfo=this->cdInfo;
  cl.connectionPosRes=this->connectionPosRes;
  cl.connectionOriRes=this->connectionOriRes;
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  vector< pair<size_t,VID> > ccs;
  int ccsize=get_cc_stats(*pMap, cmap, ccs);

  //getCC
  vector<VID> ccid;
  vector< vector<VID> > ccids;
  ccid.reserve(ccsize);
  for(typename vector< pair<size_t,VID> >::iterator i=ccs.begin();i!=ccs.end();i++){
      ccid.push_back(i->second);
      ccids.push_back(vector<VID>()); 
      cmap.reset();
      get_cc(*pMap, cmap, i->second, ccids.back());
  }//end for i

  //(1) compute all pair cc dists
  compute_AllPairs_CCDist_com(_rm,dm,ccid,ccid);
  //compute_AllPairs_CCDist_closest(_rm,dm,ccid,ccid);
  //for(int i=0;i<ccsize;i++) ccDist[i][i]=1e20; //so it wont be in k2 closest
  //(2) 
 /*
  vector<vector<size_t> > ccids_aux(ccids.size()); // conversion between ccids and ccids_aux;
  for(unsigned int i=0; i<ccids.size(); ++i)
     for(typename vector<VID>::iterator itr= ccids[i].begin(); itr!= ccids[i].end(); ++itr)
	ccids_aux[i].push_back(size_t(*itr));
*/
  typename vector<VID>::reverse_iterator V1;
  for(V1 = ccid.rbegin(); V1 != ccid.rend(); ++V1) {

      int id=ccid.rend()-V1-1;
      //find k2 closest
      vector<int> k2_ccid;
      get_K2_Pairs(id,k2_ccid);

      //connect
      for(vector<int>::iterator v2=k2_ccid.begin();v2!=k2_ccid.end();v2++)
          cl.Connect(_rm,Stats,dm,lp,addPartialEdge,addAllEdges,ccids[id],ccids[*v2]);
          //cl.Connect(_rm,Stats,dm,lp,addPartialEdge,addAllEdges,ccids_aux[id],ccids_aux[*v2]);
  }/*endfor V1*/
}


template <class CFG, class WEIGHT>
template<typename InputIterator>
void ConnectkCCs<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
          DistanceMetricMethod * dm,
          LocalPlanners<CFG,WEIGHT>* lp,
          bool addPartialEdge,
          bool addAllEdges,
          InputIterator _itr1_first, InputIterator _itr1_last,
          InputIterator _itr2_first, InputIterator _itr2_last) {
#ifndef QUIET
  cout << this->GetName()<<" (k1="<<k1<<", k2="<<k2<<"): "<<flush;
#endif

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  NeighborhoodConnection<CFG,WEIGHT> nc(k1);
  nc.cdInfo=this->cdInfo;
  nc.connectionPosRes=this->connectionPosRes;
  nc.connectionOriRes=this->connectionOriRes;

  //DisplayCCStats(*pMap); cout << endl;

  stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  //getCC data
  vector< vector<VID> > ccset1, ccset2;
  
  for(int j=0;j<2;j++){
    InputIterator vids_first = (j==0) ? _itr1_first : _itr2_first;
    InputIterator vids_last = (j==0) ? _itr1_last : _itr2_last;
    
    //vector<VID> & vids = (j==0) ? vids1 : vids2;
    vector< vector<VID> >& ccset=(j==0)?ccset1:ccset2;
    
    for(InputIterator i = vids_first; i != vids_last; i++){
      ccset.push_back(vector<VID>());  
      cmap.reset();
      get_cc(*pMap, cmap, *i, ccset.back());
    }//end for i
  }//end for j

  //(1) compute all pair cc dists
  compute_AllPairs_CCDist_com(_rm,dm, _itr1_first, _itr1_last, _itr2_first, _itr2_last);
  //compute_AllPairs_CCDist_closest(_rm,dm,vids1,vids2 );
  //(2) 
  typename vector<VID>::reverse_iterator V1;
  for(InputIterator itr1 = _itr1_last; itr1 != _itr1_first; --itr1) {
      int id = itr1 - _itr1_first - 1;
      //find k2 closest
      vector<int> k2_ccid;
      get_K2_Pairs(id,k2_ccid);
      //connect
      for(vector<int>::iterator v2=k2_ccid.begin();v2!=k2_ccid.end();v2++)
          nc.ConnectNodes(_rm,Stats,dm,lp,addPartialEdge,addAllEdges,
                          ccset1[id].begin(), ccset1[id].end(),
                          ccset2[*v2].begin(), ccset2[*v2].end());
  }/*endfor V1*/
  //DisplayCCStats(*pMap); cout << endl;
}

//get k2 closest pairs
template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::get_K2_Pairs(int ccid,vector<int>&k2_ccid)
{
    typedef pair<double,int> CCD; //dist to cc
    vector<double>& dis2CCs=ccDist[ccid];
    vector<CCD> ccids;
    ccids.reserve(k2);
    // go through each cc
    for(vector<double>::iterator it=dis2CCs.begin();it!=dis2CCs.end();it++){
        int id=it-dis2CCs.begin();
        if(ccids.size()<k2){ //not yet enough
            ccids.push_back(CCD(*it,id));
            push_heap(ccids.begin(),ccids.end());
        }
        else{//start to check
            pop_heap(ccids.begin(),ccids.end());
            CCD& back=ccids.back();
            if(back.first>*it){
                back.first=*it;
                back.second=id;
            }  
            push_heap(ccids.begin(),ccids.end());
        }
    }//end for it
    //copy
    k2_ccid.reserve(k2);
    for(vector<CCD>::iterator i=ccids.begin();i!=ccids.end();i++)
        k2_ccid.push_back(i->second);
}

// compute all pair distance between ccs.
// exact cc dist     
template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::
compute_AllPairs_CCDist_closest
(Roadmap<CFG, WEIGHT>* _rm,
 DistanceMetricMethod * dm,
 vector<VID>& ccs1,vector<VID>& ccs2)
{
    RoadmapGraph<CFG,WEIGHT> * rmapG=_rm->m_pRoadmap;
    stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
    ccDist.clear();
    for(typename vector<VID>::iterator i=ccs1.begin();i!=ccs1.end();i++){
        ccDist.push_back(vector<double>());
        vector<VID> cc1;
        get_cc(*rmapG,cmap,  *i, cc1);
        for(typename vector<VID>::iterator j=ccs2.begin();j!=ccs2.end();j++){
            double d=1e20;
            if((*i)!=(*j)){//if not the same cc
                vector<VID> cc2;
		            cmap.reset();
                get_cc(*rmapG, cmap, *j, cc2);
                d=closestInterCCDist(_rm,dm,cc1,cc2);
            }
            ccDist.back().push_back(d);
        }//end j
    }//end i
}

template <class CFG, class WEIGHT>
double ConnectkCCs<CFG,WEIGHT>::
closestInterCCDist( Roadmap<CFG, WEIGHT>* _rm, DistanceMetricMethod * dm,
vector<VID>& cc1, vector<VID>& cc2)
{
    RoadmapGraph<CFG,WEIGHT> * rmapG=_rm->m_pRoadmap;
    Environment * p_env=_rm->GetEnvironment();
    typedef typename vector<VID>::iterator IT;
    double min_dist=1e20;
    for(IT i=cc1.begin();i!=cc1.end();i++){
        const CFG& cfg1=(*(rmapG->find_vertex(*i))).property();
        for(IT j=cc2.begin();j!=cc2.end();j++){
            const CFG& cfg2=(*(rmapG->find_vertex(*j))).property();
            double d=dm->Distance(p_env,cfg1,cfg2);
            if(d<min_dist) min_dist=d;
        }//end j
    }//end i
    return min_dist;
}

// compute all pair distance between ccs.
// approximated using com of cc
template <class CFG, class WEIGHT>
template <typename InputIterator>
void ConnectkCCs<CFG,WEIGHT>::
compute_AllPairs_CCDist_com(Roadmap<CFG, WEIGHT>* _rm, DistanceMetricMethod * dm,
                            InputIterator _ccs1_first, InputIterator _ccs1_last,
                            InputIterator _ccs2_first, InputIterator _ccs2_last)
{
    RoadmapGraph<CFG,WEIGHT> * rmapG=_rm->m_pRoadmap;
    Environment * p_env=_rm->GetEnvironment();
    //compute com of ccs
    vector<CFG> com1,com2;
    for (InputIterator i = _ccs1_first; i != _ccs1_last; i++) 
      com1.push_back(CC_com(rmapG,*i));
    for (InputIterator i = _ccs2_first; i != _ccs2_last; i++) 
      com2.push_back(CC_com(rmapG,*i));

    //dist between ccs
    ccDist.clear();
    for(typename vector<CFG>::iterator i=com1.begin();i!=com1.end();i++){
        ccDist.push_back(vector<double>());
        int id1=i-com1.begin();
        for(typename vector<CFG>::iterator j=com2.begin();j!=com2.end();j++){
            int id2=j-com2.begin();
            if (*(_ccs1_first + id1) != *(_ccs2_first + id2))
            //if(ccs1[id1]!=ccs2[id2]) //not the same
                ccDist.back().push_back(dm->Distance(p_env,*i,*j));
            else //same cc
                ccDist.back().push_back(1e20);
        }
    }//end for i
}

template <class CFG, class WEIGHT>
CFG ConnectkCCs<CFG,WEIGHT>::CC_com
(RoadmapGraph<CFG, WEIGHT> *rmapG,VID vid)
{
// RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
    stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
    vector<VID> ccvids;
    get_cc(*rmapG, cmap, vid, ccvids);

    //compute com
    CFG com;
    for(typename vector<VID>::iterator i=ccvids.begin();i!=ccvids.end();i++)
        com.add(com,(*(rmapG->find_vertex(*i))).property());
    com.divide(com,ccvids.size());

    return com;
}
/*
template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::getCCIDs
(RoadmapGraph<CFG, WEIGHT> *rmapG,VID vid,vector<VID>& ccid)
{
    Get CC(*rmapG, vid, ccid);
}
*/

#endif
