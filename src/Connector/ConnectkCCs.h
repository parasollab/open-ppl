#ifndef ConnectkCCs_h
#define ConnectkCCs_h

#include "ComponentConnectionMethod.h"
#include "Closest.h"

//ConnectkCCs
   /**Try to connect different connected components of the roadmap.
    *We try to connect all pairs of connected components. If both
    *components are small (less than "smallcc" nodes), then we try to 
    *connect all pairs of nodes.  If at least one of the components is 
    *large, we try to connect the "kpairs" closest pairs of nodes.
    *
    *@see WeightedGraph ::GetCCStats, WeightedGraph ::GetCC, 
    *WeightedGraph ::IsSameCC for information about connected 
    *component in graph,and ConnectSmallCCs, ConnectBigCCs for 
    *connecting between connected component.
    */


#define K1_CLOSEST 5         // k1 closest nodes between two CC
#define K2_CLOSEST 2         // k2 closest CCs

template <class CFG, class WEIGHT>
class ConnectkCCs: public ComponentConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ConnectkCCs();
  ConnectkCCs(Roadmap<CFG,WEIGHT>*, CollisionDetection*, 
              DistanceMetric*, LocalPlanners<CFG,WEIGHT>*);
  ~ConnectkCCs();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods

  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual ComponentConnectionMethod<CFG, WEIGHT>* CreateCopy();

  //Connect
  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
         CollisionDetection*, 
         DistanceMetric *,
         LocalPlanners<CFG,WEIGHT>*,
         bool addPartialEdge,
         bool addAllEdges);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
         CollisionDetection*, 
         DistanceMetric *,
         LocalPlanners<CFG,WEIGHT>*,
         bool addPartialEdge,
         bool addAllEdges,
         vector<VID>& vids1, vector<VID>& vids2);

protected:

    // compute all pair distance between ccs.
    // approximated using coms of ccs
    void compute_AllPairs_CCDist_com(Roadmap<CFG, WEIGHT>* _rm,
                                                   DistanceMetric * dm,
                                                   vector<VID>& ccs1,vector<VID>& ccs2);
    // compute all pair distance between ccs.
    // shortest dist betweem ccs
    void compute_AllPairs_CCDist_closest(Roadmap<CFG, WEIGHT>* _rm,
                                                       DistanceMetric * dm,
                                                       vector<VID>& ccs1,vector<VID>& ccs2);

    //get k2 closest pairs
    void get_K2_Pairs(int ccid,vector<int>&k2_ccid);
    //void getCCData(RoadmapGraph<CFG, WEIGHT> *rmapG,VID vid,vector<CFG>& ccdata);
    CFG CC_com(RoadmapGraph<CFG, WEIGHT> *rmapG,VID vid);

    // get closest dist between two given CCs
    double closestInterCCDist( Roadmap<CFG, WEIGHT>* _rm, DistanceMetric * dm,
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
ConnectkCCs<CFG,WEIGHT>::ConnectkCCs(Roadmap<CFG,WEIGHT> * rdmp, CollisionDetection* cd, DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp):
  ComponentConnectionMethod<CFG,WEIGHT>(rdmp, cd, dm, lp) {
  this->element_name = string("k-components");

  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectkCCs<CFG,WEIGHT>::~ConnectkCCs() { 
}


template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::
ParseCommandLine(std::istringstream& is) {
  char c;
  SetDefault();
  try {
    c = is.peek();
    while(c == ' ' || c == '\n') {
      is.get();
      c = is.peek();
    }
    if (c >= '0' && c <= '9') {
      if (is >> k1) {
        if (k1 < 0)
        throw BadUsage();

        c = is.peek();
        while(c == ' ' || c == '\n') {
          is.get();
          c = is.peek();
        }
        if (c >= '0' && c <='9') {
          if (is >> k2) {
            if (k2 < 0)
          throw BadUsage();
          } else
            throw BadUsage();
        }

      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "Error in \'"<<this->GetName()<<"\' parameters" << endl;
    PrintUsage(cerr);
    exit(-1);
  }
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
          CollisionDetection* cd , 
          DistanceMetric * dm,
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
  vector< pair<int,VID> > ccs;
  int ccsize=GetCCStats(*pMap,ccs);
  
  //getCC
  vector<VID> ccid;
  vector< vector<VID> > ccids;
  ccid.reserve(ccsize);
  for(vector< pair<int,VID> >::iterator i=ccs.begin();i!=ccs.end();i++){
      ccid.push_back(i->second);
      ccids.push_back(vector<VID>()); 
      GetCC(*pMap,i->second, ccids.back());
  }//end for i

  //(1) compute all pair cc dists
  compute_AllPairs_CCDist_com(_rm,dm,ccid,ccid);
  //compute_AllPairs_CCDist_closest(_rm,dm,ccid,ccid);
  //for(int i=0;i<ccsize;i++) ccDist[i][i]=1e20; //so it wont be in k2 closest
  //(2) 
  vector<VID>::reverse_iterator V1;
  for(V1 = ccid.rbegin(); V1 != ccid.rend(); ++V1) {

      int id=ccid.rend()-V1-1;
      //find k2 closest
      vector<int> k2_ccid;
      get_K2_Pairs(id,k2_ccid);

      //connect
      for(vector<int>::iterator v2=k2_ccid.begin();v2!=k2_ccid.end();v2++)
          cl.Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges,ccids[id],ccids[*v2]);
  }/*endfor V1*/
}


template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
          CollisionDetection* cd , 
          DistanceMetric * dm,
          LocalPlanners<CFG,WEIGHT>* lp,
          bool addPartialEdge,
          bool addAllEdges,
      vector<VID> & vids1, vector<VID> & vids2) {
#ifndef QUIET
  cout << this->GetName()<<" (k1="<<k1<<", k2="<<k2<<"): "<<flush;
#endif

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  Closest<CFG,WEIGHT> cl(k1);
  cl.cdInfo=this->cdInfo;
  cl.connectionPosRes=this->connectionPosRes;
  cl.connectionOriRes=this->connectionOriRes;

  //DisplayCCStats(*pMap); cout << endl;

  //getCC data
  vector< vector<VID> > ccset1, ccset2;
  for(int j=0;j<2;j++){
    vector<VID> & vids=(j==0)?vids1:vids2;
    vector< vector<VID> >& ccset=(j==0)?ccset1:ccset2;
    for(vector<VID>::iterator i=vids.begin();i!=vids.end();i++){
        ccset.push_back(vector<VID>());  
        GetCC(*pMap, *i, ccset.back());
    }//end for i
  }//end for j

  //(1) compute all pair cc dists
  compute_AllPairs_CCDist_com(_rm,dm,vids1,vids2);
  //compute_AllPairs_CCDist_closest(_rm,dm,vids1,vids2 );
  //(2) 
  vector<VID>::reverse_iterator V1;
  for(V1 = vids1.rbegin(); V1 != vids1.rend(); ++V1) {
      int id=vids1.rend()-V1-1;
      //find k2 closest
      vector<int> k2_ccid;
      get_K2_Pairs(id,k2_ccid);
      //connect
      for(vector<int>::iterator v2=k2_ccid.begin();v2!=k2_ccid.end();v2++)
          cl.Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges,ccset1[id],ccset2[*v2]);
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
 DistanceMetric * dm,
 vector<VID>& ccs1,vector<VID>& ccs2)
{
    RoadmapGraph<CFG,WEIGHT> * rmapG=_rm->m_pRoadmap;
    ccDist.clear();
    for(vector<VID>::iterator i=ccs1.begin();i!=ccs1.end();i++){
        ccDist.push_back(vector<double>());
        vector<VID> cc1;
        GetCC(*rmapG, *i, cc1);
        for(vector<VID>::iterator j=ccs2.begin();j!=ccs2.end();j++){
            double d=1e20;
            if((*i)!=(*j)){//if not the same cc
                vector<VID> cc2;
                GetCC(*rmapG, *j, cc2);
                d=closestInterCCDist(_rm,dm,cc1,cc2);
            }
            ccDist.back().push_back(d);
        }//end j
    }//end i
}

template <class CFG, class WEIGHT>
double ConnectkCCs<CFG,WEIGHT>::
closestInterCCDist( Roadmap<CFG, WEIGHT>* _rm, DistanceMetric * dm,
vector<VID>& cc1, vector<VID>& cc2)
{
    RoadmapGraph<CFG,WEIGHT> * rmapG=_rm->m_pRoadmap;
    Environment * p_env=_rm->GetEnvironment();
    typedef vector<VID>::iterator IT;
    double min_dist=1e20;
    for(IT i=cc1.begin();i!=cc1.end();i++){
        const CFG& cfg1=rmapG->GetData(*i);
        for(IT j=cc2.begin();j!=cc2.end();j++){
            const CFG& cfg2=rmapG->GetData(*j);
            double d=dm->Distance(p_env,cfg1,cfg2);
            if(d<min_dist) min_dist=d;
        }//end j
    }//end i
    return min_dist;
}

// compute all pair distance between ccs.
// approximated using com of cc
template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::
compute_AllPairs_CCDist_com
(Roadmap<CFG, WEIGHT>* _rm,
 DistanceMetric * dm,
 vector<VID>& ccs1,vector<VID>& ccs2)
{
    RoadmapGraph<CFG,WEIGHT> * rmapG=_rm->m_pRoadmap;
    Environment * p_env=_rm->GetEnvironment();
    //compute com of ccs
    vector<CFG> com1,com2;
    for(vector<VID>::iterator i=ccs1.begin();i!=ccs1.end();i++) 
        com1.push_back(CC_com(rmapG,*i));
    for(vector<VID>::iterator i=ccs2.begin();i!=ccs2.end();i++) 
        com2.push_back(CC_com(rmapG,*i));

    //dist between ccs
    ccDist.clear();
    for(typename vector<CFG>::iterator i=com1.begin();i!=com1.end();i++){
        ccDist.push_back(vector<double>());
        int id1=i-com1.begin();
        for(typename vector<CFG>::iterator j=com2.begin();j!=com2.end();j++){
            int id2=j-com2.begin();
            if(ccs1[id1]!=ccs2[id2]) //not the same
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
    vector<VID> ccvids;
    GetCC(*rmapG, vid, ccvids);

    //compute com
    CFG com;
    for(vector<VID>::iterator i=ccvids.begin();i!=ccvids.end();i++)
        com.add(com,rmapG->GetData(*i));
    com.divide(com,ccvids.size());

    return com;
}
/*
template <class CFG, class WEIGHT>
void ConnectkCCs<CFG,WEIGHT>::getCCIDs
(RoadmapGraph<CFG, WEIGHT> *rmapG,VID vid,vector<VID>& ccid)
{
    GetCC(*rmapG, vid, ccid);
}
*/

#endif
