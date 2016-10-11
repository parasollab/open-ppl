#ifndef RRTcomponents_h
#define RRTcomponents_h
#include "RRTexpand.h"


//#define STEP_FACTOR  50        // default for rrt stepFactor
//#define ITERATIONS   50        // default for rrt iterations
//#define SMALL_CC      5        // default for rrt smalcc size
//#define O_CLEARANCE   1
//#define CLEARANCE_FROM_NODE 4


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class RRTcomponents: public RRTexpand<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  RRTcomponents();
  ~RRTcomponents();

  //////////////////////
  // Access
  //void SetDefault();

  //////////////////////
  // I/O methods
  virtual ComponentConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection methods
  /**Copy vertices and all incident edges associated with "vids"
   *from one roadmap to another.
   *toMap Target, Cfgs in vids and incident edges in fromMap
   *will be copied to this submap.
   *fromMap Source, edge information will be retrived from here.
   *vids Source, vertex information will be retrived from here.
   *Usually, in this list, elements are Cfgs in same connected component.
   */

  void OrderCCByCloseness(Roadmap<CFG,WEIGHT> * rm,
				 DistanceMetric * dm,
				 vector< pair<int,VID> >& ccvec);

  // new component connection interface
  void Connect(Roadmap<CFG,WEIGHT>* _rm, StatClass& Stats,
	       CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges);
  void Connect(Roadmap<CFG,WEIGHT>* _rm, StatClass& Stats,
	       CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges,
	       vector<VID>& vids1, vector<VID>& vids2);
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  RRTcomponents
template <class CFG, class WEIGHT>
RRTcomponents<CFG,WEIGHT>::RRTcomponents():RRTexpand<CFG,WEIGHT>() {
  element_name = "RRTcomponents"; //in ConnectCCs there is RRTcomponents

  SetDefault();
}


template <class CFG, class WEIGHT>
RRTcomponents<CFG,WEIGHT>::~RRTcomponents() {
}


template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>*
RRTcomponents<CFG,WEIGHT>::
CreateCopy() {
  RRTcomponents<CFG,WEIGHT>* _copy =
           new RRTcomponents<CFG,WEIGHT>(*this);
  return _copy;
}


/*---------------------------------------------------------------
Will order CCs by their distances from their center's of mass to
the largest CCs center of mass
---------------------------------------------------------------*/
template <class CFG, class WEIGHT>
void RRTcomponents<CFG,WEIGHT>::
OrderCCByCloseness(Roadmap<CFG,WEIGHT> * rm,
		   DistanceMetric * dm,
		   vector< pair<int,VID> >& ccvec) {

  Environment *env = rm->GetEnvironment();

  vector< pair<int,VID> >::iterator cc2=ccvec.begin();

  stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  vector<VID> vidvec;

  get_cc(*(rm->m_pRoadmap),(*cc2).second ,cmap, vidvec);
  int i = 0; int index = 0;
  CFG vtemp;
  vector<CFG> centervec;

  while(cc2<ccvec.end()) {

    while( i < vidvec.size() ) {
      vtemp=(*(rm->m_pRoadmap->find_vertex(vidvec[i]))).property();
      if (i==0)
	centervec.push_back(vtemp);
      else{
	centervec[index].add(centervec[index],vtemp);

      }
      i++;
    } //while i<vidvec.size()

    //DisplayCC((*rm->m_pRoadmap),(*cc2).second);

    centervec[index].divide(centervec[index],i);

    cc2++; i = 0; index++;
    vidvec.clear();
    cmap.reset();
    get_cc(*(rm->m_pRoadmap), cmap, (*cc2).second ,vidvec);
  } //while cc2<ccvec.end()
  vector<CFG> centvec;
  for(i=1; i<centervec.size();i++) {
    centvec.push_back( centervec[i] );
  }

  vector<pair<CFG,CFG> > kp= dm->FindKClosestPairs(env, centervec[0], centvec,
						   centervec.size()-1);
  vector< pair<int,VID> > ccvec_tmp;
  ccvec_tmp.push_back( *ccvec.begin() );
  for(i=0; i<kp.size(); i++) {
    int j=0;

    while ( (kp[i].second != centervec[j]) && (j<centervec.size()) )
      j++;
    if ( j < centervec.size() ) {

      pair<int,VID>& tmp = ccvec[j];
      ccvec_tmp.push_back( tmp );
      }

  }// end for
  ccvec.swap(ccvec_tmp);
  for(int b=0;b<ccvec.size();b++) {
    //cout << "ccvec["<<b<<"] "<<ccvec[b].second;
    //cout << endl;
  }
}

template <class CFG, class WEIGHT>
void RRTcomponents<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
	CollisionDetection* cd ,
	DistanceMetric * dm,
	LocalPlanners<CFG,WEIGHT>* lp,
	bool addPartialEdge,
	bool addAllEdges,
	vector<VID>& vids1, vector<VID>& vids2) {

  Roadmap<CFG,WEIGHT> submap1;
  submap1.environment = _rm->GetEnvironment();

  ModifyRoadMap(&submap1, _rm, vids1);
  ModifyRoadMap(&submap1, _rm, vids2);

  Connect(&submap1, Stats, cd, dm, lp, addPartialEdge, addAllEdges);

  vector<VID> verts;
  (&submap1)->m_pRoadmap->GetVerticesVID(verts);
  ModifyRoadMap(_rm, &submap1,verts);
  submap1.environment = NULL;


}

template <class CFG, class WEIGHT>
void RRTcomponents<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
	CollisionDetection* cd ,
	DistanceMetric * dm,
	LocalPlanners<CFG,WEIGHT>* lp,
	bool addPartialEdge,
	bool addAllEdges) {
  // display information specific to method

  cout << "RRTcomponents(iterations = "<< iterations
       << ", stepFactor= "<< stepFactor
       << ", smallcc   = "<< smallcc
       << ", oclearance = " << o_clearance
       << ", node_clearance = " << clearance_from_node
       <<"): " <<flush;

  ///Modified for VC
#if defined(_WIN32)
  using namespace std;
#endif

  // process components from smallest to biggest
  stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  vector< pair<size_t,VID> > ccvec;
  get_cc_stats(*(_rm->m_pRoadmap),cmap, ccvec);

  vector< pair<int,VID> >::iterator cc1=ccvec.begin();
  for ( int z = 0; z <=1; z++) {

    vector< pair<int,VID> > tempccvec;
    cmap.reset();
    get_cc_stats(*(_rm->m_pRoadmap), cmap, tempccvec);

    if( tempccvec.size() <= 1 ) continue;

    //-- submap = vertices & edges of current (cc1) connected component
    Roadmap<CFG,WEIGHT> submap1;

    vector<CFG> dummyU;

    vector< pair<int,VID> >::iterator cc2=cc1+1;

    //DisplayCCStats(*(_rm->m_pRoadmap),0);

    vector< pair<int,VID> >::iterator cctemp=ccvec.begin();
    int b =0;
    if (z == 0)
      while(cctemp!=ccvec.end()) {
	vector<VID> cc;
	cmap.reset();
	get_cc(*(_rm->m_pRoadmap), cmap, (*cctemp).second,cc);
	if ( cc.size()<= smallcc ) {
	  Roadmap<CFG,WEIGHT> submap3;
	  submap3.environment = _rm->GetEnvironment();
	  vector<VID> cct;
	  cmap.reset();
	  get_cc(*(_rm->m_pRoadmap), cmap, (*cctemp).second,cct);
	  ModifyRoadMap(&submap3,_rm,cct);
	  bool toConnect = FALSE;

	  RRT(&submap3, Stats,
	      iterations/2,
	      stepFactor * _rm->GetEnvironment()->GetPositionRes(),
	      o_clearance,clearance_from_node,
	      dummyU,
	      cd, lp, dm, toConnect,FALSE,
	      addPartialEdge, addAllEdges);
	  vector<VID> verts1;
	  (&submap3)->m_pRoadmap->GetVerticesVID(verts1);
	  ModifyRoadMap(_rm,&submap3,verts1);
	  submap3.environment = NULL;
	} //end if

	cctemp++; b++;
      } //cctemp<=ccvec.end()


    if (( z == 1) && (ccvec.size()>2)) {
      cmap.reset();
      get_cc_stats(*(_rm->m_pRoadmap),cmap, ccvec);
      // Will put the first/largest CC at the end move everything
      // else up one spot
      vector< pair<int,VID> >::iterator startcciter = ccvec.begin();
      vector< pair<int,VID> >::iterator endcciter = ccvec.end();
      startcciter++;
      vector< pair<int,VID> > tmp_vec1;
      vector< pair<int,VID> >::iterator startIterator = tmp_vec1.begin();
      tmp_vec1.insert(startIterator,startcciter,endcciter);
      startcciter = ccvec.begin();
      tmp_vec1.push_back(*startcciter);
      ccvec.swap( tmp_vec1  );
    }

    OrderCCByCloseness(_rm,dm,ccvec);

    cc1= ccvec.begin();
    cc2 = ccvec.begin(); cc2++;
    VID cc1id = (*cc1).second;

    //while(cc2!=ccvec.end()) {
    while(cc2<ccvec.end()) {

      Roadmap<CFG,WEIGHT> submap2;
      submap2.environment = _rm->GetEnvironment();
      vector<VID> cct2;
      cmap.reset();
      get_cc(*(_rm->m_pRoadmap),cmap, (*cc2).second,cct2);
      ModifyRoadMap(&submap2,_rm,cct2);
      submap1.environment = _rm->GetEnvironment();
      vector<VID> cc;
      cmap.reset();
      get_cc(*(_rm->m_pRoadmap),cmap, (*cc1).second,cc);
      ModifyRoadMap(&submap1,_rm,cc);

      VID cc2id = (*cc2).second;
      int i = 0;
      vector<CFG> dummyU;
      vector<CFG> U;
      int d=0;

      //DisplayCCStats(*(_rm->m_pRoadmap),-1);

      bool toConnect = FALSE;
      vector<VID> cct3;
      cmap.reset();
      get_cc(*(_rm->m_pRoadmap),cmap, (*cc2).second,cct3);
      cmap.reset();
      if(cct3.size()>= smallcc)
	while ( !is_same_cc(*(_rm->m_pRoadmap), cmap, cc1id,cc2id)
                && !toConnect
		&& (i<iterations) ) {
	  U.clear();
 	  cmap.reset();
          toConnect = FALSE;

	  cmap.reset();
	  while (!toConnect
		 && (!is_same_cc(*(_rm->m_pRoadmap), cmap, cc1id,cc2id))
		 && ( i <= iterations )) {


	    U.clear();
	    cmap.reset();
	    i++;
	    if ((i % 2 )== 0) {

	      toConnect = FALSE;
	      RRT(&submap1, Stats,
		  1,
		  stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		  o_clearance,clearance_from_node,
		  U,cd,lp,dm,toConnect,TRUE,
		  addPartialEdge, addAllEdges);

	      toConnect = TRUE;
	      RRT(&submap2, Stats,
		  1,
		  stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		  o_clearance,clearance_from_node,
		  U,cd,lp,dm,toConnect,TRUE,
		  addPartialEdge, addAllEdges);
	    }
	    else {
	      toConnect = FALSE;
	      RRT(&submap2, Stats,
		  1,
		  stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		  o_clearance,clearance_from_node,
		  U,cd,lp,dm,toConnect,TRUE,
		  addPartialEdge, addAllEdges);

	      toConnect = TRUE;
	      RRT(&submap1, Stats,
		  1,
		  stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		  o_clearance,clearance_from_node,
		  U,cd,lp,dm,toConnect,TRUE,
		  addPartialEdge, addAllEdges);
	    }

	  } //end while (!toConnect && ( i <= _cn.GetIterations() )

	  vector<VID> vertsa;
	  (&submap1)->m_pRoadmap->GetVerticesVID(vertsa);
	  ModifyRoadMap(_rm,&submap1, vertsa);
	  vector<VID> vertsb;  //not sure here if cc2 should be cc2 or vice versa
	  (&submap2)->m_pRoadmap->GetVerticesVID(vertsb);
	  ModifyRoadMap(_rm,&submap2, vertsb);

	  d++;
	} //end while sameCC
      cc2++; d=0; i=0;
      submap2.environment = NULL;
      submap2.m_pRoadmap = NULL;
    } //end while cc2 != ccvec.end()

    submap1.environment = NULL;
    submap1.m_pRoadmap = NULL;

  } //endfor cc1 and cc1.next





}
#endif
