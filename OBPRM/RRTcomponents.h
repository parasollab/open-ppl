#ifndef RRTcomponents_h
#define RRTcomponents_h
#include "RRTexpand.h"


//#define STEP_FACTOR  50        // default for rrt stepFactor
//#define ITERATIONS   50        // default for rrt iterations
//#define SMALL_CC      5        // default for rrt smalcc size
//#define O_CLEARANCE   1
//#define CLEARANCE_FROM_NODE 4 


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
  // ParseCommandLine derived
  virtual ConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection methods 
  /**Copy vertices and all incident edges associated with "vids" 
   *from one roadmap to another.
   *@param toMap Target, Cfgs in vids and incident edges in fromMap  
   *will be copied to this submap.
   *@param fromMap Source, edge information will be retrived from here.
   *@param vids Source, vertex information will be retrived from here.
   *Usually, in this list, elements are Cfgs in same connected component.
   */
/*  static void ModifyRoadMap(Roadmap<CFG, WEIGHT>* toMap, 
			    Roadmap<CFG, WEIGHT>* fromMap, 
			    vector<VID> vids);
  void RRT(Roadmap<CFG, WEIGHT>* rm, 
	   int K, double deltaT, int o_clearance, 
	   int clearance_from_node,vector<CFG>& U,
	   CollisionDetection*, LocalPlanners<CFG,WEIGHT>*,
	   DistanceMetric *,
	   bool & toConnect, bool connecting);
*/
  void OrderCCByCloseness(Roadmap<CFG,WEIGHT> * rm,
				 DistanceMetric * dm,
				 vector< pair<int,VID> >& ccvec);
  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);
 private:
  //////////////////////
  // Data
  //int iterations; 
  //int stepFactor;  
  //int smallcc; 
  //int o_clearance;
  //int clearance_from_node;
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
ConnectionMethod<CFG,WEIGHT>* 
RRTcomponents<CFG,WEIGHT>::
CreateCopy() {
  RRTcomponents<CFG,WEIGHT>* _copy = 
           new RRTcomponents<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void RRTcomponents<CFG,WEIGHT>::ConnectComponents() {
  cout << "Connecting CCs with method: RRTcomponents" << endl;
  cout << "DO NOTHING" <<endl;
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
  cout << "\nOrderCCByCloseness\n";
  Environment *env = rm->GetEnvironment();   
  //rm->m_pRoadmap->DisplayCCSt	ats(	-1); 
  vector< pair<int,VID> >::iterator cc2=ccvec.begin();
  //vector<VID> vidvec = rm->m_pRoadmap	->GetCC((*cc2).second);
  vector<VID> vidvec;
  //cout << "cc2.se	cond in order cc:" << (*cc2).second;
  GetCC(*(rm->m_pRoadmap),(*cc2).second ,vidvec);
  int i = 0; int index = 0;
  CFG vtemp;
  vector<CFG> centervec;
  cout << "orderccs problem-1";
  while(cc2<ccvec.end()) {

    while( i < vidvec.size() ) {
      vtemp=rm->m_pRoadmap->GetData(vidvec[i]);
	    //cout << vtemp << endl;
      if (i==0)
	centervec.push_back(vtemp);
      else{
	centervec[index].add(centervec[index],vtemp);
	//for(int d=0;
      }
      i++;
    } //while i<vidvec.size()
    cout << i << endl;
    DisplayCC((*rm->m_pRoadmap),(*cc2).second);
    cout << "centervec=" << centervec[index] << endl;
    centervec[index].divide(centervec[index],i);
    cout << "centervec=" << centervec[index] << endl;
    cc2++; i = 0; index++;     
    cout << "cc2.second in order cc:" << (*cc2).second;
    //vidvec=rm->m_pRoadmap->GetCC((*cc2).second);
    vidvec.clear();
    GetCC(*(rm->m_pRoadmap),(*cc2).second ,vidvec);
  } //while cc2<ccvec.end()
  vector<CFG> centvec; 
  for(i=1; i<centervec.size();i++) {
    centvec.push_back( centervec[i] );
  }  
  cout << "orderccs problem1";
  vector<pair<CFG,CFG> > kp= dm->FindKClosestPairs(env, centervec[0], centvec, 
						   centervec.size()-1);
  vector< pair<int,VID> > ccvec_tmp;
  ccvec_tmp.push_back( *ccvec.begin() );
  for(i=0; i<kp.size(); i++) {
    int j=0;
    cout << "orderccs problem2";
    while ( (kp[i].second != centervec[j]) && (j<centervec.size()) )
      j++;
    if ( j < centervec.size() ) {
      cout << "putting centervec "<<j<<"at position "<<i+1<<"\n";
      //pair<int,VID> tmp = ccvec[i+1];
      //ccvec[i+1] = ccvec[j];
      //ccvec[j] = tmp;
      pair<int,VID>& tmp = ccvec[j];
      ccvec_tmp.push_back( tmp );
      }
    else cout << "ERROR in OrderCCByCloseness" << endl; 
  }// end for
  ccvec.swap(ccvec_tmp);
  for(int b=0;b<ccvec.size();b++)
    cout << "ccvec["<<b<<"] "<<ccvec[b].second;
  cout << endl;
}


template <class CFG, class WEIGHT>
void RRTcomponents<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  // display information specific to method
  #ifndef QUIET
  cout << "RRTcomponents(iterations = "<< iterations
       << ", stepFactor= "<< stepFactor
       << ", smallcc   = "<< smallcc
       << ", oclearance = " << o_clearance
       << ", node_clearance = " << clearance_from_node
       <<"): " <<flush;
  #endif
 
///Modified for VC
#if defined(_WIN32)
	using namespace std;
#endif

  // process components from smallest to biggest
  // vector< pair<int,VID> > ccvec = _rm->m_pRoadmap->GetCCStats();
	vector< pair<int,VID> > ccvec;
	GetCCStats(*(_rm->m_pRoadmap),ccvec);
/*  for ( vector< pair<int,VID> >::reverse_iterator cc1=ccvec.rbegin();
       (*cc1).first <= _cn.GetSmallCCSize() && 
#if defined(__HP_aCC)
       (cc1!=ccvec.rend())
#else
       (cc1<ccvec.rend())
#endif
       ;++cc1){ 
*/
      vector< pair<int,VID> >::iterator cc1=ccvec.begin();
for ( int z = 0; z <=1; z++) {



      //-- submap = vertices & edges of current (cc1) connected component
      Roadmap<CFG,WEIGHT> submap1;

      //submap->InitEnvironment(_rm->GetEnvironment());
      //Environment temp_env = Environment::Environment(0);
      /*temp_env = *_rm->GetEnvironment(); */
      //*submap.environment = new Environment;
      //&submap.environment.pathversion = _rm->GetEnvironment()->GetPathVersion();
      //submap.environment.multibodyCount= ( _rm->GetEnvironment() )->GetMultiBodyCount();
      //for(int i=0;i<submap.environment.multibodyCount; i++)
      //submap.environment.multibody[i] = _rm->GetEnvironment()->GetMultiBody(i);
      //submap.environment = _rm->GetEnvironment();       //submap.environment = Environment(submap.environment);
      

      //submap1.environment = _rm->GetEnvironment();

      //vector<VID> cc;
      //GetCC(*(_rm->m_pRoadmap),(*cc1).second,cc);
      //ModifyRoadMap(&submap1,_rm,cc);
      vector<CFG> dummyU;


	//"RRTcomponents"
	//submap1.m_pRoadmap = _rm->m_pRoadmap;

          vector< pair<int,VID> >::iterator cc2=cc1+1;

	  //_rm->m_pRoadmap->DisplayCCStats(-1);
	  vector< pair<int,VID> >::iterator cctemp=ccvec.begin();
          int b =0;
	  if (z == 0)
	  while(cctemp!=ccvec.end()) {
		vector<VID> cc;
      		GetCC(*(_rm->m_pRoadmap),(*cctemp).second,cc);
		if ( cc.size()<= smallcc ) {
	  	  Roadmap<CFG,WEIGHT> submap3;
	          submap3.environment = _rm->GetEnvironment();
		  vector<VID> cct;
                  GetCC(*(_rm->m_pRoadmap),(*cctemp).second,cct);
	 	  ModifyRoadMap(&submap3,_rm,cct);
		  bool toConnect = FALSE;
                  cout << "problem here 12atrue";
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
		cout << "iter" << b;
		//_rm->m_pRoadmap->DisplayCC((*cctemp).second);
		
		cctemp++; b++;
	  } //cctemp<=ccvec.end()

        cout << "z==1\n";
	if (( z == 1) && (ccvec.size()>2)) {
	   GetCCStats(*(_rm->m_pRoadmap),ccvec);
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
	   //vector< pair<int,VID> >::iterator startIterator = ccvec.begin();
 	   //vector< pair<int,VID> >::iterator ccswitch=ccvec.begin();
	   //ccswitch++; 
	   //pair<int,VID> ins = ccvec[0];
	   //ccvec.push_back(ins);
	   //startIterator = ccvec.begin();
	   //ccvec.erase(startIterator);
	   //cc1 = ccvec.begin();
	   //cc2 = cc1 + 1; 
	   //pair<int,VID>  first = ccvec[0];
	   //ccvec[1] = ccvec[0];
	   //cc
	   }
          
	  OrderCCByCloseness(_rm,dm,ccvec);
	  cout<<"\n";
          for(b=0;b<ccvec.size();b++)
             cout << "ccvec["<<b<<"] "<<ccvec[b].second;
          cout<<endl;
          cc1= ccvec.begin();
          cc2 = ccvec.begin(); cc2++;
          cout << "outside orderccsbycloseness\n";
	  VID cc1id = (*cc1).second;
          cout << "cc1: " << (*cc1).second << " cc2: " << (*cc2).second << "\n";
     while(cc2!=ccvec.end()) {
          /*for (vector< pair<int,VID> >::iterator cctemp=ccvec.begin();
		  cctemp<=ccvec.end();++cctemp) 
	      _rm->m_pRoadmap->DisplayCC((*cctemp).second);
	  */
          Roadmap<CFG,WEIGHT> submap2;
          submap2.environment = _rm->GetEnvironment();
	  //cc2 = ++cc1;
          //cout << "problem here\n";
          vector<VID> cct2;
          GetCC(*(_rm->m_pRoadmap),(*cc2).second,cct2);
	  ModifyRoadMap(&submap2,_rm,cct2);
          submap1.environment = _rm->GetEnvironment();
          vector<VID> cc;
          GetCC(*(_rm->m_pRoadmap),(*cc1).second,cc);
          ModifyRoadMap(&submap1,_rm,cc);

		VID cc2id = (*cc2).second;
	  int i = 0;
          vector<CFG> dummyU;
	  vector<CFG> U;
	  cout << "outside while\n";
          cout << "cc1: " << (*cc1).second << " cc2: " << (*cc2).second << "\n";
	  int d=0;
          cout << "GetCCcount" << GetCCcount(*(_rm->m_pRoadmap)) << "\n";
          DisplayCCStats(*(_rm->m_pRoadmap),-1);
          //cout << "problem here 2\n";
          bool toConnect = FALSE;
          vector<VID> cct3;
          GetCC(*(_rm->m_pRoadmap),(*cc2).second,cct3);
	  if(cct3.size()>= smallcc)
	  while ( !IsSameCC(*(_rm->m_pRoadmap),cc1id,cc2id) 
                && !toConnect
		&& (i<iterations) ) {
	  U.clear();
          toConnect = FALSE;
	  /*while ( (U.size()<2)
	    && (!IsSameCC(*(_rm->m_pRoadmap),cc1id,cc2id))
	    && ( i <= _cn.GetIterations() ) ) {
          */
           while (!toConnect 
                  && (!IsSameCC(*(_rm->m_pRoadmap),cc1id,cc2id))
                  && ( i <= iterations )) {

		//vector<Cfg> U;
 		U.clear();
          	i++; cout << "\ni=" << i << "  \n ";
	        //cout << "inside while not same cc";
	      if ((i % 2 )== 0) {
		//Cfg tmp = Cfg::GetRandomCfg(submap1.environment);
		//Cfg x_rand = Cfg(tmp);
		//U.push_back(x_rand);
                
    		toConnect = FALSE;            
                //cout << "problem here 12btrue\n";
		RRT(&submap1, Stats,
		    1,
		    stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		    o_clearance,clearance_from_node,
		    U,cd,lp,dm,toConnect,TRUE,
		    addPartialEdge, addAllEdges);
                if (U.size() > 0)
		  cout << "between RRT calls " << U[0] << "\n";
                else cout << "between RRTcalls random config\n";
                toConnect = TRUE;
                //cout << "start of problem\n";
		RRT(&submap2, Stats,
		    1,
		    stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		    o_clearance,clearance_from_node,
		    U,cd,lp,dm,toConnect,TRUE,
		    addPartialEdge, addAllEdges);
		}
		else {
		//Cfg tmp = Cfg::GetRandomCfg(submap2.environment);
		//Cfg x_rand = Cfg(tmp);
		//U.push_back(x_rand);
		toConnect = FALSE;
                cout << "problem here 12ctrue\n";
		RRT(&submap2, Stats,
		    1,
		    stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		    o_clearance,clearance_from_node,
		    U,cd,lp,dm,toConnect,TRUE,
		    addPartialEdge, addAllEdges);
		if (U.size() > 0)
                  cout << "between RRT calls " << U[0] << "\n";
                else cout << "between RRTcalls random config\n";
                toConnect = TRUE;
                //cout << "start of problem\n";
		RRT(&submap1, Stats,
		    1,
		    stepFactor * _rm->GetEnvironment()->GetPositionRes(),
		    o_clearance,clearance_from_node,
		    U,cd,lp,dm,toConnect,TRUE,
		    addPartialEdge, addAllEdges);
		}
		//U.erase(U.begin());
                
	  } //end while (!toConnect && ( i <= _cn.GetIterations() )
	      //if(!IsSameCC(*(_rm->m_pRoadmap),cc1id,cc2id)) {
		cout << "Modifying map\n";
      		vector<VID> vertsa;
      		(&submap1)->m_pRoadmap->GetVerticesVID(vertsa);
                ModifyRoadMap(_rm,&submap1, vertsa);
      		vector<VID> vertsb;  //not sure here if cc2 should be cc2 or vice versa
      	        (&submap2)->m_pRoadmap->GetVerticesVID(vertsb);
                ModifyRoadMap(_rm,&submap2, vertsb);
		//}
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
