#ifndef RRTexpand_h
#define RRTexpand_h
#include "ConnectionMethod.h"

#define STEP_FACTOR  10000        // default for rrt stepFactor
#define ITERATIONS   50        // default for rrt iterations
#define SMALL_CC      3        // default for rrt and connectCCs: smallcc size
#define O_CLEARANCE   1
#define CLEARANCE_FROM_NODE 1 


template <class CFG, class WEIGHT>
class RRTexpand: public ConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  RRTexpand();
  ~RRTexpand();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods

  void ParseCommandLine(istrstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);   
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
  static void ModifyRoadMap(Roadmap<CFG, WEIGHT>* toMap, 
			    Roadmap<CFG, WEIGHT>* fromMap, 
			    vector<VID> vids);
  void RRT(Roadmap<CFG, WEIGHT>* rm, Stat_Class& Stats,
	   int K, double deltaT, int o_clearance, 
	   int clearance_from_node,vector<CFG>& U,
	   CollisionDetection*, LocalPlanners<CFG,WEIGHT>*,
	   DistanceMetric *,
	   bool & toConnect, bool connecting,
	   bool addPartialEdge, bool addAllEdges);

  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);
 
  //////////////////////
  // Data
  int iterations; 
  int stepFactor;  
  int smallcc; 
  int o_clearance;
  int clearance_from_node;
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  RRTexpand
template <class CFG, class WEIGHT>
RRTexpand<CFG,WEIGHT>::RRTexpand():ConnectionMethod<CFG,WEIGHT>() { 
  element_name = "RRTexpand"; //in ConnectCCs there is RRTexpand

  SetDefault();
}


template <class CFG, class WEIGHT>
RRTexpand<CFG,WEIGHT>::~RRTexpand() { 
}


template <class CFG, class WEIGHT>
void RRTexpand<CFG,WEIGHT>::
ParseCommandLine(istrstream &is) { 
  char c;
  SetDefault();
  try {
    c = is.peek();
    while(c == ' ' || c == '\n') {
      is.get();
      c = is.peek();
    }    
    if (c >= '0' && c <= '9') {
      if (is >> iterations) {
        if (iterations < 0)
	  throw BadUsage();

        c = is.peek();
        while(c == ' ' || c == '\n') {
          is.get();
          c = is.peek();
        }    
        if (c >= '0' && c <= '9') {      
          if (is >> stepFactor) {
	    if (stepFactor < 0)
	      throw BadUsage();
	
            c = is.peek();
            while(c == ' ' || c == '\n') {
              is.get();
              c = is.peek();
            }    
            if (c >= '0' && c <= '9') {
	      if (is >> smallcc) {
	        if (smallcc < 0)
	          throw BadUsage();
	  
                c = is.peek();
                while(c == ' ' || c == '\n') {
                  is.get();
                  c = is.peek();
                }    
                if (c >= '0' && c <= '9') {
                  if (is >> o_clearance) {
	            if (o_clearance < 0)
	              throw BadUsage();
	    
                    c = is.peek();
                    while(c == ' ' || c == '\n') {
                      is.get();
                      c = is.peek();
                    }    
                    if (c >= '0' && c <= '9') {
  	              if (is >> clearance_from_node) {
	                if (clearance_from_node < 0)
	                  throw BadUsage();
	              } else
                        throw BadUsage();
                    }

	          } else
                    throw BadUsage();
                }

	      } else
                throw BadUsage();
            }

          } else
            throw BadUsage();
        }

      } else
        throw BadUsage();
    }
  } catch (BadUsage) {
    cerr << "Error in \'"<< GetName()<<"\' parameters" << endl;
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void
RRTexpand<CFG,WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINT INT INT INT INT (iter:" << ITERATIONS
      << " factor:" << STEP_FACTOR
      << " cc:" << SMALL_CC
      << " o_clr:" << O_CLEARANCE
      << " node_clr:" << CLEARANCE_FROM_NODE
      << ")";
  _os << endl;  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
RRTexpand<CFG,WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << "(iterations: " << iterations
      << ", stepFactor: " <<  stepFactor
      << ", smallcc: " << smallcc 
      << ", o_clearance " << o_clearance
      << ", clearance_from_node: "<< clearance_from_node<< ")";
  _os << endl;
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>* 
RRTexpand<CFG,WEIGHT>::
CreateCopy() {
  RRTexpand<CFG,WEIGHT>* _copy = 
           new RRTexpand<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void RRTexpand<CFG,WEIGHT>::SetDefault() {
  iterations = ITERATIONS;
  stepFactor = STEP_FACTOR;
  smallcc    = SMALL_CC;
  o_clearance = O_CLEARANCE;
  clearance_from_node = CLEARANCE_FROM_NODE;
}

 
template <class CFG, class WEIGHT>
void RRTexpand<CFG,WEIGHT>::ConnectComponents() {
  cout << "Connecting CCs with method: RRTexpand" << endl;
  cout << "DO NOTHING" <<endl;
}


/*---------------------------------------------------------------
Copy vertices and all incident edges associated with "vids" 
from one roadmap to another
---------------------------------------------------------------*/
template <class CFG, class WEIGHT>
void
RRTexpand<CFG, WEIGHT>::
ModifyRoadMap(Roadmap<CFG, WEIGHT>* toMap,
	      Roadmap<CFG, WEIGHT>* fromMap,
	      vector<VID> vids){
  CFG t;
  int i;
  
  //Add vertex
  for (i=0;i<vids.size();++i) {
    t=fromMap->m_pRoadmap->GetData(vids[i]);
    
    toMap->m_pRoadmap->AddVertex(t);
  } //endfor i


  //-- get edges from _rm connected component and add to submap
  for (i=0;i<vids.size();++i) {
    vector< pair<pair<VID,VID>,WEIGHT> > edges; 
    fromMap->m_pRoadmap->GetOutgoingEdges(vids[i], edges);
    
    for (int j=0;j<edges.size();++j) {
      CFG t1=fromMap->m_pRoadmap->GetData(edges[j].first.first),
	t2=fromMap->m_pRoadmap->GetData(edges[j].first.second);
      
      toMap->m_pRoadmap->AddEdge(t1,t2, edges[j].second);
    } //endfor j
    
  } //endfor i

}


template<class CFG, class WEIGHT>
void 
RRTexpand<CFG,WEIGHT>::
RRT( Roadmap<CFG,WEIGHT> * rm, Stat_Class& Stats, int K, double deltaT,
     int o_clearance, int clearance_from_node, vector<CFG>&U,
     CollisionDetection* cd, LocalPlanners<CFG,WEIGHT>* lp, DistanceMetric * dm,
     bool & toConnect,bool connecting,
     bool addPartialEdge, bool addAllEdges){

  // get a local copy for brevity of notation
  Environment *env = rm->GetEnvironment();
  CFG uu,xnew;

  for (int k=0;k<K;++k){
      CFG tmp;
      tmp.GetRandomCfg(env);
      CFG x_rand = CFG(tmp);
      vector<CFG> verticesData;
      rm->m_pRoadmap->GetVerticesData(verticesData);

      //find closest Cfg in map from random cfg.
      vector<  pair<CFG, CFG> > kp;
      cout << "\nU.size() = " << U.size()<< ". ";
      if (U.size()>0) 
        kp = dm->FindKClosestPairs(env,U[0],verticesData,1);
      else
        kp = dm->FindKClosestPairs(env,x_rand,verticesData,1);

	  //if there is closet vertex.
      if (kp.size()>0) {

         CFG x_near = kp[0].second;

         CFG u;
	 bool close = FALSE;
	 int translate_or_not = lrand48();
	 //select direction
         if ( connecting ) {
             //cout << "selecting from U\n";
             if (U.size() > 0)
               u = U[0];
             else { 
                    toConnect = FALSE;
		    if(translate_or_not%2 == 0) {
		      CFG tmp_cfg = x_rand;
		      x_rand.GetPositionOrientationFrom2Cfg(tmp_cfg,x_near);
		      //cout << x_rand << endl;
		      //cout << x_near <<endl;
		    }
		    u = x_rand;
	            uu = u;
                  }
         } else {
	   if(translate_or_not%2 == 0) {
	     CFG tmp_cfg = x_rand;
	     x_rand.GetPositionOrientationFrom2Cfg(tmp_cfg,x_near);
	     //cout << x_rand << endl;
	     //cout << x_near <<endl;
	   }
	   // holonomic robot assumption in purely random selection of u
	   u = x_rand;
         }//endif connecting
         //cout << "VID#" << kp[0].first;
// Based on Marco's code which is based off Shawna's
         bool collision=false; 
         CFG lastFreeConfiguration;
         CFG cfg = CFG(x_near); //Cfg of first collision
         double positionRes = env->GetPositionRes();
         double orientationRes = env->GetOrientationRes();
         cout << "positionRes = " << positionRes << " orientationRes " << orientationRes << endl;
         int n_ticks;
         double tmpDist;
         double collisionDistance;
         double maxLength = deltaT;
         CFG dir = u; //direction of the ray
         CFG tick = cfg;//to walk through a straight line heading direction
         CFG incr;
	 incr.FindIncrement(cfg,dir,&n_ticks,positionRes,orientationRes);
         int tk = 0; // controls the number of ticks generated
	 cout << "# of ticks: "<< n_ticks << " ";
         //clearance_cfgs will be a vector containing last n_clearance cfgs

         vector<CFG> clearance_cfgs;
         typename vector<CFG>::iterator startIterator;
         while(!collision && (tk<=n_ticks)&& (dm->Distance(env,cfg,tick) < maxLength) ) {
            lastFreeConfiguration = tick;
            if ( clearance_cfgs.size() <= o_clearance )
               clearance_cfgs.push_back( CFG(lastFreeConfiguration) );
            else {
               clearance_cfgs.push_back( CFG(lastFreeConfiguration) );
               startIterator = clearance_cfgs.begin();
               clearance_cfgs.erase( startIterator );
               }
            tick.Increment(incr); //next configuration to check
            cout << "here" << tk ;
            if( (tick.isCollision(env,Stats,cd,*cdInfo)) || !(tick.InBoundingBox(env)) ) {
               collisionDistance = dm->Distance(env, cfg, tick);
               collision = true;
               }
            tk++;// increases the tick
            } // end_while
         bool attemptConnection = TRUE;
         //int clearance_from_node = 5;
         CFG x_new;
         if ( tk < (clearance_from_node + o_clearance) ) {
            if (toConnect && ( lastFreeConfiguration == u ) && connecting) {
               cout << "\ntk small but still want to connect!!!\n";
               x_new = clearance_cfgs[clearance_cfgs.size()-1];
	    }
            else { 
               cout << "\ntk " <<tk<< " too small! donot connect!\n";
               attemptConnection = FALSE;
               toConnect = FALSE;
               x_new = lastFreeConfiguration;
               } 
            }
         else {
            if (toConnect && ( lastFreeConfiguration == u )){
               cout << "\nfound configuration to connect " <<lastFreeConfiguration<< "\n";
               x_new = lastFreeConfiguration;
               }
            else {
       	       cout << "\nadding: " << clearance_cfgs[0] << "\n";
               x_new = clearance_cfgs[0];
               }
            }
	 cout << "GetCCcount" << GetCCcount(*(rm->m_pRoadmap));
	 DisplayCCStats(*(rm->m_pRoadmap),-1);

	 LPOutput<CFG,WEIGHT> lpOutput;
	 if (x_new.InBoundingBox(env) && attemptConnection
	     && !x_new.isCollision(env,Stats,cd,*cdInfo)
	     && !rm->m_pRoadmap->IsEdge(x_near,x_new)
	     && lp->IsConnected(rm->GetEnvironment(),Stats,cd,dm,x_near,x_new,&lpOutput,connectionPosRes, connectionOriRes, (!addAllEdges))) {
	        //xnew = x_new;
                bool settoConnect = FALSE;
                // add x_new and connecting edge to x_near into roadmap
                //Cfg t=Cfg(x_new);
                if (x_new == u && connecting && toConnect) {
                   x_new = u; 
                   settoConnect = TRUE;
                   if (x_new.InBoundingBox(env) && attemptConnection
                       && !x_new.isCollision(env,Stats,cd,*cdInfo)
		       && !rm->m_pRoadmap->IsEdge(x_near,x_new)
		       && lp->IsConnected(rm->GetEnvironment(),Stats,cd,dm,x_near,x_new,&lpOutput,connectionPosRes, connectionOriRes, (!addAllEdges))) {
                         cout << "configurations equal==";
                         CFG t=CFG(x_new);
                         rm->m_pRoadmap->AddVertex(t);
                         rm->m_pRoadmap->AddEdge(x_near, x_new, lpOutput.edge);
                         //rm->m_pRoadmap->AddEdge(x_near, x_new);
                         //cout << "VID" << x_new.first
                       }   
                   }
                else { CFG t=CFG(x_new);
                   settoConnect = FALSE;
                   rm->m_pRoadmap->AddVertex(t);
                   rm->m_pRoadmap->AddEdge(x_near, x_new, lpOutput.edge);
                   //rm->m_pRoadmap->AddEdge(x_near, x_new);
                   }
                cout << "GetCCcount" << GetCCcount(*(rm->m_pRoadmap)) << "\n";
                cout <<"\nfrom    " << x_near;
		cout <<"\nto added "<< x_new << "\n distance to collision: " << collisionDistance << "\n";
                cout << "distance to u: " << dm->Distance(env,x_new,u) << "  "<< rm->GetEnvironment()->GetPositionRes()<< "\n";
                if (connecting) {
                   //Cfg temp=x_new;
                   U.push_back(x_new);
                   if (toConnect && dm->Distance(env,x_new,u)<=env->GetPositionRes()){
                        //Cfg temp1 = Cfg(temp);
                        //if ( lp->IsConnected(env,cd,dm,temp1,u,info.lpsetid,&lpOutput) ) {
                        if (settoConnect) {
                           toConnect = TRUE;
                           //rm->m_pRoadmap->AddEdge(x_new, u, lpOutput.edge);
                           cout << "configuration connectable\n u  ="<<u<<"\nxnew="<<x_new<< "\n";
                           }
                        else {
                           toConnect = FALSE;
                           cout << "configuration NOTconnectable\n u  ="<<u<<"\nxnew="<<x_new<< "\n";
                           
                           }
                        }
                   else toConnect = FALSE;
                   }

           }
         else {
           toConnect = FALSE; 
           }             
      } //endif (kp.size()>0) 
      else { cout << "no closest pair " << verticesData.size();
	     xnew.GetRandomCfg(env);
	   }

  }//endfor k
}


template <class CFG, class WEIGHT>
void RRTexpand<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  // display information specific to method
  #ifndef QUIET
  cout << "RRTexpand(iterations = "<< iterations
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
  //vector< pair<int,VID> > ccvec = _rm->m_pRoadmap->GetCCStats();
  vector< pair<int,VID> > ccvec;
  GetCCStats(*(_rm->m_pRoadmap),ccvec);
  vector< pair<int,VID> >::iterator cc1=ccvec.begin();
  //-- submap = vertices & edges of current (cc1) connected component


  while (cc1 != ccvec.end()) {
    Roadmap<CFG,WEIGHT> submap1;
    submap1.environment = _rm->GetEnvironment();
    vector<VID> cc;
    GetCC(*(_rm->m_pRoadmap),(*cc1).second ,cc);
    ModifyRoadMap(&submap1,_rm,cc);
    vector<CFG> dummyU;
    if (cc.size()<= smallcc) {
      bool toConnect = FALSE;
      RRT(&submap1, Stats,
	  iterations,
	  stepFactor * connectionPosRes,
	  o_clearance, clearance_from_node,
	  dummyU,
	  cd, lp, dm, toConnect,FALSE,
	  addPartialEdge, addAllEdges);
      vector<VID> verts;
      (&submap1)->m_pRoadmap->GetVerticesVID(verts);
      //-- map = map + submap
      ModifyRoadMap(_rm,&submap1,verts);
    }
    cc1++;   submap1.environment = NULL;
  }
 
}
#endif
