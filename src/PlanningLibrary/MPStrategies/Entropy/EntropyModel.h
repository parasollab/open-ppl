#ifndef CMODEL_h
#define CMODEL_h

#include "EntropyRegionSet.h"
#include "EntropyRegion.h"
#include <string>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class DoubleWeight {
 public:
  DoubleWeight() {}
  DoubleWeight(double w) : weight(w) {}
  ~DoubleWeight() {}

  double GetWeight() const { return weight; }
  double Weight() { return GetWeight(); }
  void SetWeight(double w) { weight = w; }

  double weight;
};

ostream& operator<<(ostream& os, const DoubleWeight& dw) {
  os << dw;
  return os;
}
istream& operator>>(istream& is, DoubleWeight& dw) {
  is >> dw.weight;
  return is;
}


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class CModel {
 public:
  //typedefs
  typedef Graph<
    UG<CRegionSet<CFG>, DoubleWeight>,
    NMG<CRegionSet<CFG>, DoubleWeight>,
    WG<CRegionSet<CFG>, DoubleWeight>,
    CRegionSet<CFG>, DoubleWeight> Region_Graph;
  typedef typename Region_Graph::VI regionset_iterator;
  typedef typename Region_Graph::CVI regionset_const_iterator;
  typedef typename vector<Sample<CFG> >::iterator sample_iterator;
  typedef typename vector<Sample<CFG> >::const_iterator sample_const_iterator;

  CModel();
  ~CModel();

  void Init(Environment* _env, int K);

  void ConstructRegions( Environment* _env, DistanceMetric *dm,
			 Stat_Class& Stats, CollisionDetection* cd,
			 CDInfo* cdInfo, int kclose);

/*
  double DistanceToClosestRegion( Environment* _env, DistanceMetric *dm,
				  CFG t );
*/

  void LearnRegions(Environment* _env, DistanceMetric *dm,
		    Stat_Class& Stats, CollisionDetection* cd,
		    CDInfo* cdInfo,
		    double _LowEntropy, int _KSamples, int _Tries);

  void WriteRegionsToSpecFile(const char* filename = "regions.spec") const;

  //  void WriteRegionsPathToSpecFile(vector<int> selected_regions_index,
  void WriteRegionsPathToSpecFile(vector<pair< CRegionSet<CFG>, DoubleWeight> >& rp,
				  const char* filename = "path.spec");

  template <class WEIGHT>
  void
  WriteRegionsToMapFile(Roadmap<CFG,WEIGHT>* rm, Input* input,
			CollisionDetection* cd, DistanceMetric* dm,
			LocalPlanners<CFG,WEIGHT>* lp, const char* _fname = "regions.map") const;

  void BuildRegionMap(Environment* env, DistanceMetric* dm);

  VID ClosestFreeRegion(regionset_iterator first,
			regionset_iterator last,
			CFG qry_cfg, Environment* env,
			DistanceMetric* dm) const;
  /*
  template <class WEIGHT>
    void  ExtractRegionPath(Roadmap<CFG,WEIGHT>* rm, CFG s_cfg, CFG g_cfg, Stat_Class& Stats,
			    CollisionDetection* cd, CDInfo* cdInfo,
			    DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp);
  */
  void MergeRegions(Environment* env, Stat_Class& Stats,
		    CollisionDetection* cd, CDInfo* cdInfo, DistanceMetric* dm);

  /*
  void SetVID(const CFG& c, VID v, CRegion<CFG>& region) {
    SetVID(c, v, region, regions);
  }
  */
  void SetVID(const CFG& c, VID v, CRegion<CFG>& region,
	      vector<CRegion<CFG> >& neighbors);

  // Initial Model nodes...may want to store if in collision
  vector<CFG> model_nodes;
  //vector<CFG> initial_nodes;
  vector<bool> isCollision;
  vector<bool> isMarked;

  //network of regions
  Region_Graph region_map;

  // temp params
  double LowEntropy;
  int KSamples;
  int Tries;
};


template <class CFG>
CModel<CFG>::
CModel() {
  model_nodes.clear();
  isCollision.clear();
  region_map.EraseGraph();
}


template <class CFG>
CModel<CFG>::
~CModel() {
}


template <class CFG>
void
CModel<CFG>::
Init(Environment* _env, int K) {
  int N = K;
  int num_coll = 0;
  for(int i=0; i<N; ++i) {
    CFG tmp;
    tmp.GetRandomCfg(_env);
    model_nodes.push_back( tmp );
    isMarked.push_back( false );
  }

  //initial_nodes = model_nodes;
}


/*
template <class CFG>
double
CModel<CFG>::
DistanceToClosestRegion( Environment* _env, DistanceMetric *dm,
			 CFG t ) {
  double min_dist = 10000;

  for(int i=0; i<regions.size(); i++) {
    CFG t_i_cent = regions[i].getCenter();
    double dist = dm->Distance( _env, t, t_i_cent );
    if( dist < min_dist ) min_dist = dist;
  }

  return min_dist;
}
*/


template <class CFG>
void
CModel<CFG>::
ConstructRegions( Environment* _env, DistanceMetric *dm,
		  Stat_Class& Stats, CollisionDetection* cd,
		  CDInfo* cdInfo, int kclose) {
  // for each model_node...check initial collision
  // store so a node does not need to be checked more than once
  for(int I=0; I<model_nodes.size(); I++) {
    CFG tmp = model_nodes[I];
    bool isColl = tmp.isCollision(_env, Stats, cd, *cdInfo);
    isCollision.push_back( isColl );
  }

  int MarkedNodes = 0;
  while ( MarkedNodes < model_nodes.size() ) {

    CRegion<CFG> cregion = CRegion<CFG>();

    // randomly selected unmarked node (will be center of current region)
    int id = lrand48() % model_nodes.size();
    while( isMarked[id] )
      id = lrand48() % model_nodes.size();// id=some unmarked node
    CFG tmp = model_nodes[id];

    bool isColl = isCollision[id];
    cregion.center = tmp;
    cregion.AddNode( tmp, isColl, 0 );//will be 0 dist to center
    if( !isMarked[id] ) {
      MarkedNodes++;
      isMarked[id] = true;
    }

    // find closest K nodes (to set up initial local region)
    vector< int > kp = dm->KClosestByIndex(_env, tmp, model_nodes,kclose);
    for(int I=0; I<kp.size(); I++) {
      int t_index = kp[I];
      double dist = dm->Distance( _env, model_nodes[id],model_nodes[t_index] );
      //cout <<I<< " Distances: "<< dist << endl;
      CFG tnode = model_nodes[t_index];
      bool isColl = isCollision[t_index];
      cregion.AddNode( tnode, isColl, dist );
      if( !isMarked[t_index] ) {
	MarkedNodes++;
	isMarked[t_index] = true;
      }
    }//end for I<kp.size()

    cregion.SetRegionStats();
    CRegionSet<CFG> cregion_set = CRegionSet<CFG>(cregion);
    region_map.AddVertex(cregion_set);

  }//end while ( MarkedNodes < initial_nodes.size() )
}



template <class CFG>
void
CModel<CFG>::
WriteRegionsToSpecFile(const char* filename) const {
  ofstream outfile(filename);

  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R) {
    const vector< CRegion<CFG> >& rs =  R->data.m_RegionSet;
    int _ID = R->data.ID;
    for(int I=0; I<rs.size(); I++) {
      outfile << " region " << _ID << " "
	      << rs[I].GetStringType() << " ";

      CFG _center = rs[I].getCenter();
      for(int J=0; J<_center.posDOF(); ++J)
	outfile << _center.GetSingleParam(J) << " ";
      //outfile << rs[I].getRadius() << endl;

      CenterOfMassDistance dm;
      double d = dm.Distance(_center, rs[I].samples.begin()->node);
      for(sample_const_iterator S = rs[I].samples.begin()+1;
	  S != rs[I].samples.end(); ++S)
	d = max(d, dm.Distance(_center, S->node));
      outfile << d << endl;
      //outfile << "0.5" << endl;
    }
  }

  outfile.close();
}

template <class CFG>
template <class WEIGHT>
void
CModel<CFG>::
WriteRegionsToMapFile(Roadmap<CFG,WEIGHT>* rm, Input* input,
		      CollisionDetection* cd, DistanceMetric* dm,
		      LocalPlanners<CFG,WEIGHT>* lp, const char* _fname) const {

  WEIGHT w(1);
  /*
  //NARROW
  rm->m_pRoadmap->EraseGraph();
  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == NARROW)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i) {
	CFG center = R->data.m_RegionSet[i].center;
	VID centerVID = rm->m_pRoadmap->GetVID(center);
	if(centerVID == INVALID_VID)
	  centerVID = rm->m_pRoadmap->AddVertex(center);
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S) {
	  //if(!S->isColl) {
	    CFG node = S->node;
	    VID nodeVID = rm->m_pRoadmap->GetVID(node);
	    if(nodeVID == INVALID_VID)
	      nodeVID = rm->m_pRoadmap->AddVertex(node);
	    rm->m_pRoadmap->AddEdge(centerVID, nodeVID, w);
	    rm->m_pRoadmap->AddEdge(nodeVID, centerVID, w);
	    //}
	}
      }
  rm->WriteRoadmap(input, cd, dm, lp, "narrow.map");
  cout << "done narrow.map\n" << flush;

  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == NARROW)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i)
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S)
	  if(S->isColl) {
	    CFG node = S->node;
	    rm->m_pRoadmap->DeleteVertex(node);
	  }
  rm->WriteRoadmap(input, cd, dm, lp, "narrow_free.map");


  //SURFACE
  rm->m_pRoadmap->EraseGraph();
  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == SURFACE)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i) {
	CFG center = R->data.m_RegionSet[i].center;
	VID centerVID = rm->m_pRoadmap->GetVID(center);
	if(centerVID == INVALID_VID)
	  centerVID = rm->m_pRoadmap->AddVertex(center);
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S) {
	  //if(!S->isColl) {
	    CFG node = S->node;
	    VID nodeVID = rm->m_pRoadmap->GetVID(node);
	    if(nodeVID == INVALID_VID)
	      nodeVID = rm->m_pRoadmap->AddVertex(node);
	    rm->m_pRoadmap->AddEdge(centerVID, nodeVID, w);
	    rm->m_pRoadmap->AddEdge(nodeVID, centerVID, w);
	    //}
	}
      }
  rm->WriteRoadmap(input, cd, dm, lp, "surface.map");
  cout << "done surface.map\n" << flush;

  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == SURFACE)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i)
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S)
	  if(S->isColl) {
	    CFG node = S->node;
	    rm->m_pRoadmap->DeleteVertex(node);
	  }
  rm->WriteRoadmap(input, cd, dm, lp, "surface_free.map");


  //FREE
  rm->m_pRoadmap->EraseGraph();
  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == FREE)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i) {
	CFG center = R->data.m_RegionSet[i].center;
	VID centerVID = rm->m_pRoadmap->GetVID(center);
	if(centerVID == INVALID_VID)
	  centerVID = rm->m_pRoadmap->AddVertex(center);
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S) {
	  //if(!S->isColl) {
	    CFG node = S->node;
	    VID nodeVID = rm->m_pRoadmap->GetVID(node);
	    if(nodeVID == INVALID_VID)
	      nodeVID = rm->m_pRoadmap->AddVertex(node);
	    rm->m_pRoadmap->AddEdge(centerVID, nodeVID, w);
	    rm->m_pRoadmap->AddEdge(nodeVID, centerVID, w);
	    //}
	}
      }
  rm->WriteRoadmap(input, cd, dm, lp, "free.map");
  cout << "done free.map\n" << flush;

  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == FREE)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i)
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S)
	  if(S->isColl) {
	    CFG node = S->node;
	    rm->m_pRoadmap->DeleteVertex(node);
	  }
  rm->WriteRoadmap(input, cd, dm, lp, "free_free.map");


  //BLOCKED
  rm->m_pRoadmap->EraseGraph();
  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == BLOCKED)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i) {
	CFG center = R->data.m_RegionSet[i].center;
	VID centerVID = rm->m_pRoadmap->GetVID(center);
	if(centerVID == INVALID_VID)
	  centerVID = rm->m_pRoadmap->AddVertex(center);
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S) {
	  //if(!S->isColl) {
	    CFG node = S->node;
	    VID nodeVID = rm->m_pRoadmap->GetVID(node);
	    if(nodeVID == INVALID_VID)
	      nodeVID = rm->m_pRoadmap->AddVertex(node);
	    rm->m_pRoadmap->AddEdge(centerVID, nodeVID, w);
	    rm->m_pRoadmap->AddEdge(nodeVID, centerVID, w);
	    //}
	}
      }
  rm->WriteRoadmap(input, cd, dm, lp, "blocked.map");
  cout << "done blocked.map\n" << flush;
  */

  //UNKNOWN
  /*
  rm->m_pRoadmap->EraseGraph();
  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == UNKNOWN)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i) {
	CFG center = R->data.m_RegionSet[i].center;
	VID centerVID = rm->m_pRoadmap->GetVID(center);
	if(centerVID == INVALID_VID)
	  centerVID = rm->m_pRoadmap->AddVertex(center);
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S) {
	  //if(!S->isColl) {
	    CFG node = S->node;
	    VID nodeVID = rm->m_pRoadmap->GetVID(node);
	    if(nodeVID == INVALID_VID)
	      nodeVID = rm->m_pRoadmap->AddVertex(node);
	    rm->m_pRoadmap->AddEdge(centerVID, nodeVID, w);
	    rm->m_pRoadmap->AddEdge(nodeVID, centerVID, w);
	    //}
	}
      }
  rm->WriteRoadmap(input, cd, dm, lp, "unknown.map");
  cout << "done unknown.map\n" << flush;

  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R)
    if(R->data.type == UNKNOWN)
      for(int i=0; i<R->data.m_RegionSet.size(); ++i)
	for(sample_const_iterator S = R->data.m_RegionSet[i].samples.begin();
	    S != R->data.m_RegionSet[i].samples.end(); ++S)
	  if(S->isColl) {
	    CFG node = S->node;
	    rm->m_pRoadmap->DeleteVertex(node);
	  }
  rm->WriteRoadmap(input, cd, dm, lp, "unknown_free.map");
  */

  //INITITAL SAMPLES
/*
  rm->m_pRoadmap->EraseGraph();
  for(typename vector<CFG>::const_iterator N = model_nodes.begin(); N != model_nodes.end(); ++N) {
    CFG c = *N;
    rm->m_pRoadmap->AddVertex(c);
  }
  rm->WriteRoadmap(input, cd, dm, lp, "initial.map");
*/

  vector<CFG> initial_free, initial_coll;
  for(int i=0; i<model_nodes.size(); ++i)
    if(isCollision[i])
      initial_coll.push_back(model_nodes[i]);
    else
      initial_free.push_back(model_nodes[i]);

  //INITIAL FREE SAMPLES
  rm->m_pRoadmap->EraseGraph();
  for(typename vector<CFG>::iterator N = initial_free.begin(); N != initial_free.end(); ++N)
    rm->m_pRoadmap->AddVertex(*N);
  rm->WriteRoadmap(input, cd, dm, lp, "initial_free.map");

  //INITIAL BLOCKED_SAMPLES
  rm->m_pRoadmap->EraseGraph();
  for(typename vector<CFG>::iterator N = initial_coll.begin(); N != initial_coll.end(); ++N)
    rm->m_pRoadmap->AddVertex(*N);
  rm->WriteRoadmap(input, cd, dm, lp, "initial_coll.map");


  /*
  rm->m_pRoadmap->EraseGraph();

  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R) {
    CFG c = R->data.center;
    rm->m_pRoadmap->AddVertex(c);
  }

  WEIGHT w;
  for(regionset_const_iterator R = region_map.begin();
      R != region_map.end(); ++R) {
    vector<VID> neighbors;
    region_map.GetAdjacentVertices(R->vid, neighbors);
    for(vector<VID>::const_iterator N = neighbors.begin();
	N != neighbors.end(); ++N)
      rm->m_pRoadmap->AddEdge(R->vid, *N, w);
  }

  rm->WriteRoadmap(input, cd, dm, lp, _fname);
  */
}


template <class CFG>
void
CModel<CFG>::
//WriteRegionsPathToSpecFile(vector<int> selected_regions_index,
WriteRegionsPathToSpecFile(vector<pair< CRegionSet<CFG>, DoubleWeight> >& rp,
			   const char* filename) {
  ofstream outfile( filename );
  cout << " writing region path file. " << endl;
  //region_const_iterator R;
  //for(int I=0; I<selected_regions_index.size(); I++) {
  //R = region_map.begin() + selected_regions_index[I];
  for(int I=0; I<rp.size(); I++) {
    vector< CRegion<CFG> >& rs = rp[I].first.m_RegionSet;
    int _ID = rp[I].first.ID;
    for(int J=0; J<rs.size(); J++) {
      CFG _center = rs[J].getCenter();
      outfile << " region " << _ID << " "
	      << rs[J].GetStringType() << " ";
      for(int K=0; K<_center.posDOF(); ++K)
	outfile << _center.GetSingleParam(K) << " ";
      outfile << rs[I].getRadius() << endl;
    }//end for J<rs.size()
  }//endfor I<rp.size()

  outfile.close();
}

template <class CFG>
void
CModel<CFG>::
LearnRegions(Environment* _env, DistanceMetric *dm,
	     Stat_Class& Stats, CollisionDetection* cd, CDInfo* cdInfo,
	     double _LowEntropy, int _KSamples, int _Tries) {
  LowEntropy = _LowEntropy;
  KSamples = _KSamples;
  Tries = _Tries;
  cout << " num regions: " << region_map.get_num_vertices() << endl;
  cout << " Params: (LowEntropy,KSamples,Tries): "
       << LowEntropy << ", " << KSamples << ", " << Tries << endl;

  for(regionset_iterator R = region_map.begin(); R != region_map.end(); ++R)
    R->data.Classify(_env, Stats, cd, cdInfo, dm, LowEntropy, KSamples, Tries);
}

template <class CFG>
VID
CModel<CFG>::
ClosestFreeRegion(regionset_iterator first, regionset_iterator last,
		  CFG qry_cfg, Environment* env, DistanceMetric* dm) const {
  //CRegionSet<CFG> return_reg;
  //CRegionSet<CFG> default_reg;
  double dist = 1e6;
  double dist_all = 1e6;
  bool free_found = false;
  VID closest_vid=-1;
  VID default_vid=-1;
  for(regionset_iterator R = first; R != last; ++R) {
    //cout << "checking if " << qry_cfg << " is close to region id: " << R->data.ID << endl;

    //VID t_vid = cmodel.GetVID( *R );
    VID t_vid = R->vid;
    vector< CRegion<CFG> > & rs = R->data.m_RegionSet;
    for(int I=0; I<rs.size(); I++) {
      double t_dist = dm->Distance(env, rs[I].center, qry_cfg);
      if( t_dist < dist &&
	  (rs[I].type!=BLOCKED) ) {
	//return_reg = R->data;
	dist = t_dist;
	free_found = true;
	closest_vid = t_vid;
      }
      if( t_dist < dist_all ) {
	//default_reg = R->data;
	dist_all = t_dist;
	default_vid= t_vid;
      }
    }//endfor I<rs.size()
  }
  if( !free_found ) closest_vid = default_vid;
  if( (closest_vid == -1) && (default_vid == -1) ) {

    cout << "In CModel::ClosestFreeRegion closest and default vid still -1.\n"
	 << "--Something ugly will happen" << endl;

  }

  return closest_vid;
}

/*
template <class CFG>
template <class WEIGHT>
void
CModel<CFG>::
ExtractRegionPath(Roadmap<CFG,WEIGHT>* rm, CFG s_cfg, CFG g_cfg,
		  Stat_Class& Stats,
		  CollisionDetection* cd, CDInfo* cdInfo,
		  DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp ) {
  //This portion is now in EntroyPRM

}
*/

template <class CFG>
void
CModel<CFG>::
BuildRegionMap(Environment* env, DistanceMetric* dm) {
  //give varying weight based on region types
  DoubleWeight w_default(1);
  DoubleWeight w_freeblocked( pow(region_map.get_num_vertices(), 4.0) );
  DoubleWeight w_blockedblocked( pow(region_map.get_num_vertices(), 10.0) );
  for(regionset_iterator R = region_map.begin(); R != region_map.end(); ++R) {
    //vector<regionset_const_iterator> neighbors;
    vector<regionset_iterator> neighbors;
    //regionset_const_iterator B = region_map.begin();
    //regionset_const_iterator E = region_map.end();
    regionset_iterator B = region_map.begin();
    regionset_iterator E = region_map.end();
    R->data.GetOverlappingRegionsReference(B, E, env, dm, neighbors);
    //for(typename vector<regionset_const_iterator>::const_iterator N = neighbors.begin();
    for(typename vector<regionset_iterator>::iterator N = neighbors.begin();
	N != neighbors.end(); ++N)
      if((*N)->data.type == BLOCKED && R->data.type == BLOCKED)
	region_map.AddEdge(R->vid, (*N)->vid, w_blockedblocked);
      else
	if((*N)->data.type == BLOCKED || R->data.type == BLOCKED)
	  region_map.AddEdge(R->vid, (*N)->vid, w_freeblocked);
	else
	  region_map.AddEdge(R->vid, (*N)->vid, w_default);
  }
}
template <class CFG>
void
CModel<CFG>::
MergeRegions(Environment* env, Stat_Class& Stats,
	     CollisionDetection* cd, CDInfo* cdInfo, DistanceMetric* dm) {
  //look at each region, check its neighbors
  //if neighbor is of the same type, and merged region is of the same type,
  //merge regions, update map, update region list
  //newstyle creates blobs
  //will probably be problems if region graph already made
  cout << " Number of initial regions generated: "
       << region_map.get_num_vertices() << endl;

  for(regionset_iterator I = region_map.begin(); I != region_map.end(); ++I) {

    CRegionSet<CFG>& cur_reg = I->data;
    VID cur_reg_vid = I->vid;
    int cur_type = cur_reg.type;
    if( cur_type == BLOCKED ) continue; // no worries with these yet
    vector< CRegion<CFG> >& regions = cur_reg.m_RegionSet;
    int J=0;
    while( J < regions.size() ) {
      //for each single regionset...check
      vector<VID> vids_to_delete;
      for(regionset_iterator K=region_map.begin(); K!=region_map.end(); ++K) {

	VID next_reg_vid = K->vid;
	if( next_reg_vid == cur_reg_vid ) continue; //skip over self

	CRegionSet<CFG>& next_reg = K->data;
	if( cur_reg.type != next_reg.type ) continue;
	//just to keep it simple at first
	if( next_reg.NumSubRegions() > 1 ) continue; //skip over already made merged regions
	if( next_reg.NumSubRegions() != 1 ) {
	  cout << "Something weird happening in MergeRegions (numsub regions!=1)"<<endl;
	}

	CRegion<CFG>& next_sub_reg = next_reg.m_RegionSet[0];

	if( regions[J].Overlaps( next_sub_reg, env, dm ) ) {

	  //Add to sublist and remove from graph
	  cur_reg.AddRegion( next_sub_reg );
	  //region_map.DeleteVertex( next_reg );
	  vids_to_delete.push_back( next_reg_vid );
	}

      }//end for K
      for(int z=0; z<vids_to_delete.size(); z++) {
	region_map.DeleteVertex( vids_to_delete[z] );
      }
      J++;
    }//end for J
    cur_reg.SetRegionStats();

  }//end for main loop I


  cout << " After merge. regions generated: "
       << region_map.get_num_vertices() << endl;

}


//////////////////////////////////////////////////
/// CHANGED WAY TO MERGE
/*

template <class CFG>
void
CModel<CFG>::
MergeRegions(Environment* env, Stat_Class& Stats,
	     CollisionDetection* cd, CDInfo* cdInfo, DistanceMetric* dm) {
 //look at each region, check its neighbors
 //if neighbor is of the same type, and merged region is of the same type,
 //merge regions, update map, update region list

  vector<pair<int,int> > unmergable_regions;

  deque<VID> Q;
  for(region_const_iterator I = region_map.begin(); I != region_map.end(); ++I)
    Q.push_back(I->vid);
  while(!Q.empty()) {
    VID Rvid = Q.front(); Q.pop_front();
    CRegion<CFG>* R = region_map.GetReferenceofData(Rvid);

    vector<VID> neighbor_list;
    region_map.GetAdjacentVertices(Rvid, neighbor_list);
    for(vector<VID>::const_iterator Nvid = neighbor_list.begin();
	Nvid != neighbor_list.end(); ++Nvid) {
      CRegion<CFG>* N = region_map.GetReferenceofData(*Nvid);

      //if of the same type and haven't failed to merge before
      if((N->type == R->type) &&
	 (find(unmergable_regions.begin(), unmergable_regions.end(),
	       make_pair(min(R->ID,N->ID), max(R->ID,N->ID))) ==
	  unmergable_regions.end())) {
        //create new merged region
        CRegion<CFG> merged;
        vector<double> Rcenter = R->getCenter().GetData();
        vector<double> Ncenter = N->getCenter().GetData();
        for(int i=0; i<Rcenter.size(); ++i)
          Rcenter[i] = (Rcenter[i] + Ncenter[i])/2;
        CFG center(Rcenter);
	merged.center = center;
	merged.radius = max(R->getRadius(), N->getRadius()) +
	  dm->Distance(env, R->getCenter(), N->getCenter())/2;
	if(R->getRadius() > N.getRadius)
	  merged.radius_cfg = R->radius_cfg;
	else
	  merged.radius_cfg = N.radius_cfg;
        merged.RadiusSet = true;
        merged.AddNode(center, center.isCollision(env, Stats, cd, *cdInfo), 0);

        //add all the nodes in each region to the merged one
	for(sample_const_iterator S = R->samples.begin();
	    S != R->samples.end(); ++S)
          merged.AddNode(S->node, S->isColl,
			 dm->Distance(env, merged.center, S->node));
	for(sample_const_iterator S = N->samples.begin();
	    S != N->samples.end(); ++S)
	  merged.AddNode(S->node, S->isColl,
			 dm->Distance(env, merged.center, S->node));

        //check for extra nodes in the neighbor list
	vector<VID> inside;
        for(vector<VID>::const_iterator N2vid = neighbor_list.begin();
	    N2vid != neighbor_list.end(); ++N2vid)
          if((*Nvid != *N2vid) && (Rvid != *N2vid)) {
            CRegion<CFG>* N2 = region_map.GetReferenceofData(*N2vid);
	    if(dm->Distance(env, merged.center, N2->center) <= merged.radius) {
	      //make sure the inside region is the same as R, ow break
	      if(N2->type != R->type) {
		merged.type = N2->type;
		break;
	      }
	      inside.push_back(*N2vid);
              for(sample_const_iterator S = N2->samples.begin();
		  S != N2->samples.end(); ++S)
		merged.AddNode(S->node, S->isColl,
			       dm->Distance(env, merged.center, S->node));
	    } else {
	      for(sample_const_iterator S = N2->samples.begin();
		  S != N2->samples.end(); ++S)
		if(dm->Distance(env, merged.center, S->node) < merged.radius)
		  merged.AddNode(S->node, S->isColl,
				 dm->Distance(env, merged.center, S->node));
	    }
          }

	//(make sure didn't find an inside region of a different type)
	if(merged.type == R->type) {
	  //remove any duplicates
	  merged.samples.erase(unique(merged.samples.begin(),
				      merged.samples.end(),
				      equal_node<CFG>()),
			       merged.samples.end());

	  //update merged region statistics
	  merged.SetRegionStats();

	  //reclassify new region
	  merged.Classify(env, Stats, cd, cdInfo, dm,
			  LowEntropy, KSamples, Tries);

          //if classification is the same, replace with merged region
	  if(merged.type == R->type) {
	    merged.ID = R->ID;

	    region_map.DeleteVertex(Rvid);

	    region_map.DeleteVertex(*Nvid);
	    Q.erase(remove(Q.begin(), Q.end(), *Nvid), Q.end());

	    for(typename vector<VID>::const_iterator I = inside.begin();
		I != inside.end(); ++I) {
	      region_map.DeleteVertex(*I);
	      Q.erase(remove(Q.begin(), Q.end(), *I), Q.end());
	    }

	    VID mergedvid = region_map.AddVertex(merged);

	    vector<region_const_iterator> new_neighbors;
	    region_const_iterator B = region_map.begin();
	    region_const_iterator E = region_map.end();
	    merged.GetOverlappingRegionsReference(B, E,
						  env, dm, new_neighbors);
	    DoubleWeight w_default(1);
	    DoubleWeight w_freeblocked(pow(region_map.Get VertexCount(), 4.0));
	    DoubleWeight w_blockedblocked(pow(region_map.Get VertexCount(), 10.0));
	    for(typename vector<region_const_iterator>::const_iterator new_N = new_neighbors.begin();
		new_N != new_neighbors.end(); ++new_N) {
	      if((*new_N)->data.type == BLOCKED && merged.type == BLOCKED)
		region_map.AddEdge(mergedvid, (*new_N)->vid, w_blockedblocked);
	      else
		if((*new_N)->data.type == BLOCKED || merged.type == BLOCKED)
		  region_map.AddEdge(mergedvid, (*new_N)->vid, w_freeblocked);
		else
		  region_map.AddEdge(mergedvid, (*new_N)->vid, w_default);
	    }
	    Q.push_front(mergedvid);
	    break; //break out of for loop
	  } else {
	    unmergable_regions.push_back(make_pair(min(R->ID,N->ID),
						   max(R->ID,N->ID)));
	  }
	} else {
	  unmergable_regions.push_back(make_pair(min(R->ID,N->ID),
						 max(R->ID,N->ID)));
	}
      } //end if(N->type == R->type)
    } //end for(neighbor_list)
  } //end while(!Q.empty())

  //for testing
  cout << " Number of regions after merge: "
       << region_map.Get VertexCount() << endl;
  for(region_iterator R = region_map.begin(); R != region_map.end(); ++R)
    cout << " Region: " << R->data.ID << "\t size: " << R->data.size()
         << "\t entropy: " << R->data.entropy
         << "\t radius: " << R->data.radius
         << "\t type : " << R->data.GetStringType()<< endl;
}
*/
/// END CHANGE OF MERGING
///////////////////////////////////////////////////////////

template <class CFG>
void
CModel<CFG>::
SetVID(const CFG& c, VID v, CRegion<CFG>& region,
       vector<CRegion<CFG> >& neighbors) {
  region.SetVID(c, v);
  for(typename vector<CRegion<CFG> >::iterator N = neighbors.begin();
      N != neighbors.end(); ++N)
    N->SetVID(c, v);
}

#endif
