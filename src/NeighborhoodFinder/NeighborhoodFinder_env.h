/**
 * @file NeighborhoodFinder.h
 *
 * @author Bryan Boyd
 * @date 1/28/2008
 */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef NeighborhoodFinder_h
#define NeighborhoodFinder_h

//////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers

#include <functional>
#include <ANN.h>

#include "OBPRMDef.h"
#include "util.h"
#ifndef VID //from BaseGraph.h
///ID for every vertex in graph.
typedef int VID;
#endif

#include "Clock_Class.h"
/////////////////////////////////////////////////////////////////////////////////////////

class Cfg;
class MultiBody;
class Input;
class Environment;
class n_str_param;
class MPProblem;
template <class CFG, class WEIGHT> class Roadmap;


/**This is the main distance metric class.  It contains two vectors: all 
  *and selected.  all contains all of the different types of distance 
  *metric methods.  selected contains only those selected by the user.
  */
class NeighborhoodFinder : MPBaseObject {
 public:
 
  // Default constructor
  NeighborhoodFinder() { };
  NeighborhoodFinder(DistanceMetric* dm, MPProblem* in_pProblem)
    : MPBaseObject(in_pProblem) { this->dm = dm; };
  NeighborhoodFinder(TiXmlNode* in_pNode, MPProblem* in_pProblem) { };
  // Destructor
  ~NeighborhoodFinder();
/*
  //////////////////////
  // Access
  virtual char* GetName() = 0;
  virtual void SetDefault();
  virtual int GetNextNodeIndex() = 0;
  virtual void SetNextNodeIndex(int) = 0;
  virtual void IncreaseNextNodeIndex(int) = 0;
  virtual void ParseXML(TiXmlNode* in_pNode) = 0;
  void ParseXMLnum_nodes(TiXmlNode* in_pNode);
  
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os) = 0;
  virtual NeighborhoodFinder* CreateCopy() = 0;
  */
  
  //Assumed Class member variables

  
  // KClosest that operate over a range of VIDS to find the kclosest to a VID or CFG
  //
  // WARNING: these methods are difficult to improve faster than brute-force
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosest( const Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v,
    OutputIterator _out);
  
  // do the work here, and have the function above obtain the CFG and call this one
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, 
    OutputIterator _out);
  /*
  
  // Bryan will work on these two...
  
  // KClosest that operate over the entire roadmap to find the kclosest to a VID or CFG
  //
  // NOTE: These are the prefered methods for kClosest computations
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosest( const Roadmap<CFG,WEIGHT>* _rmp, 
    VID _v, OutputIterator _out);
  
  template <typename InputIterator, typename OutputIterator, typename
  CFG, typename WEIGHT>
  OutputIterator
  KClosest( const Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, OutputIterator _out);
  
  */
  
  // Xiabing will work on this one...
  
  // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
  // represent the kclosest pairs of VIDs between the two ranges.
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
 // vector<pair<VID,VID> >
  KClosestPairs(Roadmap<CFG,WEIGHT>* _rmp,
    InputIterator _in1_first, InputIterator _in1_last, 
    InputIterator _in2_first, InputIterator _in2_last, 
    OutputIterator _out);
  
  // KClosest that operate over a range of CFGs to find the kclosest to another CFG
  //
  // WARNING: use this method only when working with CFGs not inside a Roadmap
  template <typename InputIterator, typename OutputIterator, typename CFG>
  OutputIterator
  KClosest( Environment* _env, 
    InputIterator _intput_first, InputIterator _input_last, CFG _cfg, 
    OutputIterator _out);
  
  
  
  
  // old interfaces
  template <class CFG, class WEIGHT>
  vector<pair<VID,VID> >
  FindKClosestPairs( Roadmap<CFG, WEIGHT>* rm,
          vector<VID>& vec1,
          vector<VID>& vec2, int k);
  
  template <class CFG, class WEIGHT>
  vector<pair<VID,VID> >
  FindKClosestPairs( Roadmap<CFG,WEIGHT>* rm,
          vector<VID>& vec1, int k);
  
  template <class CFG>
  vector< pair<CFG,CFG> >
  KClosest(Environment *env, 
     vector<CFG>& v1, vector<CFG>& v2, unsigned int k);
  
  template <class CFG>
  vector< int >
  KClosestByIndex(Environment *env, 
      CFG &cc, vector<CFG>& v, unsigned int k);
  
  template <class CFG>
  vector< pair<CFG,CFG> >
  KClosest(Environment *env, 
     CFG &cc, vector<CFG>& v, unsigned int k);
  
  template <class CFG, class WEIGHT>
  vector< pair<VID,VID> >
  KClosest(Roadmap<CFG,WEIGHT>* rdmp, 
     vector<CFG>& v1, vector<CFG>& v2, unsigned int k);
  
  template <class CFG, class WEIGHT>
  vector< pair<VID,VID> >
  KClosest(Roadmap<CFG,WEIGHT> *rdmp, 
     CFG &cc, vector<CFG>& v, unsigned int k);
  
  template <class CFG, class WEIGHT>
  vector< pair<VID,VID> >
  KUnconnectedClosest(Roadmap<CFG,WEIGHT> *rdmp, 
     CFG &cc, vector<CFG>& v, unsigned int k);
  
  template <class CFG, class WEIGHT>
  vector<VID>
  RangeQuery(Roadmap<CFG, WEIGHT>* rm, VID in_query, double in_radius);
  
  template <class CFG, class WEIGHT>
  vector<VID>
  RangeQuery(Roadmap<CFG, WEIGHT>* rm, CFG in_query, double in_radius);	  
  
  // ANN specific functions
  template <class CFG>
  void addCfgs(vector<CFG> cfgs);
  
  template <class CFG>
  void buildTree(int dim);
  
  // TODO: move this to protected, provide accessor functions
  bool check_connectivity;
  int k;
  
  protected:
    ANNkd_tree* kdTree;
    vector< vector<double> > treeNodes;
    
    DistanceMetric* dm;
};

/**Compare two distances in DIST_TYPE instances.
 *return (_cc1.second < _cc2.second)
 */
template <class T>
class T_DIST_Compare : public binary_function<const pair<T,double>,
              const pair<T,double>,
              bool> {
 public:
  bool operator()(const pair<T,double> _cc1,
      const pair<T,double> _cc2) {
    return (_cc1.second < _cc2.second);
  }
};

/*
NeighborhoodFinder::
NeighborhoodFinder() {
  
}

NeighborhoodFinder::
~NeighborhoodFinder() {
  
}
*/

template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
OutputIterator
NeighborhoodFinder::
KClosest( const Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v,
    OutputIterator _out) {

  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  CFG _v_cfg = pMap->GetData(_v);
  
  return KClosest(_rmp, _input_first, _input_last, _v_cfg, _out);
}
  
template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
OutputIterator
NeighborhoodFinder::
KClosest( Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, 
    OutputIterator _out) {

  Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;

  int max_index = 0;
  double max_value = MAX_DIST;
  vector< pair< VID, double > > closest(k, make_pair(-999, max_value));

  for (vector<int>::size_type ix = 0; ix != closest.size(); ++ix)
      cout << closest[ix].first << " " << closest[ix].second << endl;
  cout << "\t\tK:" << k << endl;
  
  // now go through all kp and find closest k
  InputIterator V1;
  int count = 0;
  for(V1 = _input_first; V1 != _input_last; ++V1) {
    ++count;
    cout << "\tcount:" << count ;
    CFG v1 = pMap->GetData(*V1);
    
    if(v1 == _cfg)
      continue; //don't connect same

    double dist = dm->Distance(_env, _cfg, v1);
    cout << "\tdist:" << dist << "\t\tmax_index.second" <<closest[max_index].second <<endl;
    if(dist < closest[max_index].second) { 
      closest[max_index] = make_pair(*V1,dist);
      cout << "\t\t\tclosest[" << max_index << "] = (" << *V1 << ", " << dist << ")" << endl; 
      max_value = dist;
  
      //search for new max_index (faster O(k) than sort O(k log k) )
      for (int p = 0; p < closest.size(); p++) {
        if (max_value < closest[p].second) {
          max_value = closest[p].second;
          max_index = p;
          cout << "\t\t\tnew max_index = " << max_index << ", value = " << max_value << endl;
        }
      }

    }
  }
 
  sort(closest.begin(), closest.end(), T_DIST_Compare<VID>());
    
  // now add VIDs from closest to
    cout << "\t\tAdding k closest" << endl;
    for (int p = 0; p < closest.size(); ++p)
      if (closest[p].first != -999)
      {
        *_out = closest[p].first;
        cout << "\t\t\t" << *_out++ << endl;
      //  ++_out;
      }
  
  distance_time.StopClock();
  dm->m_distance_time += distance_time.GetClock_SEC();
  
  return _out;
}


// KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
// represent the kclosest pairs of VIDs between the two ranges.
template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
OutputIterator
//vector<pair<VID,VID> >
NeighborhoodFinder::
// ex.
// InputIterator = vector<VID>::iterator
// OutputIterator = vector<pair<VID,VID>>::iterator
KClosestPairs(Roadmap<CFG,WEIGHT>* _rmp,
  InputIterator _in1_first, InputIterator _in1_last, 
  InputIterator _in2_first, InputIterator _in2_last, 
  OutputIterator _out) {
 
//vector<pair<VID,VID> > pairs;	  
	  
 Clock_Class distance_time;
 distance_time.StartClock("distance_time");
 
 //   if(vec1.size()==vec2.size() &&
 //	 equal(vec1.begin(), vec1.end(), vec2.begin()) ){
	 //	return FindKClosestPairs(rm, vec1, k);
 //    } else {
 Environment* _env = _rmp->GetEnvironment();
 RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
 int max_index = 0;
 double max_value = MAX_DIST;
 vector< pair< pair< VID, VID >, double > > kall;
 
 // now go through all kp and find closest k                                     
 //Old: vector<VID>::iterator V1, V2;
 InputIterator V1, V2;
 for(V1 = _in1_first; V1 != _in1_last; ++V1) {
	 
  // initialize w/ k elements each with huge distance...                        
  vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(-999,-999),
  max_value));
  CFG v1 = pMap->GetData(*V1);
  for(V2 = _in2_first; V2 != _in2_last; ++V2) {
	  //marcom/08nov03 check if results in other functions is same                      
	  if(*V1 == *V2)
		  continue; //don't connect same                                                  
	  
	  double dist = dm->Distance(_env, v1, pMap->GetData(*V2));
	  if(dist < kp[max_index].second) {
		  kp[max_index] = make_pair(make_pair(*V1,*V2),dist);
		  max_value = dist;
		  
		  //search for new max_index (faster O(k) than sort O(k log k) )                  
		  for (int p = 0; p < kp.size(); ++p) {
			  if (max_value < kp[p].second) {
				  max_value = kp[p].second;
				  max_index = p;
			  }
		  }
	  }
  }//endfor c2                                                                  
  kall.insert(kall.end(),kp.begin(),kp.end());
 }//endfor c1                                                                    
 
 sort(kall.begin(), kall.end(), DIST_Compare<VID>());
 
// now construct vector of k pairs to return (don't need distances...)          
// old one
//  for (int p = 0; p < kall.size(); ++p)
//	 if (kall[p].first.first != -999 && kall[p].first.second != -999)
//		 pairs.push_back( kall[p].first );
//     }//endif vec1 == vec2 
 
    for (int p = 0; p < k; ++p)
 	if (kall[p].first.first != -999 && kall[p].first.second != -999){
	 	  *_out = kall[p].first;
	 	  ++_out;
		  //pairs.push_back(kall[p].first);
 	 }
	 
 distance_time.StopClock();
 // TODO: do we need to keep this?  it comes from the DistanceMetrics class.       
 dm->m_distance_time += distance_time.GetClock_SEC();
 return _out;
//return pairs;
}


// KClosest that operate over a range of CFGs to find the kclosest to another CFG
// WARNING: use this method only when working with CFGs not inside a Roadmap
template <typename InputIterator, typename OutputIterator, typename CFG>
OutputIterator
NeighborhoodFinder::
KClosest( Environment* _env, 
	InputIterator _intput_first, InputIterator _input_last, CFG _cfg, 
	OutputIterator _out){
  vector<pair<CFG,CFG> > kpairs;
  kpairs.reserve(k); //it won't grow bigger than k
  //if (k<=0) //no valid number of pairs requested
  //  return kpairs;

  Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  CFG invalid;
  invalid.InvalidData(); //make an invalid all kpairs initially
  double max_value = MAX_DIST;
  vector<pair<CFG,double> > kpairs_dist(k,pair<CFG,double>(invalid,max_value));
  kpairs_dist.reserve(k);//it won't grow more than k

  int max_index = 0;
  double dist;
 // typename vector<CFG>::iterator vi;
 InputIterator vi;
 for (vi = _intput_first; vi != _input_last; ++vi) {
    if (_cfg == (*vi))
      continue; //don't check distance to same
    dist = dm->Distance(_env, _cfg, *vi);
    if (dist < kpairs_dist[max_index].second) {
      kpairs_dist[max_index] = pair<CFG,double>((*vi),dist);
      max_value = dist;
      //search for new max_index (faster O(k) than sort O(klogk))
      for (int i = 0; i < kpairs_dist.size(); ++i)
  if (max_value < kpairs_dist[i].second) {
    max_value = kpairs_dist[i].second;
    max_index = i;
  }
    }
  }
  sort (kpairs_dist.begin(), kpairs_dist.end(), CFG_DIST_COMPARE<CFG>());
  // return only cfgs
  typename vector<pair<CFG,double> >::iterator c_iter;
  for (c_iter = kpairs_dist.begin(); c_iter < kpairs_dist.end(); ++c_iter) 
    if (c_iter->first !=invalid)
      //kpairs.push_back(pair<CFG,CFG>(cc,c_iter->first));
  	*_out++ = c_iter->first;
  distance_time.StopClock();
  dm->m_distance_time += distance_time.GetClock_SEC();

  return _out;
 // return kpairs; //by construction kpairs is never larger than k  
}


//----------------------------------------------------------------------
// Given: k and TWO vectors
// Find : k pairs of closest cfgs between the two input vectors of cfgs
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector<pair<VID,VID> >
NeighborhoodFinder::
// TODO: do we want to change the method signature to pass DistanceMetric
//       or store a local copy in the class?
FindKClosestPairs(Roadmap<CFG,WEIGHT>* rm,
      vector<VID>& vec1, vector<VID>& vec2, int k) {
  vector<pair<VID,VID> > pairs;
  // if valid number of pairs requested
  if (k<=0) 
    return pairs;

  Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  if(vec1.size()==vec2.size() && 
     equal(vec1.begin(), vec1.end(), vec2.begin()) ){
    return FindKClosestPairs(rm, vec1, k);
  } else {
    Environment* _env = rm->GetEnvironment();
    RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;

    int max_index = 0;
    double max_value = MAX_DIST;
    vector< pair< pair< VID, VID >, double > > kall;

    // now go through all kp and find closest k
    vector<VID>::iterator V1, V2;
    for(V1 = vec1.begin(); V1 != vec1.end(); ++V1) {

      // initialize w/ k elements each with huge distance...
      vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(-999,-999),
              max_value));
      CFG v1 = pMap->GetData(*V1);
      for(V2 = vec2.begin(); V2 != vec2.end(); ++V2) {
  //marcom/08nov03 check if results in other functions is same
  if(*V1 == *V2)
    continue; //don't connect same
  
  double dist = dm->Distance(_env, v1, pMap->GetData(*V2));
  if(dist < kp[max_index].second) { 
    kp[max_index] = make_pair(make_pair(*V1,*V2),dist);
    max_value = dist;
    
    //search for new max_index (faster O(k) than sort O(k log k) )
    for (int p = 0; p < kp.size(); ++p) {
      if (max_value < kp[p].second) {
        max_value = kp[p].second;
        max_index = p;
      }
    }

  }
      }//endfor c2
      kall.insert(kall.end(),kp.begin(),kp.end());
    }//endfor c1

    sort(kall.begin(), kall.end(), DIST_Compare<VID>());
    
    // now construct vector of k pairs to return (don't need distances...)
    for (int p = 0; p < kall.size(); ++p)
      if (kall[p].first.first != -999 && kall[p].first.second != -999)
  pairs.push_back( kall[p].first );
  }//endif vec1 == vec2 
  
  distance_time.StopClock();
  // TODO: do we need to keep this?  it comes from the DistanceMetrics class.
  dm->m_distance_time += distance_time.GetClock_SEC();
  return pairs;
}


//----------------------------------------------------------------------
// Given: k and ONE vectors
// Find : k pairs of closest cfgs between the input vector of cfgs
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector<pair<VID,VID> >
NeighborhoodFinder::
FindKClosestPairs(Roadmap<CFG,WEIGHT>* rm,
      vector<VID>& vec1, int k) {
  vector<pair<VID,VID> > pairs;
  // if valid number of pairs requested
  if (k<=0) 
    return pairs;
  Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  Environment* _env = rm->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;
  
  // now go through all kp and find closest k
  vector<VID>::iterator V1, V2;
  for(V1 = vec1.begin(); V1 != vec1.end(); ++V1) {
    // initialize w/ k elements each with huge distance...
    vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(-999,-999),
              MAX_DIST));
    int max_index = 0;
    double max_value = MAX_DIST;
 
    CFG v1 = pMap->GetData(*V1);

    for(V2 = vec1.begin(); V2 != vec1.end(); ++V2) {
      if(*V1 == *V2)
  continue;

      double dist = dm->Distance(_env, v1, pMap->GetData(*V2));
      if(dist < kp[max_index].second) { 
  kp[max_index] = make_pair(make_pair(*V1,*V2),dist);
  max_value = dist;
  
  //search for new max_index (faster O(k) than sort O(k log k) )
  for (int p = 0; p < kp.size(); ++p) {
    if (max_value < kp[p].second) {
      max_value = kp[p].second;
      max_index = p;
    }
  }
  
      }
    }//endfor c2
  
    sort(kp.begin(), kp.end(), DIST_Compare<VID>());
  
    // now construct vector of k pairs to return (don't need distances...)
    for (int p = 0; p < k && p < kp.size(); ++p)
      if (kp[p].first.first != -999 && kp[p].first.second != -999)
  pairs.push_back( kp[p].first );

  }//endfor c1

  distance_time.StopClock();
  dm->m_distance_time += distance_time.GetClock_SEC();

  return pairs;
}




/*
//-----------------------------------------------------------------------
// Input: vectors of CFG (v1, v2) and k
// Process: for each cfg cc1 in v1, finds the k closest cfgs in v2 to cc1
// Output: vector closest of pairs of k closest
//-----------------------------------------------------------------------
template <class CFG>
vector< pair<CFG,CFG> >
NeighborhoodFinder::
KClosest(Environment *env, 
   vector<CFG>& v1, vector<CFG>& v2, unsigned int k) {
  vector< pair<CFG,CFG> > kpairs;
  vector< pair<CFG,CFG> > kpairs_i;
  typename vector<CFG>::iterator v1_i;
  for (v1_i = v1.begin(); v1_i < v1.end(); v1_i++) {
    kpairs_i = KClosest(env,(*v1_i),v2,k);
    kpairs.insert(kpairs.end(),kpairs_i.begin(),kpairs_i.end());
  }
  return kpairs;
}

// sam's function

// this function will return indices into v that are the k
// closest (for direct access outside of function)
// 
//-----------------------------------------------------------------------
// Input: CFG cc, vector of CFG v, and k
// Process: finds the k closest cfgs in v to cc (cfg_v)
// Output: vector indices into v of k closest to cc
//-----------------------------------------------------------------------
template <class CFG>
vector< int >
NeighborhoodFinder::
KClosestByIndex(Environment *env, 
		CFG &cc, vector<CFG>& v, unsigned int k) {
  vector< int > kpairs;
  kpairs.reserve(k); //it won't grow bigger than k
  if (k<=0) //no valid number of pairs requested
    return kpairs;
  
  // cout << "got k closest here" << endl;

   Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  CFG invalid;
  invalid.InvalidData(); //make an invalid all kpairs initially
  double max_value = MAX_DIST;
  vector<pair<CFG,double> > kpairs_dist(k,pair<CFG,double>(invalid,max_value));

  vector<pair<int,double> > kpairs_index_dist(k,pair<int,double>(-1, max_value));

  kpairs_dist.reserve(k);//it won't grow more than k
  kpairs_index_dist.reserve(k);//it won't grow more than k

  int max_index = 0;
  double dist;
  typename vector<CFG>::iterator vi;
  int I=0;
  for (vi = v.begin(); vi < v.end(); vi++,I++) {
    if (cc == (*vi))
      continue; //don't check distance to same
    dist = Distance(env, cc, *vi);
    if (dist < kpairs_dist[max_index].second) {
      kpairs_dist[max_index] = pair<CFG,double>((*vi),dist);
      kpairs_index_dist[max_index].first = I;
      kpairs_index_dist[max_index].second = dist;
      max_value = dist;
      //search for new max_index (faster O(k) than sort O(klogk))
      for (int i = 0; i < kpairs_dist.size(); i++)
	if (max_value < kpairs_dist[i].second) {
	  max_value = kpairs_dist[i].second;
	  max_index = i;
	}
    }
  }
  sort (kpairs_index_dist.begin(), kpairs_index_dist.end(), 
	CFG_DIST_COMPARE_INDEX<CFG>());
  // return only indices
  typename vector< pair<int,double> >::iterator c_iter;

  for (c_iter = kpairs_index_dist.begin(); 
       c_iter < kpairs_index_dist.end(); c_iter++) 

    if (c_iter->first != -1)
      kpairs.push_back(c_iter->first);

  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();

 
  return kpairs; //by construction kpairs is never larger than k  
}





// end sam's function

//-----------------------------------------------------------------------
// Input: CFG cc, vector of CFG v, and k
// Process: finds the k closest cfgs in v to cc (cfg_v)
// Output: vector of pairs (cc, cfgv) of k closest
//-----------------------------------------------------------------------
template <class CFG>
vector< pair<CFG,CFG> >
NeighborhoodFinder::
KClosest(Environment *env, 
   CFG &cc, vector<CFG>& v, unsigned int k) {
  vector<pair<CFG,CFG> > kpairs;
  kpairs.reserve(k); //it won't grow bigger than k
  if (k<=0) //no valid number of pairs requested
    return kpairs;

 Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  CFG invalid;
  invalid.InvalidData(); //make an invalid all kpairs initially
  double max_value = MAX_DIST;
  vector<pair<CFG,double> > kpairs_dist(k,pair<CFG,double>(invalid,max_value));
  kpairs_dist.reserve(k);//it won't grow more than k

  int max_index = 0;
  double dist;
  typename vector<CFG>::iterator vi;
  for (vi = v.begin(); vi < v.end(); vi++) {
    if (cc == (*vi))
      continue; //don't check distance to same
    dist = Distance(env, cc, *vi);
    if (dist < kpairs_dist[max_index].second) {
      kpairs_dist[max_index] = pair<CFG,double>((*vi),dist);
      max_value = dist;
      //search for new max_index (faster O(k) than sort O(klogk))
      for (int i = 0; i < kpairs_dist.size(); i++)
  if (max_value < kpairs_dist[i].second) {
    max_value = kpairs_dist[i].second;
    max_index = i;
  }
    }
  }
  sort (kpairs_dist.begin(), kpairs_dist.end(), CFG_DIST_COMPARE<CFG>());
  // return only cfgs
  typename vector<pair<CFG,double> >::iterator c_iter;
  for (c_iter = kpairs_dist.begin(); c_iter < kpairs_dist.end(); c_iter++) 
    if (c_iter->first !=invalid)
      kpairs.push_back(pair<CFG,CFG>(cc,c_iter->first));
 
  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();


  return kpairs; //by construction kpairs is never larger than k  
}

//-----------------------------------------------------------------------
// Input: vectors of CFG (v1, v2) and k
// Process: for each cfg cc1 in v1, finds the k closest cfgs in v2 to cc1
// Output: vector closest of pairs of k closest
//-----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector< pair<VID,VID> >
NeighborhoodFinder::
KClosest(Roadmap<CFG,WEIGHT>* rdmp, 
   vector<CFG>& v1, vector<CFG>& v2, unsigned int k) {
  vector< pair<VID,VID> > kpairs;
  vector< pair<VID,VID> > kpairs_i;
  typename vector<CFG>::iterator v1_i;
  //  for (v1_i = v1.begin(); v1_i < v1.end(); v1_i++) {
  for (v1_i = v1.end()-1; v1_i >= v1.begin(); v1_i--) {
    kpairs_i = KClosest(rdmp,(*v1_i),v2,k);
    kpairs.insert(kpairs.end(),kpairs_i.begin(),kpairs_i.end());
  }
  return kpairs;
}

//-----------------------------------------------------------------------
// Input: CFG cc, vector of CFG v, and k
// Process: finds the k closest cfgs in v to cc (cfg_v)
// Output: vector of pairs (cc, cfgv) of k closest
//-----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector< pair<VID,VID> >
NeighborhoodFinder::
KClosest(Roadmap<CFG,WEIGHT> *rdmp, 
   CFG &cc, vector<CFG>& v, unsigned int k) {
  vector<pair<VID,VID> > kpairs;
  kpairs.reserve(k); //it won't grow bigger than k
  if (k<=0) //no valid number of pairs requested
    return kpairs;

  VID cc_id = rdmp->m_pRoadmap->GetVID(cc);
  vector<pair<CFG,CFG> > kpairs_cfg = KClosest(rdmp->GetEnvironment(),
                 cc, v, k);
  typename vector<pair<CFG,CFG> >::iterator pairs_i;
  for (pairs_i = kpairs_cfg.begin(); pairs_i < kpairs_cfg.end(); pairs_i++)
    kpairs.push_back(pair<VID,VID>(cc_id, //same as GetVID(pairs_i->first
           rdmp->m_pRoadmap->GetVID(pairs_i->second)));

  return kpairs;
}


//-----------------------------------------------------------------------
// Input: CFG cc, vector of CFG v, and k
// Process: finds the k closest cfgs in v to cc (cfg_v)
// Output: vector of pairs (cc, cfgv) of k closest
//-----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector< pair<VID,VID> >
NeighborhoodFinder::
KUnconnectedClosest(Roadmap<CFG,WEIGHT> *rdmp, 
   CFG &cc, vector<CFG>& v, unsigned int k) {
  vector<pair<VID,VID> > kpairs;
  kpairs.reserve(k); //it won't grow bigger than k
  if (k<=0) //no valid number of pairs requested
    return kpairs;

  CFG invalid;
  invalid.InvalidData(); //make an invalid all kpairs initially
  double max_value = MAX_DIST;
  vector<pair<CFG,double> > kpairs_dist(k,pair<CFG,double>(invalid,max_value));
  kpairs_dist.reserve(k);//it won't grow more than k

  int max_index = 0;
  double dist;
  Environment *env = rdmp->GetEnvironment();
  typename vector<CFG>::iterator vi;
  for (vi = v.begin(); vi < v.end(); vi++) {
    if (cc == (*vi))
      continue; //don't check distance to same
    dist = Distance(env, cc, *vi);
    if (dist < kpairs_dist[max_index].second) {
#if CHECKIFSAMECC
      if (!IsSameCC(*(rdmp->m_pRoadmap), cc,(*vi))) {
#endif
  kpairs_dist[max_index] = pair<CFG,double>((*vi),dist);
  max_value = dist;
  //search for new max_index (faster O(k) than sort O(klogk))
  for (int i = 0; i < kpairs_dist.size(); i++)
    if (max_value < kpairs_dist[i].second) {
      max_value = kpairs_dist[i].second;
      max_index = i;
    }
#if CHECKIFSAMECC
      }
#endif
    }
  }
  sort (kpairs_dist.begin(), kpairs_dist.end(), CFG_DIST_COMPARE<CFG>());
  // return only cfgs
  typename vector<pair<CFG,double> >::iterator c_iter;
  VID cc_vid = rdmp->m_pRoadmap->GetVID(cc);
  for (c_iter = kpairs_dist.begin(); c_iter < kpairs_dist.end(); c_iter++) 
    if (c_iter->first !=invalid)
      kpairs.push_back(pair<VID,VID>(cc_vid,
             rdmp->m_pRoadmap->GetVID(c_iter->first)));

  cout << "K closest happens here" << endl;
 
  return kpairs; //by construction kpairs is never larger than k  
}

template <class CFG, class WEIGHT>
vector<VID> 
NeighborhoodFinder::
RangeQuery(Roadmap<CFG, WEIGHT>* rm, VID in_query, double in_radius) {
  vector<VID> returnVec;
  RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;
  Environment* _env = rm->GetEnvironment();


  Clock_Class distance_time;
  distance_time.StartClock("distance_time");


  vector<VID> vec_vids;
  pMap->GetVerticesVID(vec_vids);
  typename vector<VID>::iterator itr;
  for(itr = vec_vids.begin(); itr != vec_vids.end(); ++itr)
  {
    if(in_query == *itr) continue;
    double dist = Distance(_env, pMap->GetData(in_query), pMap->GetData(*itr));
    //cout << "Distance = " << dist << " Radius = " << in_radius << endl;
    if( dist< in_radius) {
      returnVec.push_back(*itr);
    }
  }
  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();

  return returnVec;
}


template <class CFG, class WEIGHT>
vector<VID> 
NeighborhoodFinder::
RangeQuery(Roadmap<CFG, WEIGHT>* rm, CFG in_query, double in_radius) {
  vector<VID> returnVec;
  RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;
  Environment* _env = rm->GetEnvironment();
  
  Clock_Class distance_time;
  distance_time.StartClock("distance_time");
  


  vector<VID> vec_vids;
  pMap->GetVerticesVID(vec_vids);
  typename vector<VID>::iterator itr;
  for(itr = vec_vids.begin(); itr != vec_vids.end(); ++itr)
  {
    if(in_query == pMap->GetData(*itr)) continue;
    if(Distance(_env, in_query, pMap->GetData(*itr)) < in_radius) {
      returnVec.push_back(*itr);
    }
  }
  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();

  return returnVec;
}
*/

template <class CFG>
void
NeighborhoodFinder::
addCfgs(vector<CFG> cfgs) {
  cout << "NeighborhoodFinder::addCfgs()" << endl;
  for (int i = 0; i < cfgs.size(); ++i) {
    vector<double> data = cfgs.at(i).GetData();
    treeNodes.push_back(data);
  }
}

template <class CFG>
void
NeighborhoodFinder::
buildTree(int dim) {
  cout << "NeighborhoodFinder::buildTree()" << endl;
  ANNpointArray dataPts = annAllocPts(treeNodes.size(), dim);
  
  for(int i = 0; i < treeNodes.size(); ++i) {
    //cout << i << ":" << endl;
    ANNpoint p = annAllocPt(dim);
    for (int j = 0; j < treeNodes.at(i).size(); ++j) {
      p[j] = treeNodes.at(i).at(j);
      //cout << "\t" << p[j] << endl;
    }
    dataPts[i] = p;
  }
  
  kdTree = new ANNkd_tree(dataPts, treeNodes.size(), dim);
}


#endif

