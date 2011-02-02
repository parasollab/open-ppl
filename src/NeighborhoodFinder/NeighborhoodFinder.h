
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
#include "DistanceMetricMethod.h"
#include "util.h"
#include "Clock_Class.h"
#include "BFNF.hpp"
#include "BFFNF.hpp"
#include "DPESNF.hpp"
#include "MPNNNF.hpp"
#include "CGALNF.hpp"
#include "STNF.hpp"
#include "MTNF.hpp"
#include "BandsNF.hpp" 
//#include "DMNF.hpp"
#include "PMPL_Container_Base.h"
#include "CfgTypes.h"

#include <functional>

/////////////////////////////////////////////////////////////////////////////////////////

namespace pmpl_detail { //hide NeighborhoodFinderMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
    BFNF<CfgType,WeightType>,
    BFFNF<CfgType,WeightType>,
    DPESNF<CfgType,WeightType>,
    MPNNNF<CfgType,WeightType>,
    CGALNF<CfgType,WeightType>,
    STNF<CfgType,WeightType>, 
    MTNF<CfgType,WeightType>,
    BandsNF<CfgType,WeightType> 
   // DMNF<CfgType,WeightType>
    > NeighborhoodFinderMethodList;
}

/**This is the main distance metric class.  It contains two vectors: all 
  *and selected.  all contains all of the different types of distance 
  *metric methods.  selected contains only those selected by the user.
  */
class NeighborhoodFinder : private PMPL_Container_Base< NeighborhoodFinderMethod, 
                    pmpl_detail::NeighborhoodFinderMethodList>, public MPBaseObject {

  
private:
  typedef PMPL_Container_Base< NeighborhoodFinderMethod, pmpl_detail::NeighborhoodFinderMethodList> NeighborhoodFinderContainer;
    
  public:
    typedef NeighborhoodFinderContainer::method_pointer NeighborhoodFinderPointer;
    

  public:
 
  // Default constructor
  ///\name Constructors & Destructors
  ///
  //@{
  NeighborhoodFinder(bool check, int _k, shared_ptr<DistanceMetricMethod> _dm) : check_connectivity(check), k(_k), dm(_dm){  
    //cout<<"in empty const"<<endl;
};
  NeighborhoodFinder(XMLNodeReader& in_Node, MPProblem* in_pProblem)
    : MPBaseObject(in_pProblem) { 
    //cout<<"in nf parse xml"<<endl;
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(citr->getName() == "BFNF") {
        NeighborhoodFinderMethod* nf = new BFNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "DPESNF") {
        NeighborhoodFinderMethod* nf = new DPESNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "BFFNF") {
        NeighborhoodFinderMethod* nf = new BFFNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "MPNNNF") {
        NeighborhoodFinderMethod* nf = new MPNNNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));       
      } else if(citr->getName() == "CGALNF") {
        NeighborhoodFinderMethod* nf = new CGALNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "STNF") {
        NeighborhoodFinderMethod* nf = new STNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "MTNF") {
        NeighborhoodFinderMethod* nf = new MTNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "BandsNF") {
        NeighborhoodFinderMethod* nf = new BandsNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf));
      //} else if(citr->getName() == "DMNF") {
      //  NeighborhoodFinderMethod* nf = new DMNF<CfgType,WeightType>(*citr, in_pProblem);
      //  AddNFMethod(nf->GetObjectLabel(), NeighborhoodFinderPointer(nf)); //Chinwe 
      }else {
        citr->warnUnknownNode();
      }
    }
  };
  // Destructor
  ~NeighborhoodFinder() { }
  //@}
  
  ///\name Access Methods
  ///
  //@{
  NeighborhoodFinderPointer GetNFMethod(string in_strLabel) {
    NeighborhoodFinderPointer to_return = 
                            NeighborhoodFinderContainer::GetMethod(in_strLabel);
    if(to_return.get() == NULL) {
      exit(-1);
    }
    return to_return;
  }

  void AddNFMethod(string in_strLabel, NeighborhoodFinderPointer in_ptr) {
          NeighborhoodFinderContainer::AddMethod(in_strLabel, in_ptr);
  }
  //@}
  
/*

  
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os) = 0;
  virtual NeighborhoodFinder* CreateCopy() = 0;
  */
  
  ///\name KClosest methods for a range of VIDS
  ///\warning these methods are difficult to improve faster than brute-force
  ///
  //@{
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT, typename VID>
  OutputIterator
  KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int _k,
    OutputIterator _out) {
    return _KClosest(_nf, _rmp, _input_first, _input_last, _v, _k, _out,
        typename NeighborhoodFinderContainer::MethodTypes_begin(),
        typename NeighborhoodFinderContainer::MethodTypes_end());
  }
  
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int _k,
    OutputIterator _out) {
    //cout << "NeighborhoodFinder::KClosest - 1 set of InputIterators & CFG" << endl;
    return _KClosest(_nf, _rmp, _input_first, _input_last, _cfg, _k, _out,
        typename NeighborhoodFinderContainer::MethodTypes_begin(),
        typename NeighborhoodFinderContainer::MethodTypes_end());

  }
  //@}
  
  
  ///\name KClosest methods for the entire roadmap
  ///\note These are the prefered methods for KClosest computations
  ///
  //@{
  template <typename OutputIterator, typename CFG, typename WEIGHT, typename VID>
  OutputIterator
  KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    VID _v, int _k, OutputIterator _out) {
		return _KClosest(_nf, _rmp, _v, _k, _out,
        typename NeighborhoodFinderContainer::MethodTypes_begin(),
        typename NeighborhoodFinderContainer::MethodTypes_end());
	}
  
  template <typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, int _k, OutputIterator _out) {
		return _KClosest(_nf, _rmp, _cfg, _k, _out,
        typename NeighborhoodFinderContainer::MethodTypes_begin(),
        typename NeighborhoodFinderContainer::MethodTypes_end());
	}
  //@}

  // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
  // represent the kclosest pairs of VIDs between the two ranges.
  ///\name KClosestPairs methods for 2 VID ranges
  ///
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosestPairs( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp,
    InputIterator _in1_first, InputIterator _in1_last, 
    InputIterator _in2_first, InputIterator _in2_last, 
    int k, OutputIterator _out) {
    return _KClosestPairs(_nf, _rmp, _in1_first, _in1_last, _in2_first, _in2_last, k, _out,
        typename NeighborhoodFinderContainer::MethodTypes_begin(),
        typename NeighborhoodFinderContainer::MethodTypes_end());
  }

/////////
//// Begin private boost functions
/////////
private: 

	///\name Internal PrivateKClosest methods for a range of VIDS & a VID query
  ///
  //@{
  template <typename InputIterator, typename OutputIterator, typename CFG, 
          typename WEIGHT, typename First, typename Last>
  OutputIterator
  _KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int _k,
    OutputIterator _out, First, Last) {
		typedef typename boost::mpl::deref<First>::type MethodType;
		if(MethodType* finder = dynamic_cast<MethodType*>(_nf.get()))
		{
			return finder->KClosest(_rmp, _input_first, _input_last, _cfg, _k, _out);
			//cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetObjectLabel();
			return _out;
		}
		else 
		{
			typedef typename boost::mpl::next<First>::type Next;
			return _KClosest( _nf, _rmp, _input_first, _input_last, _cfg, _k, _out, Next(), Last());  
		}
	}

  template <typename InputIterator, typename OutputIterator, typename CFG, 
          typename WEIGHT, typename Last>
  OutputIterator
  _KClosest(  NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int k,
    OutputIterator _out, Last, Last) {
    cerr << "ERROR, dynamic_cast of NeighborhoodFinderMethod failed, method type not found!\n\n";
    exit(-1);
  }
	//@}


	///\name Internal PrivateKClosest methods for a range of VIDS & a CFG query
  ///
  //@{
  template <typename InputIterator, typename OutputIterator, typename CFG, 
          typename WEIGHT, typename VID, typename First, typename Last>
  OutputIterator
  _KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int _k,
    OutputIterator _out, First, Last) {
		typedef typename boost::mpl::deref<First>::type MethodType;
		if(MethodType* finder = dynamic_cast<MethodType*>(_nf.get()))
		{
			return finder->KClosest(_rmp, _input_first, _input_last, _v, _k, _out);
			cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetObjectLabel();
			return _out;
		}
		else 
		{
			typedef typename boost::mpl::next<First>::type Next;
			return _KClosest( _nf, _rmp, _input_first, _input_last, _v, _k, _out, Next(), Last());  
		}
	}

  template <typename InputIterator, typename OutputIterator, typename CFG, 
          typename WEIGHT, typename VID, typename Last>
  OutputIterator
  _KClosest(  NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, VID _v, int k,
    OutputIterator _out, Last, Last) {
    cerr << "ERROR, dynamic_cast of NeighborhoodFinderMethod failed, method type not found!\n\n";
    exit(-1);
  }
	//@}

  
  ///\name Internal PrivateKClosest methods for the entire roadmap & a VID
  ///
  //@{
  template <typename OutputIterator, typename CFG, 
          typename WEIGHT, typename VID, typename First, typename Last>
  OutputIterator
  _KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    VID _vid, int _k, OutputIterator _out, First, Last) {
		typedef typename boost::mpl::deref<First>::type MethodType;
		if(MethodType* finder = dynamic_cast<MethodType*>(_nf.get()))
		{
			return finder->KClosest(_rmp, _vid, _k, _out);
			cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetObjectLabel();
			return _out;
		}
		else 
		{
			typedef typename boost::mpl::next<First>::type Next;
			return _KClosest( _nf, _rmp, _vid, _k, _out, Next(), Last());  
		}
	}

  template <typename OutputIterator, typename CFG, 
          typename WEIGHT, typename VID, typename Last>
  OutputIterator
  _KClosest(  NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    VID _vid, int k, OutputIterator _out, Last, Last) {
    cerr << "ERROR, dynamic_cast of NeighborhoodFinderMethod failed, method type not found!\n\n";
    exit(-1);
  }
	//@}
  

  ///\name Internal PrivateKClosest methods for the entire roadmap & a CFG
  ///
  //@{
  template <typename OutputIterator, typename CFG, 
          typename WEIGHT, typename First, typename Last>
  OutputIterator
  _KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, int _k, OutputIterator _out, First, Last) {
		typedef typename boost::mpl::deref<First>::type MethodType;
		if(MethodType* finder = dynamic_cast<MethodType*>(_nf.get()))
		{
			return finder->KClosest(_rmp, _cfg, _k, _out);
			cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetObjectLabel();
			return _out;
		}
		else 
		{
			typedef typename boost::mpl::next<First>::type Next;
			return _KClosest( _nf, _rmp, _cfg, _k, _out, Next(), Last());  
		}
	}

  template <typename OutputIterator, typename CFG, 
          typename WEIGHT, typename Last>
  OutputIterator
  _KClosest(  NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, int k, OutputIterator _out, Last, Last) {
    cerr << "ERROR, dynamic_cast of NeighborhoodFinderMethod failed, method type not found!\n\n";
    exit(-1);
  }
	//@}


	///\name Internal PrivateKClosest methods for the two sets of input iterators
  ///
  //@{
  template <typename InputIterator, typename OutputIterator, typename CFG, 
          typename WEIGHT, typename First, typename Last>
  OutputIterator
  _KClosestPairs( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _in1_first, InputIterator _in1_last,
    InputIterator _in2_first, InputIterator _in2_last,
    int k, OutputIterator _out, First, Last) {
		typedef typename boost::mpl::deref<First>::type MethodType;
		if(MethodType* finder = dynamic_cast<MethodType*>(_nf.get()))
		{
			return finder->KClosestPairs(_rmp, _in1_first, _in1_last, _in2_first, _in2_last, k, _out);
      cout << "NeighborhoodFinder::_KClosestPairs 2 set InputIterator- " << _nf->GetObjectLabel();
			return _out;
		}
		else 
		{
			typedef typename boost::mpl::next<First>::type Next;
			return _KClosestPairs( _nf, _rmp, _in1_first, _in1_last, _in2_first, _in2_last, k, _out, Next(), Last());  
		}
	}

  template <typename InputIterator, typename OutputIterator, typename CFG, 
          typename WEIGHT, typename Last>
  OutputIterator
  _KClosestPairs(  NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _in1_first, InputIterator _in1_last,
    InputIterator _in2_first, InputIterator _in2_last,
    int k, OutputIterator _out, Last, Last) {
    cerr << "ERROR, dynamic_cast of NeighborhoodFinderMethod failed, method type not found!\n\n";
    exit(-1);
  }
	//@}


/////////
//// End private boost functions
/////////
public:

  // KClosest that operate over a range of CFGs to find the kclosest to another CFG
  //
  // WARNING: use this method only when working with CFGs not inside a Roadmap
  template <typename InputIterator, typename OutputIterator, typename CFG>
  OutputIterator
  KClosest( Environment* _env, 
    InputIterator _intput_first, InputIterator _input_last, CFG _cfg, int k,
    OutputIterator _out);
  
  
  
  // old interfaces
  template <class CFG, class WEIGHT, typename VID>
  vector<pair<VID,VID> >
  FindKClosestPairs(Roadmap<CFG, WEIGHT>* rm,
          vector<VID>& vec1,
          vector<VID>& vec2, int k);
  
  template <class CFG, class WEIGHT, typename VID>
  vector<pair<VID,VID> >
  FindKClosestPairs(Roadmap<CFG,WEIGHT>* rm,
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
  
  template <class CFG, class WEIGHT, typename VID>
  vector< pair<VID,VID> >
  KClosest(Roadmap<CFG,WEIGHT>* rdmp, 
     vector<CFG>& v1, vector<CFG>& v2, unsigned int k);
  
  template <class CFG, class WEIGHT, typename VID>
  vector< pair<VID,VID> >
  KClosest(Roadmap<CFG,WEIGHT> *rdmp, 
     CFG &cc, vector<CFG>& v, unsigned int k);
  
  template <class CFG, class WEIGHT, typename VID>
  vector< pair<VID,VID> >
  KUnconnectedClosest(Roadmap<CFG,WEIGHT> *rdmp, 
     CFG &cc, vector<CFG>& v, unsigned int k);
  
  template <class CFG, class WEIGHT, typename VID>
  vector<VID>
  RangeQuery(Roadmap<CFG, WEIGHT>* rm, VID in_query, double in_radius);
  
  template <class CFG, class WEIGHT, typename VID>
  vector<VID>
  RangeQuery(Roadmap<CFG, WEIGHT>* rm, CFG in_query, double in_radius);	  
  
  
  // TODO: move this to protected, provide accessor functions
  bool check_connectivity;
  int k;
  
  template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
  double 
  RFD(Roadmap<CFG,WEIGHT>* _rmp, VID in_query,
      InputIterator _KNN_first, InputIterator _KNN_last,
      InputIterator _ANN_first, InputIterator _ANN_last,
      shared_ptr <DistanceMetricMethod> dmm, double _epsilon = double(0.0));

  template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
  double
  RDE(Roadmap<CFG,WEIGHT>* _rmp, VID in_query,
    InputIterator _KNN_first, InputIterator _KNN_last,
    InputIterator _ANN_first, InputIterator _ANN_last,
    shared_ptr<DistanceMetricMethod> dmm);

  template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
  double
  EDE(Roadmap<CFG,WEIGHT>* _rmp, VID in_query,
    InputIterator _KNN_first, InputIterator _KNN_last,
    InputIterator _ANN_first, InputIterator _ANN_last,
    shared_ptr<DistanceMetricMethod> dmm);    
  
  template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
  double
  OEPS(Roadmap<CFG,WEIGHT>* _rmp, VID in_query,
    InputIterator _KNN_first, InputIterator _KNN_last,
    InputIterator _ANN_first, InputIterator _ANN_last,
    shared_ptr <DistanceMetricMethod> dmm);
  
 protected :
   shared_ptr<DistanceMetricMethod> dm;
 };






// KClosest that operate over a range of CFGs to find the kclosest to another CFG
// WARNING: use this method only when working with CFGs not inside a Roadmap
template <typename InputIterator, typename OutputIterator, typename CFG>
OutputIterator
NeighborhoodFinder::
KClosest( Environment* _env, 
	InputIterator _intput_first, InputIterator _input_last, CFG _cfg, int k,
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
  sort (kpairs_dist.begin(), kpairs_dist.end(), compare_second<CFG, double>());
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
template <class CFG, class WEIGHT, typename VID>
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
    typename vector<VID>::iterator V1, V2;
    for(V1 = vec1.begin(); V1 != vec1.end(); ++V1) {

      // initialize w/ k elements each with huge distance...
      vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(-999,-999),
              max_value));
      CFG v1 = (*(pMap->find_vertex(*V1))).property();
      for(V2 = vec2.begin(); V2 != vec2.end(); ++V2) {
  //marcom/08nov03 check if results in other functions is same
  if(*V1 == *V2)
    continue; //don't connect same
  
  double dist = dm->Distance(_env, v1, (*(pMap->find_vertex(*V2))).property());
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

    sort(kall.begin(), kall.end(), compare_second<pair<VID, VID>, double>());
    
    // now construct vector of k pairs to return (don't need distances...)
    for (int p = 0; p < kall.size(); p++)
      if (kall[p].first.first != -999 && kall[p].first.second != -999)
  pairs.push_back( kall[p].first );
  }//endif vec1 == vec2 
  
  distance_time.StopClock();
  cout << "TIME ELAPSED: " << distance_time.GetClock_SEC();
  dm->m_distance_time += distance_time.GetClock_SEC();
  return pairs;
}


//----------------------------------------------------------------------
// Given: k and ONE vectors
// Find : k pairs of closest cfgs between the input vector of cfgs
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
template <class CFG, class WEIGHT, typename VID>
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
  typename vector<VID>::iterator V1, V2;
  for(V1 = vec1.begin(); V1 != vec1.end(); ++V1) {
    // initialize w/ k elements each with huge distance...
    vector<pair<pair<VID,VID>,double> > kp(k, make_pair(make_pair(-999,-999),
              MAX_DIST));
    int max_index = 0;
    double max_value = MAX_DIST;
 
    CFG v1 = (*(pMap->find_vertex(*V1))).property();

    for(V2 = vec1.begin(); V2 != vec1.end(); ++V2) {
      if(*V1 == *V2)
  continue;

      double dist = dm->Distance(_env, v1, (*(pMap->find_vertex(*V2))).property());
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
  
    sort(kp.begin(), kp.end(), compare_second<pair<VID, VID>, double>());
  
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






template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
double
NeighborhoodFinder::
RFD(Roadmap<CFG,WEIGHT>* _rmp, VID in_query,
    InputIterator _KNN_first, InputIterator _KNN_last,
    InputIterator _ANN_first, InputIterator _ANN_last,
    shared_ptr<DistanceMetricMethod> dmm, double _epsilon){
      /*cout << "NeighborhoodFinder::RFD()" << endl;
      cout << "KNN = ";
      for(InputIterator k_itr = _KNN_first; k_itr != _KNN_last; ++k_itr) {
        cout << *k_itr << " ";
        if(!_rmp->m_pRoadmap->IsVertex(*k_itr)) {
         cout << "ERROR:::: " << *k_itr << " is not in graph ::::"; 
        }
      }
      cout << endl;
      cout << "ANN = ";
      for(InputIterator a_itr = _ANN_first; a_itr != _ANN_last; ++a_itr) {
        cout << *a_itr << " ";
        if(!_rmp->m_pRoadmap->IsVertex(*a_itr)) {
         cout << "ERROR:::: " << *a_itr << " is not in graph ::::"; 
        }
      }
      cout << endl;
   cout << "Computing RFD 1 !!" << endl;*/
   Environment* _env = _rmp->GetEnvironment();
   RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;   
      
   double rfd = 0.0, dist_a, dist_k;  
   int k = 0;     
   InputIterator V2 = _KNN_last;
   --V2;
   
   CFG k_cfg = (*(pMap->find_vertex(*V2))).property();
   CFG q_cfg = (*(pMap->find_vertex(in_query))).property();

   dist_k = dmm->Distance(_env, q_cfg, k_cfg);
   dist_k = (1 + _epsilon)*dist_k;
   //cout << "epsilon is " << _epsilon << endl;
 
   InputIterator V1;
   for(V1 = _ANN_first; V1 != _ANN_last; ++V1){     
     CFG a_cfg = (*(pMap->find_vertex(*V1))).property();
     dist_a = dmm->Distance(_env, q_cfg, a_cfg);
     if(dist_a > dist_k){
      ++rfd; 
     }     
     ++k;
   }

   rfd = rfd / k;
   return rfd;
}


template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
double
NeighborhoodFinder::
RDE(Roadmap<CFG,WEIGHT>* _rmp, VID in_query,
    InputIterator _KNN_first, InputIterator _KNN_last,
    InputIterator _ANN_first, InputIterator _ANN_last,
    shared_ptr<DistanceMetricMethod> dmm){
    
   Environment* _env = _rmp->GetEnvironment();
   RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;  
   
   double rde = 0.0, dist_a = 0.0, dist_k = 0.0;  
   CFG q_cfg = (*(pMap->find_vertex(in_query))).property();
   
   InputIterator V1;
   for(V1 = _ANN_first; V1 != _ANN_last; ++V1){
    CFG a_cfg = (*(pMap->find_vertex(*V1))).property();
    dist_a += dmm->Distance(_env, q_cfg, a_cfg);
   }  
   
   InputIterator V2;
   for(V2 = _KNN_first; V2 != _KNN_last; ++V2){
    CFG k_cfg = (*(pMap->find_vertex(*V2))).property();
    dist_k += dmm->Distance(_env, q_cfg, k_cfg);
   }
   
   rde = 1 - dist_k / dist_a;
   return rde;
      
}


template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
double
NeighborhoodFinder::
EDE(Roadmap<CFG,WEIGHT>* _rmp, VID in_query,
    InputIterator _KNN_first, InputIterator _KNN_last,
    InputIterator _ANN_first, InputIterator _ANN_last,
    shared_ptr<DistanceMetricMethod> dmm){
      
   Environment* _env = _rmp->GetEnvironment();
   RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;  
   
   double ede = 0.0, dist_a = 0.0, dist_k = 0.0;  
   int k = 0;
   CFG q_cfg = (*(pMap->find_vertex(in_query))).property();
   
   InputIterator V1, V2;   
   for(V1 = _KNN_first, V2 = _ANN_first; V1 != _KNN_last, V2 != _ANN_last; ++V1, ++V2){
    CFG k_cfg = (*(pMap->find_vertex(*V1))).property();
    CFG a_cfg = (*(pMap->find_vertex(*V2))).property();
    
    dist_a += dmm->Distance(_env, q_cfg, a_cfg);
    dist_k += dmm->Distance(_env, q_cfg, k_cfg);
    
    ede += dist_a / dist_k;
    ++k;
    
//    cout << "dist_a for ede is \t" << dist_a << endl;
//    cout << "dist_k for ede is \t" << dist_k << endl;
   }
   
   ede = ede / k - 1;
   
   return ede;
      
}

template <typename InputIterator, typename CFG, typename WEIGHT, typename VID>
double
NeighborhoodFinder::
OEPS(Roadmap<CFG, WEIGHT>* _rmp, VID in_query,
    InputIterator _KNN_first, InputIterator _KNN_last,
    InputIterator _ANN_first, InputIterator _ANN_last,
    shared_ptr< DistanceMetricMethod> dmm){

  Environment* _env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;  
   
  double oeps = 0.0, max_a = 0.0, max_k = 0.0;  
  CFG q_cfg = (*(pMap->find_vertex(in_query))).property();  
   
  InputIterator V1, V2;   
  for(V1 = _KNN_first, V2 = _ANN_first; V1 != _KNN_last, V2 != _ANN_last; ++V1, ++V2){
    CFG k_cfg = (*(pMap->find_vertex(*V1))).property();
    CFG a_cfg = (*(pMap->find_vertex(*V2))).property();
    
    double dist_a = dmm->Distance(_env, q_cfg, a_cfg);
    double dist_k = dmm->Distance(_env, q_cfg, k_cfg);
    
    if (dist_a > max_a)
      max_a = dist_a;
    if (dist_k > max_k)
      max_k = dist_k;
    
    //cout << "dist_a for oeps is \t" << dist_a << endl;
    //cout << "dist_k for oeps is \t" << dist_k << endl;
  }
   
  //cout << endl << "max_a for oeps is \t" << max_a << endl;
  //cout <<         "max_k for oeps is \t" << max_k << endl;
  oeps = (max_a / max_k) - 1;
   
  return oeps;
}

#endif

