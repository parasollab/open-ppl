
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
#include "MPUtils.h"
#include "MetricUtils.h"
#include "BFNF.hpp"
#include "BFFNF.hpp"
#include "DPESNF.hpp"
#include "MPNNNF.hpp"
#include "CGALNF.hpp"
#include "STNF.hpp"
#include "MTNF.hpp"
#include "BandsNF.hpp" 
//#include "DMNF.hpp"
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
class NeighborhoodFinder : private ContainerBase< NeighborhoodFinderMethod, 
                    pmpl_detail::NeighborhoodFinderMethodList>, public MPBaseObject {

  
private:
  typedef ContainerBase< NeighborhoodFinderMethod, pmpl_detail::NeighborhoodFinderMethodList> NeighborhoodFinderContainer;
    
  public:
    typedef NeighborhoodFinderContainer::MethodPointer NeighborhoodFinderPointer;
    

  public:
 
  // Default constructor
  ///\name Constructors & Destructors
  ///
  //@{
  NeighborhoodFinder() {  
    //cout<<"in empty const"<<endl;
};
  NeighborhoodFinder(XMLNodeReader& in_Node, MPProblem* in_pProblem)
    : MPBaseObject(in_pProblem) { 
    //cout<<"in nf parse xml"<<endl;
    XMLNodeReader::childiterator citr;
    for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(citr->getName() == "BFNF") {
        NeighborhoodFinderMethod* nf = new BFNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "DPESNF") {
        NeighborhoodFinderMethod* nf = new DPESNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "BFFNF") {
        NeighborhoodFinderMethod* nf = new BFFNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "MPNNNF") {
        NeighborhoodFinderMethod* nf = new MPNNNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));       
      } else if(citr->getName() == "CGALNF") {
        NeighborhoodFinderMethod* nf = new CGALNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "STNF") {
        NeighborhoodFinderMethod* nf = new STNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "MTNF") {
        NeighborhoodFinderMethod* nf = new MTNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));
      } else if(citr->getName() == "BandsNF") {
        NeighborhoodFinderMethod* nf = new BandsNF<CfgType,WeightType>(*citr, in_pProblem);
        AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf));
      //} else if(citr->getName() == "DMNF") {
      //  NeighborhoodFinderMethod* nf = new DMNF<CfgType,WeightType>(*citr, in_pProblem);
      //  AddNFMethod(nf->GetLabel(), NeighborhoodFinderPointer(nf)); //Chinwe 
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
        typename NeighborhoodFinderContainer::MethodTypesBegin(),
        typename NeighborhoodFinderContainer::MethodTypesEnd());
  }
  
  template <typename InputIterator, typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    InputIterator _input_first, InputIterator _input_last, CFG _cfg, int _k,
    OutputIterator _out) {
    //cout << "NeighborhoodFinder::KClosest - 1 set of InputIterators & CFG" << endl;
    return _KClosest(_nf, _rmp, _input_first, _input_last, _cfg, _k, _out,
        typename NeighborhoodFinderContainer::MethodTypesBegin(),
        typename NeighborhoodFinderContainer::MethodTypesEnd());

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
        typename NeighborhoodFinderContainer::MethodTypesBegin(),
        typename NeighborhoodFinderContainer::MethodTypesEnd());
	}
  
  template <typename OutputIterator, typename CFG, typename WEIGHT>
  OutputIterator
  KClosest( NeighborhoodFinderPointer _nf, Roadmap<CFG,WEIGHT>* _rmp, 
    CFG _cfg, int _k, OutputIterator _out) {
		return _KClosest(_nf, _rmp, _cfg, _k, _out,
        typename NeighborhoodFinderContainer::MethodTypesBegin(),
        typename NeighborhoodFinderContainer::MethodTypesEnd());
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
        typename NeighborhoodFinderContainer::MethodTypesBegin(),
        typename NeighborhoodFinderContainer::MethodTypesEnd());
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
			//cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetLabel();
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
			cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetLabel();
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
			cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetLabel();
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
			cout << "NeighborhoodFinder::_KClosest 1 set InputIterator- " << _nf->GetLabel();
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
      cout << "NeighborhoodFinder::_KClosestPairs 2 set InputIterator- " << _nf->GetLabel();
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
};


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

