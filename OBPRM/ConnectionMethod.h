#ifndef ConnectionMethod_h
#define ConnectionMethod_h
#include "DistanceMetrics.h"
#include "LocalPlanners.h"

#ifdef __K2
  #include <strstream.h>
#else
#ifdef __HP_aCC
  #include <strstream.h>
#else
  #include <strstream>
#endif
#endif

//#define MAX_CN 10
const double MAX_DIST =  1e10;

// Abstract Interface Class for connection methods
template <class CFG, class WEIGHT>
class ConnectionMethod { 
 public:

  //////////////////////
  // Constructors and Destructor
  ConnectionMethod();
  ~ConnectionMethod();

  //////////////////////
  // Access
  //virtual string GetName();
  virtual char* GetName();
  virtual void SetDefault() = 0;

  //////////////////////
  // I/O methods

  virtual void ParseCommandLine(istrstream& is) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;   
  virtual ConnectionMethod<CFG, WEIGHT>* CreateCopy() =0;

  /////////////////////
  // Useful functions

   /**Compare distances in 2 CfgDistType instances.
    *@note used to sort cfgs by distance
    */
  static bool CfgDist_Compare(const pair<CFG,double>, 
			       const pair<CFG,double>);
   /**Sort given list accroding to the distance from given
    *Cfg to every Cfg in the list.
    *
    *@param _cfgs A list of Cfgs. this list will be sorted at the end of this
    *function.
    *@param _cfg1 Distances will be calculated from this Cfg to every Cfgs in
    *_cfgs.
    *
    *@note Distance is calculated depends on distance metric set defined in info.
    *@note used by Query::CanConnectToCC.
    */
   void SortByDistFromCfg(Environment* _env, DistanceMetric* dm, 
			  const CFG& _cfg1, 
			  vector<CFG>&  _cfgs);

  //////////////////////
  // Core: Connection methods 
  virtual void ConnectComponents() = 0;
  virtual void ConnectComponents(Roadmap<CFG, WEIGHT>*, 
				 CollisionDetection*, 
				 DistanceMetric *,
				 LocalPlanners<CFG,WEIGHT>*,
				 bool addPartialEdge,
				 bool addAllEdges) = 0;
 protected:
  //////////////////////
  // Data
  char* element_name; //Method name in the command line

   
  //////////////////////////////////////////////////////
  // 
  // Public Data
  //
 public:
  CDInfo* cdInfo;
  double connectionPosRes, ///< Position resolution for node connection
         connectionOriRes; ///< Orientation resolution for node connection

};


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>::ConnectionMethod() {
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>::~ConnectionMethod(){
  //rdmp = NULL;
  //cd = NULL;
  //dm = NULL;
  //lp = NULL;

}

template <class CFG, class WEIGHT>
char* ConnectionMethod<CFG,WEIGHT>::GetName() { 
  return element_name; 
}

//
// used to sort cfgs by distance (CfgDistType is single cfg & distance)
//
template <class CFG, class WEIGHT>
bool
ConnectionMethod<CFG, WEIGHT>::
CfgDist_Compare (const pair<CFG,double> _cc1, const pair<CFG,double> _cc2) {
  return (_cc1.second < _cc2.second ) ;
}
//----------------------------------------------------------------------
// SortByDistFromCfg
//
// Given: ONE cfg1  and ONE vector of cfgs
// Do:    sort cfgs by distance from cfg1 (modifies cfgs)
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
void
ConnectionMethod<CFG, WEIGHT>::
SortByDistFromCfg(Environment* _env, DistanceMetric* dm,
		  const CFG& _cfg1, 
		  vector<CFG>& _cfgs) {
  // compute distances from _cfg1 for all cfgs in _cfgs
  vector<pair<CFG,double> > distances;
  pair<CFG,double> tmp;
  for (int i=0; i < _cfgs.size(); i++) {
    double dist = dm->Distance(_env, (CFG*)&_cfg1, (CFG*)&_cfgs[i]);
    tmp.first = _cfgs[i];
    tmp.second = dist;
    distances.push_back(tmp);
  }

  sort (distances.begin(), distances.end(), ptr_fun(CfgDist_Compare) );
  
  // now reconstruct _cfgs vector, sorted by distances 
  for (int j=0; j < _cfgs.size(); j++) 
    _cfgs[j] = distances[j].first;
  
  return ;
}

#endif
