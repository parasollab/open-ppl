// $Id$

/**
 * @file DistanceMetrics.h
 *
 * @author Daniel Vallejo
 * @date 8/21/1998
 */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DistanceMetrics_h
#define DistanceMetrics_h

//////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers

#include <functional>

#include "OBPRMDef.h"
#include "Roadmap.h"

/////////////////////////////////////////////////////////////////////////////////////////

class Cfg;
class MultiBody;
class Input;
class Environment;
class n_str_param;

template <class CFG, class WEIGHT>
class Roadmap;

class DistanceMetricMethod;
class EuclideanDistance;
class ScaledEuclideanDistance;
class MinkowskiDistance;
class ManhattanDistance;
class CenterOfMassDistance;

const int CS = 0;   ///< Type CS: Configuration space distance metric
const int WS = 1;   ///< Type WS: Workspace distance metric 


/**This is the main distance metric class.  It contains two vectors: all 
  *and selected.  all contains all of the different types of distance 
  *metric methods.  selected contains only those selected by the user.
  */
class DistanceMetric {
 public:
  DistanceMetric();
  ~DistanceMetric();

  static vector<DistanceMetricMethod*> GetDefault();

  int ReadCommandLine(n_str_param* DMstrings[MAX_DM], int numDMs);
  void PrintUsage(ostream& _os) const;
  void PrintValues(ostream& _os) const;
  void PrintDefaults(ostream& _os) const;

  /**Read information about DistanceMetricMethods selected from file.
    *@param _fname filename for data file.
    *@see ReadDMs(istream& _myistream)
    */
  void ReadDMs(const char* _fname);
  /**Read information about DistanceMetricMethods selected from input stream.
    *@see WriteDMs for data format
    *@note if this method could not be able to understand
    *input file format, then error message will be post to 
    *standard output.
    */
  void ReadDMs(istream& _myistream);
  /**Ouput information about selected DistanceMetricMethods to file.
    *@param _fname filename for data file.
    *@see WriteDMs(ostream& _myostream)
    */
  void WriteDMs(const char* _fname) const;
  /**Ouput information about selected DistanceMetricMethods to output stream.
    *@note format: DM_NAME (a string) DM_PARMS (double, int, etc) 
    *for each DistanceMetricMethod.
    *if scaledEuclidean is encountered, then sValue will be printed.
    *if minkowski is encountered, r1, r2, and r3 will be printed.
    */
  void WriteDMs(ostream& _myostream) const;

  /**Get Distance between two Configurations.
    *The distance is calculated by DistanceMetricMethod in selected vector. 
    *
    *@param _c1 Start Cfg
    *@param _c2 End Cfg
    *@return distance metrics between _c1 and _c2.
    *
    *@note there is the framework for multiple selected distance metrics, 
    *but now it only uses the first one in the selected vector.
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  virtual double Distance(Environment* env, const Cfg* _c1, const Cfg* _c2);


  /**Find k pairs of closest Cfgs from a given Cfg to 
   *all Cfgs in a given vector.
   *
   *@param vec1 A list of Cfgs.
   *@param k Find k closest Cfgs in vec1 for a given Cfg.
   *
   *@return a list of pairs, <cfg1, n1>, <cfg1, n2>, ... , <cfg1, nk>
   *Here n1,..,nk are k Cfgs in vec1 and are k closest neighbors of cfg1.
   */
  //static 
  template <class CFG>
  vector<pair<CFG,CFG> > FindKClosestPairs(Environment* _env, 
					   CFG cfg1,
					   vector<CFG>& vec1, int k);
  /**Find k of closest Cfgs in given vector of Cfgs for each Cfg
   *in same vector.
   *Therefore the return list will contains k*n pairs.
   *Here n is number of Cgfs in vec1.
   *
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in vector
   *       -# find k closest Cfg in vec1 for c1.
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param k Find k pairs for each Cfg in vec1.
   *
   *@return A (k*n-elemet) list of pair of Cfgs, and each pair represents a path from
   *the frist Cfg to the second Cfg which has first-k-small distance between
   *the frist Cfg to all other Cfgs in vec1.
   *@see FindKClosestPairs(Roadmap *, DistanceMetric *, CNInfo& , vector<Cfg>& , int )
   *use same alogithm, but returns differnt format of list.
   */
  //static 
  template <class CFG>
  vector<pair<CFG,CFG> > FindKClosestPairs(Environment* _env, 
						  vector<CFG>& vec1, int k);
  /**Find k of closest Cfgs in given vector of Cfgs for each Cfg
   *in same vector.
   *Therefore the return list will contains k*n pairs.
   *Here n is number of Cgfs in vec1.
   *
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in vector
   *       -# find k closest Cfg in vec1 for c1.
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param k Find k pairs for each Cfg in vec1.
   *
   *@return A (k*n-elemet) list of pair of VIDs, and each pair represents a path from 
   *the frist VID to the second VID which has first-k-small distance between 
   *the frist VID to all other elements in vec1.
   */
  //static 
  template <class CFG, class WEIGHT>
  vector<pair<VID,VID> > FindKClosestPairs(Roadmap<CFG, WEIGHT>* rm,
					   vector<CFG>& vec1, int k);

  /**
   *k pairs of closest cfgs for each cfg in vec1 to all cfgs in vec2.
   *This means there will be k*n pairs returned. n in number of cfgs in 
   *vec1. k pair for each cfg in vec1.
   *The differences between this function and FindKClosestPairs
   *(Environment *,DistanceMetric * , CNInfo& info, vector<Cfg>& , vector<Cfg>& , int )
   *are type and size of return values.
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in first vector
   *		-# find k first closest cfg in second vector from c1
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param vec2 A list of Cfgs.
   *@param k Find k pairs with first k shortest distance between vec1 and vec2.
   *
   *@return A (k*n-elemet) list of pair of Cfgs, and each pair represents a path from 
   *the frist Cfg to the second Cfg which has k-small distance between all 
   *possilbe paths.
   */
  //static
  template <class CFG, class WEIGHT>
  vector<pair<VID,VID> > FindKClosestPairs(Roadmap<CFG, WEIGHT>* rm,
					   vector<CFG>& vec1,
					   vector<CFG>& vec2, int k);

  /**Find k pairs of closest Cfgs between the two input vectors of Cfgs.
   *This method check distance from every Cfg in vec1 to every Cfg in vec2.
   *The first k shorst path will be returned among these pathes.
   *Following is short Alg for calculating result list:
   *   -# for every Cfg, c1, in first vector
   *       -# for every Cfg, c2, in second vector
   *           -# if distance(c1,c2)< largest distance in return list.
   *           -# then replace largest distance by this path (c1->c2)
   *           -# sort return list.
   *           -# end if
   *       -# end for
   *   -# end for
   *
   *@param vec1 A list of Cfgs.
   *@param vec2 A list of Cfgs.
   *@param k Find k pairs with first k shortest distance between vec1 and vec2.
   *
   *@return A (k-elemet) list of pair of Cfgs, and each pair represents a path from 
   *the frist Cfg to the second  Cfg which has k-small distance between all 
   *possilbe paths.   */
  //static 
  template <class CFG>
  vector<pair<CFG,CFG> > FindKClosestPairs(Environment* _env, 
					   vector<CFG>& vec1,
					   vector<CFG>& vec2, int k);

 protected:
  bool ParseCommandLine(int argc, char** argv);

  vector<DistanceMetricMethod*> all;
  vector<DistanceMetricMethod*> selected;
};

/**Compare two distances in DIST_TYPE instances.
 *return (_cc1.second < _cc2.second)
 */
template <class CFG>
bool DIST_Compare(const pair<pair<CFG,CFG>,double> _cc1,
		  const pair<pair<CFG,CFG>,double> _cc2);


/**This is the interface for all distance metric methods(euclidean, 
  *scaledEuclidean, minkowski, manhattan, com, etc.).
  */
class DistanceMetricMethod {
 public:
  DistanceMetricMethod();
  ~DistanceMetricMethod();

  virtual char* GetName() const = 0;
  virtual void SetDefault() = 0;

  virtual bool operator==(const DistanceMetricMethod& dm) const;

  virtual void ParseCommandLine(int argc, char** argv) = 0;
  virtual void PrintUsage(ostream& _os) const = 0;
  virtual void PrintValues(ostream& _os) const = 0;
  virtual DistanceMetricMethod* CreateCopy() = 0;

  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2) = 0;
 protected:
  int type; ///<WS or CS. Used to classify metrics.
};
ostream& operator<< (ostream& _os, const DistanceMetricMethod& dm);


/**This computes the euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class EuclideanDistance : public DistanceMetricMethod {
 public:
  EuclideanDistance();
  ~EuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2....+(c1n-c2n)^2).
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);

 protected:
  virtual double ScaledDistance(const Cfg& _c1, const Cfg& _c2, double sValue);
};


/**This computes the scaled euclidean distance between two cfgs.  This 
  *class is derived off of DistanceMetricMethod.
  */
class ScaledEuclideanDistance : public EuclideanDistance {
 public:
  ScaledEuclideanDistance();
  ScaledEuclideanDistance(double _sValue);
  ~ScaledEuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual bool operator==(const ScaledEuclideanDistance& dm) const;

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *sqrt(s*(Position Magnitude)^ + (1-s)*(Orientation Magnitude)^2)
    *Position Magnitude is eulidean distance of position part of dimensions.
    *Position Magnitude is eulidean distance of orientation part of dimensions.
    *Usually first 3 dimensions are positions and rest of them are orientations.
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);

  double GetS() const { return sValue; }

 protected:
  /**Scale for Euludean distance of position part. 
    *Should between [0,1].
    */
  double sValue; ///<For Euclidean distance of position part.
};


/**This computes the minkowski distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class MinkowskiDistance : public DistanceMetricMethod {
 public:
  MinkowskiDistance();
  ~MinkowskiDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  bool operator==(const MinkowskiDistance& dm) const;

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *pow( (c11-c21)^r1+(c12-c22)^r1+(c13-c23)^r1+(c14-c24)^r2....+(c1n-c2n)^r2 , r3)
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *Usually first 3 dimensions are positions and rest of them are orientations.
    *position part using r1 as power factor, and orientation part using r2 as power factor.
    *usually, r1=r2 and r3=1/r1.
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);

  double GetR1() const { return r1; }
  double GetR2() const { return r2; }
  double GetR3() const { return r3; }

 protected:
  /**Power factors for Minkowski Distace.
    */
  double r1; ///<For position part.
  double r2; ///<For rotation part.
  double r3; ///<For calculating root.
};


/**This computes the manhattan distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class ManhattanDistance : public DistanceMetricMethod {
 public:
  ManhattanDistance();
  ~ManhattanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *( |c11-c21|+|c12-c22|+...+|c1n-c2n| ).
    *
    *Here |A| is absolute value of A.
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);
};


/**This computes the euclidean distance of only the position part between 
  *two cfgs.  This class is derived off of DistanceMetricMethod.
  */
class CenterOfMassDistance : public DistanceMetricMethod {
 public:
  CenterOfMassDistance();
  ~CenterOfMassDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2).
    *This method only Euclidean Distance of position part and 
    *assumed that the first 3 dimension of Cfg are for position.
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);
};


//
// used by "findKClosestPairs" for sort (DIST_TYPE is pair of cfgs & distance)
//
template <class CFG>
bool
DIST_Compare (const pair<pair<CFG,CFG>,double> _cc1, const pair<pair<CFG,CFG>,double> _cc2) {
  return (_cc1.second < _cc2.second ) ;
}


//----------------------------------------------------------------------
// Given: k and ONE cfg and ONE vector
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------
template <class CFG>
vector<pair<CFG,CFG> >
DistanceMetric::
FindKClosestPairs(Environment* _env, CFG cfg1, vector<CFG>& vec1, int k) {
  vector<CFG> cfg;
  cfg.push_back(cfg1);
  
  // find kclosest cfgs to cfg in vertices
  return FindKClosestPairs<CFG>( _env, cfg, vec1, k);
}

//----------------------------------------------------------------------
// Given: k and ONE vector
// Find : find k pairs of closest cfg from vector to each cfg in vector
//----------------------------------------------------------------------
template <class CFG>
vector<pair<CFG,CFG> >
DistanceMetric::
FindKClosestPairs(Environment* _env, vector<CFG>& vec1, int k) {
  vector< pair<CFG,CFG> > pairs;
  // if valid number of pairs requested
  if (k<=0) return pairs;
  
  vector<CFG> vec_of_cfgs = vec1;
  
  //for each cfg in given vector
  for( int iV1=vec1.size()-1; iV1>=0; iV1-- ){
    // find k closest cfgs between the two vectors 
    vector< pair<CFG,CFG> > kp = FindKClosestPairs<CFG>(_env,vec1[iV1],vec_of_cfgs,k);
    // save pairs
    pairs.insert(pairs.end(),kp.begin(),kp.end());
  }//endfor cfg
  
  return pairs;
}

//----------------------------------------------------------------------
// Given: k and a vector
// Find : k pairs of closest cfgs for each cfg in vector to all other cfgs in vector.
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector< pair<VID, VID> >
DistanceMetric::
FindKClosestPairs(Roadmap<CFG, WEIGHT> *rm, vector<CFG>& vec1, int k) {
  return FindKClosestPairs<CFG,WEIGHT>(rm, vec1, vec1, k);
}


//----------------------------------------------------------------------
// Given: k and a TWO vectors
// Find : k pairs of closest cfgs for each cfg in vec1 to all cfgs in vec2.
//		  This means there will be k*n pairs returned. n in number of cfgs in 
//        vec1. k pair for each cfg in vec1.
//		  The differences between this function and FindKClosestPairs
//        (Environment *,DistanceMetric * , CNInfo& info, vector<Cfg>& , vector<Cfg>& , int )
//		  are type and size of return values.
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector< pair<VID, VID> >
DistanceMetric::
FindKClosestPairs(Roadmap<CFG, WEIGHT>* rm,
		  vector<CFG>& vec1,vector<CFG>& vec2, int k) {
  vector< pair<VID, VID> > pairs;
  if (k<=0) return pairs;
  
  Environment *_env = rm->GetEnvironment();
  RoadmapGraph<CFG, WEIGHT> * pMap = rm->m_pRoadmap;
  
  //compute from the last to the first
  for( int iV1=vec1.size()-1; iV1>=0; iV1-- ){
    // find k closest cfgs
    vector< pair<CFG,CFG> > kpair = FindKClosestPairs<CFG>(_env,vec1[iV1],vec2,k); /*CHANGED */
    // save VID pairs (convert from Cfg to VID)
    for(int iKP=0; iKP<kpair.size(); ++iKP) {
      pairs.push_back(pair<VID, VID>(pMap->GetVID(kpair[iKP].first),pMap->GetVID(kpair[iKP].second)));
    }
  }
  
  return pairs;
}


//----------------------------------------------------------------------
// Given: k and a TWO vectors
// Find : k pairs of closest cfgs between the two input vectors of cfgs
// -- if k don't exist, return as many as do
//----------------------------------------------------------------------
template <class CFG>
vector<pair<CFG,CFG> > 
DistanceMetric::
FindKClosestPairs(Environment* _env, 
		  vector<CFG>& vec1, vector<CFG>& vec2, int k) {
  vector<pair<CFG,CFG> > pairs;	
  CFG invalid;
  invalid.InvalidData();
  pair<pair<CFG,CFG>,double> tmp;
  // if valid number of pairs requested
  if (k<=0) return pairs;
  
  //Modified for aCC, replace vec1==vec2
  if( vec1.size()==vec2.size() && equal(vec1.begin(), vec1.end(), vec2.begin()) ){
    return FindKClosestPairs<CFG>(_env, vec1, k);
  } else {
    // initialize w/ k elements each with huge distance...
    vector<pair<pair<CFG,CFG>,double> > kp;
    for (int i=0; i < k; i++) {
      tmp.first.first = invalid;
      tmp.first.second = invalid;
      tmp.second = MAX_DIST;
      kp.push_back(tmp);
    }
    
    // now go through all kp and find closest k
    for (int c1 = 0; c1 < vec1.size(); c1++) {
      for (int c2 = 0; c2 < vec2.size(); c2++) {
	//if( vec1[c1]==vec2[c2] ) continue; //don't connect same

	double dist = Distance(_env, vec1[c1],vec2[c2]);
	if ( dist < kp[k-1].second) {
	  tmp.first.first = vec1[c1];
	  tmp.first.second = vec2[c2];
	  tmp.second = dist;
	  kp[k-1] = tmp;
	  sort (kp.begin(), kp.end(), ptr_fun(DIST_Compare<CFG>));
	}
      }//endfor c2
    }//endfor c1
    
    // now construct vector of k pairs to return (don't need distances...)
    for (int p=0; p < k && p < kp.size(); p++)
      if (kp[p].first.first != invalid && kp[p].first.second != invalid)
	pairs.push_back( kp[p].first );		
  }//endif vec1 == vec2
  
  return pairs;
}

#endif

