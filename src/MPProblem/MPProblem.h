#ifndef MPProblem_h
#define MPProblem_h

#include "MPUtils.h"
#include "CfgTypes.h"
//#include "NeighborhoodFinder.h"

class MPStrategy;
class DistanceMetric;
class NeighborhoodFinder;
class CollisionDetection;
template<typename CFG> class ValidityChecker;
class Environment;
template <class CFG, class WEIGHT> class MPRegion;

class MPProblem : public MPBaseObject
{
public:
MPProblem(Environment* _m_pEnvironment, DistanceMetric* _m_pDistanceMetric, CollisionDetection* _m_pCollisionDetection, ValidityChecker<CfgType>* _m_pValidityChecker, NeighborhoodFinder* _m_pNeighborhoodFinder);
  MPProblem(XMLNodeReader& in_Node, bool parse_xml = true);
  virtual ~MPProblem() {}
  
  void PrintOptions();
 
protected:
  bool ParseChild(XMLNodeReader::childiterator citr);
  
private:
  ///\todo Create constructors for distance_metrics, collision_detection, MPRegions
  virtual void ParseXML(XMLNodeReader& in_Node); 
  
public:
  ///\todo Finish these interfaces.
 // void WriteRoadmap();
 
  inline MPStrategy* GetMPStrategy() {return m_pMPStrategy;};
  inline void SetMPStrategy(MPStrategy* in_pStrategy) {m_pMPStrategy = in_pStrategy;};
  inline DistanceMetric* GetDistanceMetric() {return m_pDistanceMetric;};
  inline void SetDistanceMetric(DistanceMetric* _dm){m_pDistanceMetric = _dm;};
  inline NeighborhoodFinder* GetNeighborhoodFinder() {return m_pNeighborhoodFinder;};
  inline void SetNeighborhoodFinder(NeighborhoodFinder* _nf) {m_pNeighborhoodFinder = _nf;};
  inline CollisionDetection* GetCollisionDetection() {return m_pCollisionDetection;};
  inline void SetCollisionDetection(CollisionDetection* _cd) {m_pCollisionDetection = _cd;};
  inline ValidityChecker<CfgType>* GetValidityChecker() {return m_pValidityChecker;};
  inline void SetValidityChecker(ValidityChecker<CfgType>* _vc) {m_pValidityChecker = _vc;};
  inline Environment* GetEnvironment() {return m_pEnvironment;};
  inline void SetEnvironment(Environment* _e) {m_pEnvironment = _e;};
  
  
  //inline Roadmap<CfgType,WeightType>* GetRoadmap() {return &rmp;};
  //inline Roadmap<CfgType,WeightType>* GetColRoadmap() {return &rmp_col;};
  virtual int CreateMPRegion();
  MPRegion<CfgType,WeightType>* GetMPRegion(int);

  //void AddToRoadmap(vector<Cfg_free >& in_Cfgs);
  void PrintOptions(ostream& out_os);
  //ostream& GetFileStreamByLabel(string& in_strLabel);
 
  void SetMPProblem();
////////////
//
//Data
//
//////////////
  protected:
  MPStrategy* m_pMPStrategy;
  Environment* m_pEnvironment;
  DistanceMetric*     m_pDistanceMetric;
  CollisionDetection* m_pCollisionDetection;
  ValidityChecker<CfgType>* m_pValidityChecker;
  NeighborhoodFinder* m_pNeighborhoodFinder;
  //Roadmap<CfgType,WeightType> rmp;
  //Roadmap<CfgType,WeightType> rmp_col;
  vector< MPRegion<CfgType,WeightType>* > m_vecMPRegions; 
  //map<string,MPFileIO> m_mapLabelFile;
  // temporary variable to deal with posDOFs() and DOFs()
};


class Cfg_reach_cc;
#include "boost/utility/enable_if.hpp"
#include "boost/type_traits/is_same.hpp"
#include "boost/type_traits/is_base_of.hpp"
#include "boost/mpl/or.hpp"

template <typename T>
struct IsClosedChain : boost::mpl::or_<
                                       boost::is_same<Cfg_reach_cc, T>,
                                       boost::is_base_of<Cfg_reach_cc, T>
                                      >::type {
};

#endif
