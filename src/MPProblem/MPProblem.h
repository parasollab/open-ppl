#ifndef MPProblem_h
#define MPProblem_h

#include "MPUtils.h"
#include "CfgTypes.h"
#include "RoadmapGraph.h"

class MPStrategy;
class DistanceMetric;
class NeighborhoodFinder;
class ValidityChecker;
class Environment;
template <typename CFG, typename WEIGHT> class Roadmap;

class MPProblem : public MPBaseObject
{
public:
MPProblem(Environment* _m_pEnvironment, DistanceMetric* _m_pDistanceMetric, ValidityChecker* _m_pValidityChecker, NeighborhoodFinder* _m_pNeighborhoodFinder);
  MPProblem(XMLNodeReader& in_Node, bool parse_xml = true);
  virtual ~MPProblem() {}
  
  void PrintOptions();
 
protected:
  bool ParseChild(XMLNodeReader::childiterator citr);
  
private:
  ///\todo Create constructors for distance_metrics, collision_detection
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
  inline ValidityChecker* GetValidityChecker() {return m_pValidityChecker;};
  inline void SetValidityChecker(ValidityChecker* _vc) {m_pValidityChecker = _vc;};
  inline Environment* GetEnvironment() {return m_pEnvironment;};
  inline void SetEnvironment(Environment* _e) {m_pEnvironment = _e;};
  inline Roadmap<CfgType,WeightType>* GetRoadmap() { return m_roadmap; }
  inline void SetRoadmap(Roadmap<CfgType,WeightType>* _r) { m_roadmap = _r; }
  inline Roadmap<CfgType,WeightType>* GetBlockRoadmap() { return m_blockRoadmap; }
  inline Roadmap<CfgType,WeightType>* GetColRoadmap() { return m_colRoadmap; }
  inline StatClass* GetStatClass() { return m_stats; }
  
  void PrintOptions(ostream& out_os);
  //ostream& GetFileStreamByLabel(string& in_strLabel);
  
  vector<RoadmapGraph<CfgType,WeightType>::VID> AddToRoadmap(vector<CfgType>& _cfgs);

  void WriteRoadmapForVizmo(ostream& _os, vector<shared_ptr<Boundary> >* _boundaries = NULL, bool _block = false);

 
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
  ValidityChecker* m_pValidityChecker;
  NeighborhoodFinder* m_pNeighborhoodFinder;
  Roadmap<CfgType,WeightType>* m_roadmap, * m_blockRoadmap, * m_colRoadmap;
  StatClass* m_stats;
  //map<string,MPFileIO> m_mapLabelFile;
  // temporary variable to deal with posDOFs() and DOFs()
};

#endif
