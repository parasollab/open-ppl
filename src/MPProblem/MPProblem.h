#ifndef MPProblem_h
#define MPProblem_h

#include "util.h"
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
 
  void SetMPStrategy(MPStrategy* in_pStrategy) {m_pMPStrategy = in_pStrategy;};
  inline DistanceMetric* GetDistanceMetric() {return m_pDistanceMetric; };
  inline NeighborhoodFinder* GetNeighborhoodFinder() {return m_pNeighborhoodFinder; };
  inline CollisionDetection* GetCollisionDetection() {return m_pCollisionDetection; };
  inline ValidityChecker<CfgType>* GetValidityChecker() {return m_pValidityChecker; };
  inline Environment* GetEnvironment() {return m_pEnvironment; };
  inline MPStrategy* GetMPStrategy() {return m_pMPStrategy;};
  //inline Stat_Class* GetStatClass() {return m_pStatClass;};
  
  
  //inline Roadmap<CfgType,WeightType>* GetRoadmap() {return &rmp;};
  //inline Roadmap<CfgType,WeightType>* GetColRoadmap() {return &rmp_col;};
  virtual int CreateMPRegion();
  MPRegion<CfgType,WeightType>* GetMPRegion(int);

  void SetNumOfJoints(int num_of_joints) {
    CfgType::setNumofJoints(num_of_joints);
    robot_cfg = CfgType();
  }
  int GetDOFs() {return robot_cfg.DOF(); }
  int GetPosDOFs() {return robot_cfg.posDOF();}
  //void AddToRoadmap(vector<Cfg_free >& in_Cfgs);
  void PrintOptions(ostream& out_os);
  //ostream& GetFileStreamByLabel(string& in_strLabel);
  
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
  //Stat_Class* m_pStatClass;
  //map<string,MPFileIO> m_mapLabelFile;
  // temporary variable to deal with posDOFs() and DOFs()
  CfgType robot_cfg;  // @todo may want to replace by real robot class
};



#endif
