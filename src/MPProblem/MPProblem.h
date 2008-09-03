#ifndef MPProblem_h
#define MPProblem_h

#include "util.h"
#include "CfgTypes.h"

class MPStrategy;
class DistanceMetric;
class CollisionDetection;
class Environment;
template <class CFG, class WEIGHT> class MPRegion;

class MPProblem : public MPBaseObject
{
public:
  MPProblem(XMLNodeReader& in_Node);
  virtual ~MPProblem() {}
  
  void PrintOptions();
  
  
private:
  ///\todo Create constructors for distance_metrics, collision_detection, MPRegions
  virtual void ParseXML(XMLNodeReader& in_Node); 
  
public:
  ///\todo Finish these interfaces.
 // void WriteRoadmap();
 
  void SetMPStrategy(MPStrategy* in_pStrategy) {m_pMPStrategy = in_pStrategy;};
  inline DistanceMetric* GetDistanceMetric() {return m_pDistanceMetric; };
  inline CollisionDetection* GetCollisionDetection() {return m_pCollisionDetection; };
  inline Environment* GetEnvironment() {return m_pEnvironment; };
  inline MPStrategy* GetMPStrategy() {return m_pMPStrategy;};
  //inline Stat_Class* GetStatClass() {return m_pStatClass;};
  
  
  //inline Roadmap<CfgType,WeightType>* GetRoadmap() {return &rmp;};
  //inline Roadmap<CfgType,WeightType>* GetColRoadmap() {return &rmp_col;};
  int CreateMPRegion();
  MPRegion<CfgType,WeightType>* GetMPRegion(int);

  void SetNumOfJoints(int num_of_joints) {CfgType::setNumofJoints(num_of_joints);}
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
  private:
  MPStrategy* m_pMPStrategy;
  DistanceMetric*     m_pDistanceMetric;
  CollisionDetection* m_pCollisionDetection;
  Environment* m_pEnvironment;
  //Roadmap<CfgType,WeightType> rmp;
  //Roadmap<CfgType,WeightType> rmp_col;
  vector< MPRegion<CfgType,WeightType>* > m_vecMPRegions; 
  //Stat_Class* m_pStatClass;
  //map<string,MPFileIO> m_mapLabelFile;
  // temporary variable to deal with posDOFs() and DOFs()
  CfgType robot_cfg;  // @todo may want to replace by real robot class
};



#endif
