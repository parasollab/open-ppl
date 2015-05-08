#ifndef UASTRATEGY_H_
#define UASTRATEGY_H_

#include "MPStrategy.h"
#include "PartitioningMethods.h"
#include "PartitioningEvaluators.h"
#include "Features.h"

class PartitionTree;
class Partition;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class UASContainer : public MPSMContainer {
  public:
    UASContainer (MPSMContainer cont = MPSMContainer()) : MPSMContainer(cont), parent(cont) {} //Container for more readabble MPStrategyMethod constructor
    vector<string> m_EvaluatorLabels;
    string m_PartitioningMethod;
    string m_DistributionFeature;
    PartitionTree* m_pt;
    string m_TrainingStrategy;
    string m_OverlapMethod;
    vector<string> m_RegionStrategies;
    vector<vector<double> > m_min, m_max;
    vector<pair<double,double> > m_hold;
    int m_CurrentIteration;
    MPSMContainer parent;

};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class UAStrategy : public MPStrategyMethod {
 public:
   UAStrategy(UASContainer cont) : MPStrategyMethod(cont) {
      m_EvaluatorLabels = cont.m_EvaluatorLabels;
      m_PartitioningMethod = cont.m_PartitioningMethod;
      m_DistributionFeature = cont.m_DistributionFeature;
      m_pt = cont.m_pt;
      m_TrainingStrategy = cont.m_TrainingStrategy;
      m_OverlapMethod = cont.m_OverlapMethod;
      m_RegionStrategies = cont.m_RegionStrategies;
      m_min = cont.m_min;
      m_max = cont.m_max;
      m_hold = cont.m_hold;
      m_CurrentIteration = cont.m_CurrentIteration;
   }
   UAStrategy(XMLNode& in_Node, MPProblem* in_pProblem);
   virtual ~UAStrategy(){}

   virtual void ParseXML(XMLNode& in_Node);

   virtual void Initialize();
   virtual void Run();
   virtual void Finalize();

   virtual void Print(ostream& out_os) const;

 protected:
   void IdentifyRegions();
   void CollectMinMaxBBX();
   void OverlapBBX();
   void IntToStr(int myInt, string &myString);
   int GetRandRegion(vector<double> probs);
   vector<double> GetProbabilities();
   void UpdateBBToRange(size_t region);
   void RestoreBB();
   vector<Partition*> GetPartitions();
   vector<vector<VID>* > GetPartitionsVID();
   void EvaluatePartitions();
   void WriteRegionsSeparate();

 private:
   vector<string> m_EvaluatorLabels;
   string m_PartitioningMethod;
   string m_DistributionFeature;
   PartitionTree* m_pt;
   string m_TrainingStrategy;
   string m_OverlapMethod;
   vector<string> m_RegionStrategies;
   vector<vector<double> > m_min, m_max;
   vector<pair<double,double> > m_hold;
   int m_CurrentIteration;
};

#endif
