#ifndef UASTRATEGY_H_
#define UASTRATEGY_H_

#include "MPStrategy.h"
#include "PartitionTree.h"
class PartitionTree;

class UAStrategy : public MPStrategyMethod {
 public:
   UAStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem);
   virtual ~UAStrategy(){}
   
   virtual void ParseXML(XMLNodeReader& in_Node);
   
   virtual void Initialize(int in_RegionID);
   virtual void Run(int in_RegionID);
   virtual void Finalize(int in_RegionID);

   virtual void PrintOptions(ostream& out_os);

 protected:
   void IdentifyRegions();
   void CollectMinMaxBBX();
   void OverlapBBX();
   void IntToStr(int myInt, string &myString);
   int GetRandRegion(vector<double> probs);
   vector<double> GetProbabilities();
   void UpdateBBToRange(int region);
   void RestoreBB();
   vector<Partition*> GetPartitions();
   vector<vector<VID>* > GetPartitionsVID();
   void EvaluatePartitions();
   bool EvaluateMap(int in_RegionID);
   
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
