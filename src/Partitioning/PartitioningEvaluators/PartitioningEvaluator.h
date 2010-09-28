#ifndef PARTITIONEVALUATOR_H_
#define PARTITIONEVALUATOR_H_

#include "Partition.h"
#include "Features.h"

class PartitioningEvaluator {
 public:
   PartitioningEvaluator(){}
   PartitioningEvaluator(MPProblem* mp){m_pProblem=mp;}
   ~PartitioningEvaluator(){}

   virtual void ParseXML(XMLNodeReader& in_Node)=0;

   virtual vector<double> Evaluate(vector<Partition*> part)=0;

   string GetLabel(){return m_label;}
   void SetLabel(string s){m_label=s;}

   MPProblem* GetMPProblem(){return m_pProblem;}

   string GetFeature(){return m_Feature;}
   void SetFeature(string f){m_Feature = f;}

 protected:

   string m_Feature;
   string m_label;
   MPProblem* m_pProblem;
};

#endif
