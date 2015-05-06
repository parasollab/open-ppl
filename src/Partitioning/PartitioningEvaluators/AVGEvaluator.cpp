#include "AVGEvaluator.h"
#include "MPStrategy.h"
#include "Partition.h"
#include "Features.h"

AVGEvaluator::AVGEvaluator():PartitioningEvaluator(){}

AVGEvaluator::AVGEvaluator(XMLNode& in_Node, MPProblem* mp):PartitioningEvaluator(in_Node, mp){
   ParseXML(in_Node);
}

void AVGEvaluator::ParseXML(XMLNode& in_Node){
   SetFeature(in_Node.Read("Feature", true, "", "Feature Name"));
   in_Node.warnUnrequestedAttributes();
}

vector<double> AVGEvaluator::Evaluate(vector<Partition*> part){
   vector<double> data;
   typedef vector<Partition*>::iterator PIT;
   for (PIT pit = part.begin(); pit!=part.end(); pit++){
      //Collect features from this partition
      vector<double> feature = GetMPProblem()->GetMPStrategy()->GetFeatures()->GetFeature(m_Feature)->Collect((*pit)->GetVID());
      //find average on features from this partition
      double sum=0;
      typedef vector<double>::iterator DIT;
      for(DIT dit=feature.begin(); dit!=feature.end(); dit++){
         sum+=(*dit);
      }
      data.push_back(sum/feature.size());
   }
   return data;
};
