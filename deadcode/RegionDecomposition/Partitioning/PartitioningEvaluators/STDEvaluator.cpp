#include "STDEvaluator.h"
#include "MPStrategy.h"
#include "Partition.h"
#include "Features.h"

STDEvaluator::STDEvaluator():PartitioningEvaluator(){}

STDEvaluator::STDEvaluator(XMLNode& in_Node, MPProblem* mp):PartitioningEvaluator(in_Node, mp){
   ParseXML(in_Node);
}

void STDEvaluator::ParseXML(XMLNode& in_Node){
   SetFeature(in_Node.Read("Feature", true, "", "Feature Name"));
   in_Node.warnUnrequestedAttributes();
}

vector<double> STDEvaluator::Evaluate(vector<Partition*> part){
   vector<double> data;
   typedef vector<Partition*>::iterator PIT;
   for (PIT pit = part.begin(); pit!=part.end(); pit++){
      //Collect features from this partition
      vector<double> feature = GetMPProblem()->GetMPStrategy()->GetFeatures()->GetFeature(m_Feature)->Collect((*pit)->GetVID());
      //find std on features from this partition
      double sum1=0,sum2=0;
      typedef vector<double>::iterator DIT;
      for(DIT dit=feature.begin(); dit!=feature.end(); dit++){
         sum1+=*dit;
         sum2+=(*dit)*(*dit);
      }
      double mean = sum1/feature.size();
      double std = sqrt((sum2-sum1*mean)/(feature.size()-1.0));
      data.push_back(std);
   }
   return data;
};
