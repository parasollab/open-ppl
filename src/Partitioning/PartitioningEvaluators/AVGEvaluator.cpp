#include "AVGEvaluator.h"
#include "MPStrategy.h"

AVGEvaluator::AVGEvaluator():PartitioningEvaluator(){}

AVGEvaluator::AVGEvaluator(XMLNodeReader& in_Node, MPProblem* mp):PartitioningEvaluator(mp){
   ParseXML(in_Node);
}

void AVGEvaluator::ParseXML(XMLNodeReader& in_Node){
   SetLabel(in_Node.stringXMLParameter(string("Label"), true, string(""), string("Partition Evaluator")));
   SetFeature(in_Node.stringXMLParameter(string("Feature"), true, string(""), string("Feature Name")));
   in_Node.warnUnrequestedAttributes();
}

vector<double> AVGEvaluator::Evaluate(vector<Partition*> part){
   vector<double> data;
   typedef vector<Partition*>::iterator PIT;
   for (PIT pit = part.begin(); pit!=part.end(); pit++){
      //Collect features from this partition
      vector<double> feature = m_pProblem->GetMPStrategy()->GetFeatures()->GetFeature(m_Feature)->Collect((*pit)->GetVID());
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
