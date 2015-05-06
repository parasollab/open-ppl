#include "PartitioningEvaluators.h"
#include "PartitioningEvaluator.h"
#include "STDEvaluator.h"
#include "AVGEvaluator.h"

PartitioningEvaluators::PartitioningEvaluators(): MPBaseObject(){
   all.push_back(new STDEvaluator());
   all.push_back(new AVGEvaluator());
}

PartitioningEvaluators::PartitioningEvaluators(XMLNode& in_Node, MPProblem* mp) : MPBaseObject(in_Node, mp){
   //read from the xml and push onto selected
   XMLNode::childiterator citr;
   for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
      if(citr->getName()=="STDEvaluator"){
         STDEvaluator* se = new STDEvaluator(*citr, mp);
         selected.push_back(se);      }
      if(citr->getName()=="AVGEvaluator"){
         AVGEvaluator* ae = new AVGEvaluator(*citr, mp);
         selected.push_back(ae);
      }
      citr->warnUnrequestedAttributes();
   }
}

PartitioningEvaluator* PartitioningEvaluators::GetEvaluator(string s){
   typedef vector<PartitioningEvaluator*>::iterator PIT;
   for(PIT pit = selected.begin(); pit!=selected.end(); pit++){
      if((*pit)->GetLabel()==s){
         return *pit;
      }
   }
   return NULL;
}

vector<vector<double> > PartitioningEvaluators::Evaluate(string filename, vector<Partition*> vp){
   typedef vector<PartitioningEvaluator*>::iterator PIT;
   vector<vector<double> > data;
   for(PIT pit=selected.begin(); pit!=selected.end(); pit++){
      data.push_back((*pit)->Evaluate(vp));
   }
   ofstream ofs(filename.c_str());
   cout << endl;
   for (unsigned int i=0; i<data[0].size(); i++){
      for (unsigned int j=0; j<data.size(); j++){
         ofs << "PARTITION: " << i << " FEATURE: " << selected[j]->GetFeature() << " EVALUATOR: " << selected[j]->GetLabel() << " VALUE: " <<  data[j][i]<< endl;
         cout << "PARTITION: " << i << " FEATURE: " << selected[j]->GetFeature() << " EVALUATOR: " << selected[j]->GetLabel() << " VALUE: " <<  data[j][i] << endl;
      }
   }
   ofs.close();
   return data;
};


