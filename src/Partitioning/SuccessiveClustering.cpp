#include "SuccessiveClustering.h"
#include "MPStrategy.h"
#include "PartitioningMethods.h"
#include "Partition.h"

SuccessiveClustering::SuccessiveClustering():PartitioningMethod(){
};

SuccessiveClustering::SuccessiveClustering(XMLNode& in_Node, MPProblem* in_pProblem):PartitioningMethod(in_Node, in_pProblem){
  this->SetName("successive");
   ParseXML(in_Node);
};

SuccessiveClustering::~SuccessiveClustering(){};

void SuccessiveClustering::ParseXML(XMLNode& in_Node){
   XMLNode::childiterator citr;
   for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
      if(citr->getName()=="PartitioningMethod"){
         string name = citr->Read(string("Method"), true, string(""), string("Partitioning Method"));
         m_PartitioningMethods.push_back(name);
      }
   }
   //SetLabel(in_Node.Read(string("Label"), true, string(""), string("PartitioningMethod Label")));
   SetClusteringDestination(in_Node.Read(string("destination"), true, string(""), string("PartitioningMethod Destination")));
   in_Node.warnUnrequestedAttributes();
}

vector<Partition*> SuccessiveClustering::MakePartitions(Partition &p){
   vector<Partition*> parts;
   parts.push_back(new Partition(p));
   typedef vector<Partition*>::iterator PIT;
   typedef vector<string>::iterator SIT;
   for(SIT sit = m_PartitioningMethods.begin(); sit!=m_PartitioningMethods.end(); sit++){
      cout<<"Starting Method::"<<*sit<<endl;
      vector<Partition*> tmp;
      for(PIT pit = parts.begin(); pit!=parts.end(); pit++){
         vector<Partition*> vp = GetMPProblem()->GetMPStrategy()->GetPartitioningMethods()->GetPartitioningMethod(*sit)->MakePartitions(**pit);
         cout<<"Made "<<vp.size()<<"regions"<<endl;
         for(PIT pit2 = vp.begin(); pit2!=vp.end(); pit2++){
            tmp.push_back(*pit2);
         }
      }
      parts.clear();
      parts=tmp;
   }



   vector<Partition*> vp;
   for(PIT pit = parts.begin(); pit!=parts.end(); pit++){
      vp.push_back(new Partition((*pit)->GetRoadmap(), (*pit)->GetID()+vp.size()-1));
      vp[vp.size()-1]->SetVID((*pit)->GetVID());
      vp[vp.size()-1]->GetBoundingBox().Print(cout);
   }
   return vp;
};


