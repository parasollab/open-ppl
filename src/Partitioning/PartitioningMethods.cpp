#include "PartitioningMethods.h"

#include "PartitioningMethod.h"
#include "GrowablePartitions.h"
#include "KMeans.h"
#include "HierarchicalClustering.h"
#include "PGMeansClustering.h"
#include "SuccessiveClustering.h"

PartitioningMethods::PartitioningMethods() : MPBaseObject(){
   all.push_back(new KMeans());
   all.push_back(new GrowablePartitions());
   all.push_back(new HierarchicalClustering());
   all.push_back(new PGMeansClustering());
   all.push_back(new SuccessiveClustering());
};

PartitioningMethods::PartitioningMethods(XMLNode& in_Node, MPProblem* in_pProblem) : MPBaseObject(in_Node, in_pProblem){
  XMLNode::childiterator citr;
  for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
    if(citr->getName()=="Kmeans"){
      KMeans* km = new KMeans(*citr, in_pProblem);
      selected.push_back(km);
    }
    else if(citr->getName()=="Growable"){
      GrowablePartitions* gp = new GrowablePartitions(*citr, in_pProblem);
      selected.push_back(gp);
    }
    else if(citr->getName()=="Hierarchical"){
      HierarchicalClustering* hc = new HierarchicalClustering(*citr, in_pProblem);
      selected.push_back(hc);
    }
    else if(citr->getName()=="PGmeans"){
      PGMeansClustering* pg = new PGMeansClustering(*citr, in_pProblem);
      selected.push_back(pg);
    }
    else if(citr->getName()=="Successive"){
      SuccessiveClustering* sc = new SuccessiveClustering(*citr, in_pProblem);
      selected.push_back(sc);
    }
    citr->warnUnrequestedAttributes();
  }
};

PartitioningMethod* PartitioningMethods::GetPartitioningMethod(string s){
   typedef vector<PartitioningMethod*>::iterator PIT;
   for(PIT pit=selected.begin(); pit!=selected.end(); pit++){
      if((*pit)->GetLabel()==s){
         return (*pit);
      }
   }
   return NULL;
};
