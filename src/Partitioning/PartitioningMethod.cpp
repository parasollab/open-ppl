#include "PartitioningMethod.h"

PartitioningMethod::PartitioningMethod():MPBaseObject(), m_Partitions(){
};

PartitioningMethod::PartitioningMethod(XMLNodeReader& in_Node, MPProblem * m): MPBaseObject(in_Node, m), m_ClusteringDestination(""), m_Partitions(){}

PartitioningMethod::~PartitioningMethod(){
};

void PartitioningMethod::ParseXML(XMLNodeReader& in_Node){
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
    if(citr->getName()=="Feature"){
      string name = citr->stringXMLParameter("Name", true, "", "FeatureName");
      double weight = citr->numberXMLParameter("Weight", true, 1.0, 0.0, 1.0, "FeatureWeight");
      m_Features.push_back(pair<string, double>(name, weight));
    }
  }
  SetClusteringDestination(in_Node.stringXMLParameter("destination", true, "", "PartitioningMethod"));
  in_Node.warnUnrequestedAttributes();
}
