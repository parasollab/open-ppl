#ifndef _PARTITIONMETHOD_H
#define _PARTITIONMETHOD_H

#include "Roadmap.h"
#include "CfgTypes.h"

class Partition;

class PartitioningMethod : public MPBaseObject
{
 public:
  typedef RoadmapGraph<CfgType, WeightType>::VID VID;

  PartitioningMethod();
  PartitioningMethod(XMLNode& in_Node, MPProblem * mp);
  virtual ~PartitioningMethod();

  virtual void ParseXML(XMLNode& in_Node);

  virtual vector<Partition*> MakePartitions(Partition &p)=0;

  string GetClusteringDestination(){return m_ClusteringDestination;}
  void SetClusteringDestination(string s){m_ClusteringDestination=s;}

  vector<Partition*> GetPartitions(){return m_Partitions;}

  vector<pair<string, double> > GetFeatures(){return m_Features;}
  void SetFeatures(vector<pair<string, double> > f){m_Features = f;}

 protected:
  string m_ClusteringDestination;
  vector<Partition*> m_Partitions;
  vector<pair<string, double> > m_Features;
};

#endif
