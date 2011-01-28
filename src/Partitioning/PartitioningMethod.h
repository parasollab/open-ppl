#ifndef _PARTITIONMETHOD_H
#define _PARTITIONMETHOD_H

#include "Roadmap.h"
#include "CfgTypes.h"
#include "Weight.h"
#include "Partition.h"

#include "Features.h"

//#include "IncrementalSamplingMetrics.h"
template<typename CFG, typename WEIGHT>
  class IncrementalSamplingMetrics;
class Partition;

class PartitioningMethod
{
 public:
  PartitioningMethod();
  PartitioningMethod(string s, MPProblem * mp);
  virtual ~PartitioningMethod();

  virtual void ParseXML(XMLNodeReader& in_Node)=0;

  virtual vector<Partition*> MakePartitions(Partition &p)=0;

  string GetName(){return m_Name;}
  void SetName(string s){m_Name=s;}

  string GetLabel(){return m_Label;}
  void SetLabel(string s){m_Label=s;}

  string GetClusteringDestination(){return m_ClusteringDestination;}
  void SetClusteringDestination(string s){m_ClusteringDestination=s;}

  vector<Partition*> GetPartitions(){return m_Partitions;}

  vector<pair<string, double> > GetFeatures(){return m_Features;}
  void SetFeatures(vector<pair<string, double> > f){m_Features = f;}
  
 protected:
  MPProblem* m_pProblem;
  string m_Name;
  string m_Label;
  vector<Partition*> m_Partitions;
  string m_ClusteringDestination;
  vector<pair<string, double> > m_Features;
};

#endif
