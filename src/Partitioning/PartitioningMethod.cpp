#include "PartitioningMethod.h"

PartitioningMethod::PartitioningMethod():m_Name("PM"),m_Partitions(){
};

PartitioningMethod::PartitioningMethod(string name, MPProblem * m):m_Name(name), m_pProblem(m),m_ClusteringDestination(""), m_Partitions(){}

PartitioningMethod::~PartitioningMethod(){
};

