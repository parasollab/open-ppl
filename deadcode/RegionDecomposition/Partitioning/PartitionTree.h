#ifndef _PARTITIONINGTREE_H
#define _PARTITIONINGTREE_H

#include "Roadmap.h"
#include "CfgTypes.h"
#include "PartitioningMethod.h"

extern double g_UAS_avgRegionStd;

typedef RoadmapGraph<CfgType, WeightType>::VID VID;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PartitionNode {
 public:

   PartitionNode();
   PartitionNode(PartitionNode *parent);
   PartitionNode(vector<PartitionNode*> children);
   PartitionNode(PartitionNode *parent, vector<PartitionNode*> children);
   PartitionNode(PartitionNode &p);
   virtual ~PartitionNode(){}

   void RemoveChild(PartitionNode *p);

   PartitionNode* GetParent(){return m_Parent;}
   void SetParent(PartitionNode *p);

   vector<PartitionNode*> GetChildren(){return m_Children;}
   void SetChildren(vector<PartitionNode*> vn);
   void AddChild(PartitionNode* child){m_Children.push_back(child);}

   virtual Partition* GetPartition(){return NULL;}
   virtual Roadmap<CfgType, WeightType>* GetRDMP(){return NULL;}
   virtual BoundingBox GetBoundingBox(){return BoundingBox(0,0);}
   virtual vector<VID>* GetVIDs(){return new vector<VID>();}
   Roadmap<CfgType, WeightType> GetPartitialRDMP();

 protected:
   PartitionNode *m_Parent;
   vector<PartitionNode*> m_Children;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class InternalPartitionNode : public PartitionNode {
 public:
   InternalPartitionNode();
   InternalPartitionNode(vector<PartitionNode*> children);
   InternalPartitionNode(PartitionNode* parent, vector<PartitionNode*> children);
   virtual ~InternalPartitionNode(){}

   //composes a new partition from getRDMP, getBB, and getVIDS
   //instead of storing the partition at this level of the tree
   virtual Partition* GetPartition();

   //calls one childs getRDMP method to get the RDMP
   virtual Roadmap<CfgType, WeightType>* GetRDMP();

   //calls all childrens getBB method to compose one big bounding volume
   virtual BoundingBox GetBoundingBox();

   //calls all childrens getVIDs method to compose one collection of
   //VID for this level of the partitio
   virtual vector<VID>* GetVIDs();
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class LeafPartitionNode : public PartitionNode {
 public:
   LeafPartitionNode();
   LeafPartitionNode(Partition* p);
   LeafPartitionNode(PartitionNode* parent, Partition* p);
   LeafPartitionNode(LeafPartitionNode &lpn);
   virtual ~LeafPartitionNode(){}

   virtual Partition* GetPartition();
   virtual Roadmap<CfgType, WeightType>* GetRDMP();
   virtual BoundingBox GetBoundingBox();
   virtual vector<VID>* GetVIDs();
 protected:
   Partition* m_Partition;//all leaf nodes contain their partition which stores all the basic information
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class PartitionTree{
 public:
   PartitionTree(){}
   PartitionTree(LeafPartitionNode root);
   ~PartitionTree(){}

   //writes partitions to a .map file for reading from vizmo
   void WritePartitions(MPProblem* pMPProblem, string base, vector<vector<double> >& min,
   vector<vector<double> >& max);

   //auto&  creates a tree into internal PartitionNode and leaf nodes for use
   void CreateTree(PartitioningMethod *pm, LeafPartitionNode* p, InternalPartitionNode* ipn);

   vector<PartitionNode*> GetNodes(){return m_pNodes;}
   void AddNode(PartitionNode* p){m_pNodes.push_back(p);}
   PartitionNode* GetRoot(){return m_Root;}
   void ReplaceNode(LeafPartitionNode* lpn, InternalPartitionNode* ipn);
   void RemoveNode(PartitionNode* pn);

 protected:
   PartitionNode* m_Root;
   vector<PartitionNode*> m_pNodes;
};

#endif
