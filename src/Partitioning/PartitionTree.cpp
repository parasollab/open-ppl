#include "PartitionTree.h"

PartitionNode::PartitionNode(){
   m_Parent=NULL;
   m_Children.clear();
}

PartitionNode::PartitionNode(PartitionNode *parent){
   SetParent(parent);
   m_Children.clear();
}

PartitionNode::PartitionNode(vector<PartitionNode*> children){
   m_Parent = NULL;
   SetChildren(children);
}

PartitionNode::PartitionNode(PartitionNode *parent, vector<PartitionNode*> children){
   SetParent(parent);
   SetChildren(children);
}

PartitionNode::PartitionNode(PartitionNode &p){
   m_Parent=p.GetParent();
   m_Children=p.GetChildren();
}

void PartitionNode::RemoveChild(PartitionNode *p){
   if(m_Children.size()!=0){
      for (vector<PartitionNode*>::iterator it = m_Children.begin(); it!=m_Children.end(); ++it){
         if((*it)==p){
            m_Children.erase(it);
            return;
         }
      } 
   }
}

void PartitionNode::SetParent(PartitionNode *p){
   m_Parent = p;
   if(m_Parent!=NULL)
      m_Parent->AddChild(this);
}

void PartitionNode::SetChildren(vector<PartitionNode*> vn){
   m_Children=vn;
   typedef vector<PartitionNode*>::iterator PIT;
   for(PIT pit = m_Children.begin(); pit!=m_Children.end(); pit++){
      (*pit)->SetParent(this);
   }
}

Roadmap<CfgType, WeightType> PartitionNode::GetPartitialRDMP(){
   vector<VID> vvid = GetVIDs();
   Roadmap<CfgType, WeightType> rdmp;
   typedef vector<VID>::iterator VIT;
   for(VIT vit = vvid.begin(); vit!=vvid.end(); vit++){
      rdmp.m_pRoadmap->AddVertex(GetRDMP()->m_pRoadmap->find_vertex(*vit)->property());
   }
   return rdmp;
};

InternalPartitionNode::InternalPartitionNode():PartitionNode(){}

InternalPartitionNode::InternalPartitionNode(vector<PartitionNode*> children):PartitionNode(children){}

InternalPartitionNode::InternalPartitionNode(PartitionNode* parent, vector<PartitionNode*> children):PartitionNode(parent, children){}

Partition* InternalPartitionNode::GetPartition(){
   Partition temp(GetRDMP(), 1);
   temp.SetVID(GetVIDs());
   return new Partition(temp);
};

Roadmap<CfgType, WeightType>* InternalPartitionNode::GetRDMP(){
   return m_Children[0]->GetRDMP();
};

BoundingBox InternalPartitionNode::GetBoundingBox(){
   return GetPartition()->GetBoundingBox();
};

vector<VID> InternalPartitionNode::GetVIDs(){
   vector<VID> temp;
   typedef vector<PartitionNode*>::iterator PIT;
   typedef vector<VID>::iterator VIT;
   for(PIT pit = m_Children.begin(); pit!=m_Children.end(); pit++){
      vector<VID> vids = (*pit)->GetVIDs();
      for(VIT vit = vids.begin(); vit!=vids.end(); vit++){
         temp.push_back(*vit);
      }
   }
   return temp;
};

LeafPartitionNode::LeafPartitionNode(Partition *p):PartitionNode(),m_Partition(p){}

LeafPartitionNode::LeafPartitionNode(PartitionNode* parent, Partition *p):PartitionNode(parent), m_Partition(p){}

LeafPartitionNode::LeafPartitionNode(LeafPartitionNode &lpn){
   m_Partition=lpn.GetPartition();
   SetParent(lpn.GetParent());
}

Partition* LeafPartitionNode::GetPartition(){return m_Partition;};

BoundingBox LeafPartitionNode::GetBoundingBox(){return m_Partition->GetBoundingBox();};

Roadmap<CfgType, WeightType>* LeafPartitionNode::GetRDMP(){return m_Partition->GetRoadmap();};

vector<VID> LeafPartitionNode::GetVIDs(){return m_Partition->GetVID();};

PartitionTree::PartitionTree(LeafPartitionNode root){
   m_Root= new LeafPartitionNode(root);
   m_pNodes.push_back(m_Root);
}

void PartitionTree::ReplaceNode(LeafPartitionNode* lpn, InternalPartitionNode* ipn){
   PartitionNode* parent = lpn->GetParent();
   RemoveNode(lpn);
   if(parent!=NULL){
      ipn->SetParent(parent);
   }
   AddNode(ipn);
   if(m_Root==lpn){m_Root=ipn;}
};

void PartitionTree::RemoveNode(PartitionNode* pn){
   if(pn->GetParent()!=NULL){
      pn->GetParent()->RemoveChild(pn);
   }
   pn->SetParent(NULL);
   for (unsigned int i =0;i<m_pNodes.size();i++){
      if((*(pn->GetPartition())) == (*(m_pNodes[i]->GetPartition()))){
         vector<PartitionNode*>::iterator it = m_pNodes.begin();
         m_pNodes.erase(it+i);
         return;
      }
   }
};

void PartitionTree::CreateTree(PartitioningMethod *pm, LeafPartitionNode* p, InternalPartitionNode* ipn){
   LOG_DEBUG_MSG("START PARTITIONTREE::CREATETREE()");
   vector<Partition*> vp= pm->MakePartitions(*(p->GetPartition()));
   typedef vector<Partition*>::iterator PIT;
   for(PIT pit = vp.begin(); pit!=vp.end(); pit++){
      LeafPartitionNode* lpn = new LeafPartitionNode(ipn, *pit);
      AddNode(lpn);
   }
   ReplaceNode(p,ipn);   
   LOG_DEBUG_MSG("END PARTITIONTREE::CREATETREE()");
};

inline void IntToStr(int myInt, string &myString) {
	std::stringstream ss;          
	ss << myInt;
	ss >> myString;
};

void PartitionTree::WritePartitions(MPProblem* pMPProblem, string base){
   string baseFileName = base;
   MPRegion<CfgType,WeightType> eachRgn(10, pMPProblem);
   RoadmapGraph<CfgType,WeightType>* rg = pMPProblem->GetMPRegion(0)->GetRoadmap()->m_pRoadmap;
   vector<PartitionNode*> pnodes = m_Root->GetChildren();
   typedef vector<PartitionNode*>::iterator PIT;
   for (PIT pit=pnodes.begin(); pit!=pnodes.end(); pit++) {
      vector<VID> vi = (*pit)->GetPartition()->GetVID();
      typedef vector<VID>::iterator VIT;
      for(VIT vit = vi.begin(); vit!=vi.end(); vit++){
         eachRgn.GetRoadmap()->m_pRoadmap->AddVertex(rg->find_vertex(*vit)->property());
      }
      for(unsigned int k = 0; k<vi.size()-1; k++){
         eachRgn.GetRoadmap()->m_pRoadmap->AddEdge(rg->find_vertex(vi[k])->property(),rg->find_vertex(vi[k+1])->property(),0);
      }
   }
   string outputFilename = baseFileName+".map";
   ofstream  myofstream(outputFilename.c_str());    
   if (!myofstream) {
      LOG_ERROR_MSG("print_feature_maps::WriteRoadmapForVizmo: can't open outfile: ");
      exit(-1);
   } 
   eachRgn.WriteRoadmapForVizmo(myofstream);
   myofstream.close();
};
