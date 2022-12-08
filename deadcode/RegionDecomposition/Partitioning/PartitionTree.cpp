#include "PartitionTree.h"
#include "Partition.h"
#include "MPProblem.h"


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
   vector<VID>* vvid = GetVIDs();
   Roadmap<CfgType, WeightType> rdmp;
   typedef vector<VID>::iterator VIT;
   for(VIT vit = vvid->begin(); vit!=vvid->end(); vit++){
      rdmp.m_pRoadmap->AddVertex(GetRDMP()->m_pRoadmap->find_vertex(*vit)->property());
   }
   return rdmp;
};

InternalPartitionNode::InternalPartitionNode():PartitionNode(){}

InternalPartitionNode::InternalPartitionNode(vector<PartitionNode*> children):PartitionNode(children){}

InternalPartitionNode::InternalPartitionNode(PartitionNode* parent, vector<PartitionNode*> children):PartitionNode(parent, children){}

Partition* InternalPartitionNode::GetPartition(){
   Partition temp(GetRDMP(), 1);
   temp.SetVID(*GetVIDs());
   return new Partition(temp);
};

Roadmap<CfgType, WeightType>* InternalPartitionNode::GetRDMP(){
   return m_Children[0]->GetRDMP();
};

BoundingBox InternalPartitionNode::GetBoundingBox(){
   return GetPartition()->GetBoundingBox();
};

vector<VID>* InternalPartitionNode::GetVIDs(){
   vector<VID>* temp = new vector<VID>();
   typedef vector<PartitionNode*>::iterator PIT;
   typedef vector<VID>::iterator VIT;
   for(PIT pit = m_Children.begin(); pit!=m_Children.end(); pit++){
      vector<VID>* vids = (*pit)->GetVIDs();
      for(VIT vit = vids->begin(); vit!=vids->end(); vit++){
         temp->push_back(*vit);
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

vector<VID>* LeafPartitionNode::GetVIDs(){return &(m_Partition->GetVID());};

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
   vector<Partition*> vp= pm->MakePartitions(*(p->GetPartition()));
   typedef vector<Partition*>::iterator PIT;
   for(PIT pit = vp.begin(); pit!=vp.end(); pit++){
      LeafPartitionNode* lpn = new LeafPartitionNode(ipn, *pit);
      AddNode(lpn);
   }
   ReplaceNode(p,ipn);
};

inline void IntToStr(int myInt, string &myString) {
	std::stringstream ss;
	ss << myInt;
	ss >> myString;
};

void PartitionTree::WritePartitions(MPProblem* pMPProblem, string base, vector<vector<double> >&
min, vector<vector<double> >& max){
   string baseFileName = base;
   MPProblem eachRgn(pMPProblem->GetEnvironment(), pMPProblem->GetDistanceMetric(), pMPProblem->GetValidityChecker(), pMPProblem->GetNeighborhoodFinder());
   RoadmapGraph<CfgType,WeightType>* rg = pMPProblem->GetRoadmap()->m_pRoadmap;
   vector<PartitionNode*> pnodes = m_Root->GetChildren();
   vector<VID> allVIDS;
   typedef vector<PartitionNode*>::iterator PIT;
   for (PIT pit=pnodes.begin(); pit!=pnodes.end(); pit++) {
      vector<VID> vi = (*pit)->GetPartition()->GetVID();
      vector<VID> newVIDS;
      typedef vector<VID>::iterator VIT;
      for(VIT vit = vi.begin(); vit!=vi.end(); vit++){
         bool alter = false;
         if(find(allVIDS.begin(), allVIDS.end(), *vit)!=allVIDS.end())
            alter=true;
         else
            allVIDS.push_back(*vit);
         CfgType cfg = rg->find_vertex(*vit)->property();
         if(alter){
            cfg.IncSingleParam(0,0.01);
         }
         newVIDS.push_back(eachRgn.GetRoadmap()->m_pRoadmap->AddVertex(cfg));
      }
      for(unsigned int k = 0; k<newVIDS.size(); k++){
         eachRgn.GetRoadmap()->m_pRoadmap->AddEdge(newVIDS[k],newVIDS[k+1], WeightType());
      }
      eachRgn.GetRoadmap()->m_pRoadmap->AddEdge(newVIDS[newVIDS.size()-1],newVIDS[0], WeightType());
   }
   vector<shared_ptr<Boundary> >* bboxes = new vector<shared_ptr<Boundary> >();
   for(unsigned int i = 0; i<min.size(); i++){
      shared_ptr<Boundary> bb;
      bb->SetParameter(0, min[i][0], max[i][0]);
      bb->SetParameter(1, min[i][1], max[i][1]);
      bb->SetParameter(2, min[i][2], max[i][2]);
      bb->SetParameter(3, 0, 1);
      bb->SetParameter(4, 0, 1);
      bb->SetParameter(5, 0, 1);
      bboxes->push_back(bb);
   }
   string outputFilename = baseFileName+".map";
   ofstream  myofstream(outputFilename.c_str());
   if (!myofstream) {
      cerr << "print_feature_maps::WriteRoadmapForVizmo: can't open outfile: " << endl;
      exit(-1);
   }
   eachRgn.WriteRoadmapForVizmo(myofstream, bboxes);
   myofstream.close();
   delete bboxes;
};
