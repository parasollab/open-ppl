#include "UAStrategy.h"
#include "PartitionTree.h"
#include "MapEvaluator.h"
#include "Partition.h"
#include "MPFeature.h"

UAStrategy::UAStrategy(XMLNode& in_Node, MPProblem* in_pProblem):MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0){
  ParseXML(in_Node);
}

void UAStrategy::ParseXML(XMLNode& in_Node){
   XMLNode::childiterator citr;
   for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
      if(citr->getName()=="training_strategy"){
         m_TrainingStrategy = citr->Read("Strategy", true, "", "Training Roadmap Creator");
      }
      else if(citr->getName()=="region_strategy"){
         string rs = citr->Read("Strategy", true, "", "Region Roadmap augmentor");
         m_RegionStrategies.push_back(rs);
      }
      else if(citr->getName()=="region_identifier"){
         m_PartitioningMethod = citr->Read("Method", true, "", "Region Identification Method");
      }
      else if(citr->getName()=="overlap_method"){
         m_OverlapMethod = citr->Read("Method", false, "default", "Forced Overlap Method");
      }
      else if(citr->getName()=="distribution_feature"){
         m_DistributionFeature = citr->Read("Feature", true, "", "Probability Distribution Feature");
      }
      else if(citr->getName()=="evaluation_method"){
         string eval = citr->Read("Method", true, "", "Evaluation Method");
         m_EvaluatorLabels.push_back(eval);
      }
      else
         citr->warnUnknownNode();
      citr->warnUnrequestedAttributes();
   }
}

void UAStrategy::Initialize(){
   cout<<"\nInitializing UAStrategy::"<<endl;



   cout<<"\nEnd Initializing UAStrategy"<<endl;
}

void UAStrategy::Run(){
   cout<<"\nRunning UAStrategy::"<<endl;

   MPProblem* mp = GetMPProblem();
   MPStrategy* ms = mp->GetMPStrategy();

   //create training roadmap
   ms->GetMPStrategyMethod(m_TrainingStrategy)->operator()();

   //identify regions
   IdentifyRegions();
   if(GetMPProblem()->GetMPStrategy()->GetPartitioningEvaluators()!=NULL)
      EvaluatePartitions();
   CollectMinMaxBBX();
   OverlapBBX();
   vector<double> probabilities = GetProbabilities();
   WriteRegionsSeparate();
   m_pt->WritePartitions(GetMPProblem(),
   GetMPProblem()->GetMPStrategy()->GetPartitioningMethods()->GetPartitioningMethod(m_PartitioningMethod)->GetClusteringDestination()+"region.",
   m_min, m_max);

   //initialize all the region strategies
   vector<vector<MPStrategyMethod*> > regionStrategyMethods;
   for(unsigned int i = 0; i<m_min.size(); i++){
      typedef vector<string>::iterator SIT;
      vector<MPStrategyMethod*> tmp;
      for(SIT sit = m_RegionStrategies.begin(); sit!=m_RegionStrategies.end(); sit++){
         tmp.push_back(ms->CreateMPStrategyMethod(*(ms->GetXMLNodeForStrategy(*sit))));
         tmp[tmp.size()-1]->Initialize();
      }
      regionStrategyMethods.push_back(tmp);
   }

   //Solve Regions probabilistically
   bool mapPassedEvaluation = false;
   while(!mapPassedEvaluation){
      m_CurrentIteration++;
      int region = GetRandRegion(probabilities);

      //update bounding box values
      UpdateBBToRange(region);

      //Do an iteration in a region
      typedef vector<MPStrategyMethod*>::iterator MIT;
      for(MIT mit = regionStrategyMethods[region].begin();mit!=regionStrategyMethods[region].end(); mit++){
         (*mit)->Run();
      }

      //restore bounding box values
      RestoreBB();

      mapPassedEvaluation = EvaluateMap(m_EvaluatorLabels);
   }

   //finalize the region solvers
   typedef vector<vector<MPStrategyMethod*> >::iterator VIT;
   typedef vector<MPStrategyMethod*>::iterator MIT;
   for(VIT vit=regionStrategyMethods.begin(); vit!=regionStrategyMethods.end(); vit++){
      for(MIT mit=vit->begin(); mit!=vit->end(); mit++){
         (*mit)->Finalize();
      }
   }

   cout<<"\nEnd Running UAStrategy"<<endl;
}

void UAStrategy::Finalize(){
   cout<<"\nFinalizing UAStrategy::"<<endl;

   //setup variables
   StatClass* stats = GetMPProblem()->GetStatClass();

   string str;

   //output final map
   str = GetBaseFilename() + ".map";
   ofstream osMap(str.c_str());
   GetMPProblem()->WriteRoadmapForVizmo(osMap);
   osMap.close();

   //output stats
   str = GetBaseFilename() + ".stat";
   ofstream  osStat(str.c_str());
   osStat << "NodeGen+Connection Stats" << endl;
   stats->PrintAllStats(osStat, GetMPProblem()->GetRoadmap());
   streambuf* sbuf = cout.rdbuf(); // to be restored later
   cout.rdbuf(osStat.rdbuf());   // redirect destination of std::cout
   //stats->PrintFeatures();
   cout.rdbuf(sbuf);  // restore original stream buffer
   osStat.close();



   cout<<"\nEnd Finalizing UAStrategy"<<endl;
}

void UAStrategy::Print(ostream& out_os) const {
   out_os<<"UAStrategy Options:"<<endl;
   out_os<<"\tTraining Strategy:"<<m_TrainingStrategy<<endl;
   GetMPProblem()->GetMPStrategy()->GetMPStrategyMethod(m_TrainingStrategy)->Print(out_os);
   out_os<<"\tRegion Identifier:"<<m_PartitioningMethod<<endl;
   out_os<<"\tDistribution Feature:"<<m_DistributionFeature<<endl;
   out_os<<"\tRegion Strategies:"<<endl;
   for(vector<string>::iterator sit = m_RegionStrategies.begin(); sit!=m_RegionStrategies.end(); sit++){
      out_os<<"\t\tStrategy:"<<*sit<<endl;
      GetMPProblem()->GetMPStrategy()->GetMPStrategyMethod(*sit)->Print(out_os);
   }
   out_os<<"\tEvaluators:"<<endl;
   for(vector<string>::iterator sit = m_EvaluatorLabels.begin(); sit!=m_EvaluatorLabels.end(); sit++){
      out_os<<"\t\tEvaluator:"<<*sit<<endl;
      GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetMethod(*sit)->Print(out_os);
   }
   out_os<<endl;
}

void UAStrategy::IdentifyRegions(){
   Partition* p=new Partition(GetMPProblem()->GetRoadmap(), 0);
   LeafPartitionNode lp(p);
   m_pt = new PartitionTree(lp);
   cout<<GetMPProblem()->GetMPStrategy()->GetPartitioningMethods()->GetPartitioningMethod(m_PartitioningMethod)->GetName()<<endl<<flush;
   m_pt->CreateTree(GetMPProblem()->GetMPStrategy()->GetPartitioningMethods()->GetPartitioningMethod(m_PartitioningMethod), dynamic_cast<LeafPartitionNode*>(m_pt->GetRoot()), new InternalPartitionNode());
}

void UAStrategy::CollectMinMaxBBX(){
   m_min.clear();
   m_max.clear();

   vector< vector< VID >* > Clusters = GetPartitionsVID();
   typedef vector< vector< VID >* >::iterator CIT;
   typedef vector< VID >::iterator NIT;

   Roadmap < CfgType, WeightType > * rm = GetMPProblem()->GetRoadmap();
   vector< double > CurrPos;
   double dtmp;
   CfgType NodeData;

   for( CIT itrCluster = Clusters.begin(); itrCluster != Clusters.end(); ++itrCluster){

      //
      // Collect the min/max x,y,z for this cluster
      //
      vector< double > Min(3),Max(3);
      Min[0] = 1.7e308;
      Min[1] = 1.7e308;
      Min[2] = 1.7e308;
      Max[0] = Max[1] = Max[2] = -(1.7e308);

      if( (*itrCluster)->size() < 1 ){
         Max[0] = Max[1] = Max[2] = Min[0] = Min[1] = Min[2] = 0;
      }
      else{
         //
         // At least 1 node in the cluster,
         // so grab it and use as the basis for the min/max values.
         //
         NIT itrNode = (*itrCluster)->begin();
         NodeData = rm->m_pRoadmap->find_vertex(*itrNode)->property();
         CurrPos = NodeData.GetPosition();

         dtmp = CurrPos[0];
         Min[0] = Max[0] = dtmp;
         dtmp = CurrPos[1];
         Min[1] = Max[1] = dtmp;
         dtmp = CurrPos[2];
         Min[2] = Max[2] = dtmp;

         itrNode++;

         //
         // Loop through remaining nodes to find min/max values.
         //
         while( itrNode != (*itrCluster)->end() ){
            NodeData = rm->m_pRoadmap->find_vertex(*itrNode)->property();
            CurrPos = NodeData.GetPosition();

            dtmp = CurrPos[0];
            if( dtmp < Min[0] )     { Min[0] = dtmp; }
            else if( dtmp > Max[0]) { Max[0] = dtmp; }

            dtmp = CurrPos[1];
            if( dtmp < Min[1] )     { Min[1] = dtmp; }
            else if( dtmp > Max[1]) { Max[1] = dtmp; }

            dtmp = CurrPos[2];
            if( dtmp < Min[2] )     { Min[2] = dtmp; }
            else if( dtmp > Max[2]) { Max[2] = dtmp; }

            itrNode++;
         }
      }

      m_min.push_back(Min);
      m_max.push_back(Max);
   }
}

bool sortRegionFunc(pair<VID, double> p1, pair<VID, double> p2){return p1.second<p2.second;}

void UAStrategy::OverlapBBX(){
   Environment *env = GetMPProblem()->GetEnvironment();
   boost::shared_ptr<Boundary> bb = GetMPProblem()->GetEnvironment()->GetBoundary();
   double robot_radius = 1.25*env->GetMultiBody(env->GetRobotIndex())->GetBoundingSphereRadius();

   if(m_OverlapMethod=="default"){
      for(int x = 0; x<3; x++){
         vector<int> indx;
         for(size_t i =0; i<m_min.size();i++)indx.push_back(i);
         bool swapped;
         do{
            swapped=false;
            for(size_t i = 0; i<m_min.size()-1; i++){
               if(m_min[indx[i]][x]>m_min[indx[i+1]][x]){
                  int temp = indx[i];
                  indx[i]=indx[i+1];
                  indx[i+1]=temp;
                  swapped=true;
               }
            }
         }while(swapped);
         for(size_t i=0;i<indx.size()-1;i++){
            if(m_max[indx[i]][x]<m_min[indx[i+1]][x]){
               double temp = m_max[indx[i]][x];
               double diff = m_min[indx[i+1]][x]-temp;
               if(m_min[indx[i+1]][x]-diff<bb->GetRange(x).first)
                  m_min[indx[i+1]][x]=bb->GetRange(x).first;
               else
                  m_min[indx[i+1]][x]=m_min[indx[i+1]][x]-diff;
               if(temp+diff>bb->GetRange(x).second)
                  m_max[indx[i]][x]=bb->GetRange(x).second;
               else
                  m_max[indx[i]][x]=temp+diff;
            }
         }
      }
   }
   else if(m_OverlapMethod=="MSTOverlap"||m_OverlapMethod=="MSTNewRegions"){
      //calculate mst of region graph//
      typedef stapl::sequential::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, int, double> Graph;
      Graph g;
      Roadmap < CfgType, WeightType > * rdmp = GetMPProblem()->GetRoadmap();
      vector< vector< VID >* > Clusters = GetPartitionsVID();
      typedef vector<vector<VID>* >::iterator VIT;
      typedef vector<VID>::iterator VVIT;
      int i = -1;
      vector<vector<double> > centers;
      for(VIT vit = Clusters.begin(); vit!=Clusters.end(); vit++){
         i++;
         int node=i;
         double x=0, y=0, z=0;
         for(VVIT vvit = (*vit)->begin(); vvit!=(*vit)->end(); vvit++){
            CfgType cfg = rdmp->m_pRoadmap->find_vertex(*vvit)->property();
            vector<double> pos = cfg.GetPosition();
            x+=pos[0];
            y+=pos[1];
            z+=pos[2];
         }
         vector<double> v;
         x/=(*vit)->size();
         y/=(*vit)->size();
         z/=(*vit)->size();
         v.push_back(x);
         v.push_back(y);
         v.push_back(z);
         centers.push_back(v);
         g.add_vertex(node);
      }
      for(size_t j = 0; j<Clusters.size(); j++){
         for(size_t k = j; k<Clusters.size(); k++){
            vector<double> centerA = centers[j];
            vector<double> centerB = centers[k];
            double weight = sqrt((centerB[0]-centerA[0])*(centerB[0]-centerA[0])+
                                 (centerB[1]-centerA[1])*(centerB[1]-centerA[1])+
                                 (centerB[2]-centerA[2])*(centerB[2]-centerA[2]));
            g.add_edge(j, k, weight);
         }
      }

      Graph mst_g;
      stapl::sequential::mst_kruskals(g, mst_g, 0);
      //perform swapping//
      if(m_OverlapMethod=="MSTOverlap"){
         for(size_t j = 0; j<mst_g.get_num_vertices(); j++){
            //get adjacent edgesi
            for(Graph::adj_edge_iterator e = mst_g.find_vertex(j)->begin(); e!=mst_g.find_vertex(j)->end(); e++){
               //use edge
               const size_t v1 = (*e).target();
               const size_t v2 = (*e).source();
               int swapRegion;
               if(v1!=j){
                  swapRegion = v1;
               }
               else{
                  swapRegion = v2;
               }
               vector<pair<VID, double> > regionDists;
               vector<double> centerA=centers[j];
               for(VVIT svit = Clusters[swapRegion]->begin(); svit!=Clusters[swapRegion]->end(); svit++){
                  CfgType cfg = rdmp->m_pRoadmap->find_vertex(*svit)->property();
                  vector<double> centerB = cfg.GetPosition();
                  double weight = sqrt((centerB[0]-centerA[0])*(centerB[0]-centerA[0])+
                        (centerB[1]-centerA[1])*(centerB[1]-centerA[1])+
                        (centerB[2]-centerA[2])*(centerB[2]-centerA[2]));
                  regionDists.push_back(pair<VID, double>(*svit, weight));
               }
               sort(regionDists.begin(), regionDists.end(), sortRegionFunc);
               int top10 = regionDists.size()/10;
               //int indx = rand()%top10;
               Clusters[i]->push_back(regionDists[top10].first);
            }
         }
      }
      else if(m_OverlapMethod=="MSTNewRegions"){
         vector<pair<int, int> > visitedEdges;
         typedef vector<pair<int, int> >::iterator PIT;
         for(Graph::edge_iterator e = mst_g.edges_begin(); e!=mst_g.edges_end(); e++){
            const int v1 = (*e).target();
            const int v2 = (*e).source();
            bool found = false;
            for(PIT pit = visitedEdges.begin(); pit!=visitedEdges.end(); pit++){
               if(pit->first==v1&&pit->second==v2){
                  found=true;
                  break;
               }
            }
            if(found){continue;}
            cout<<"FOUND!!!!!!!!!!!"<<endl;
            visitedEdges.push_back(pair<int,int>(v1,v2));
            visitedEdges.push_back(pair<int,int>(v2,v1));
            vector<pair<VID, double> > regionDistsA;
            vector<double> centerA=centers[v1];
            vector<pair<VID, double> > regionDistsB;
            vector<double> centerB=centers[v2];
            vector<double> centerN;
            centerN.push_back((centerA[0]+centerB[0])/2.0);
            centerN.push_back((centerA[1]+centerB[1])/2.0);
            centerN.push_back((centerA[2]+centerB[2])/2.0);
            for(VVIT svit = Clusters[v1]->begin(); svit!=Clusters[v1]->end(); svit++){
               CfgType cfg = rdmp->m_pRoadmap->find_vertex(*svit)->property();
               vector<double> centB = cfg.GetPosition();
               double weight = sqrt((centB[0]-centerN[0])*(centB[0]-centerN[0])+
                     (centB[1]-centerN[1])*(centB[1]-centerN[1])+
                     (centB[2]-centerN[2])*(centB[2]-centerN[2]));
               regionDistsA.push_back(pair<VID, double>(*svit, weight));
            }
            for(VVIT svit = Clusters[v2]->begin(); svit!=Clusters[v2]->end(); svit++){
               CfgType cfg = rdmp->m_pRoadmap->find_vertex(*svit)->property();
               vector<double> centB = cfg.GetPosition();
               double weight = sqrt((centB[0]-centerN[0])*(centB[0]-centerN[0])+
                     (centB[1]-centerN[1])*(centB[1]-centerN[1])+
                     (centB[2]-centerN[2])*(centB[2]-centerN[2]));
               regionDistsB.push_back(pair<VID, double>(*svit, weight));
            }
            sort(regionDistsA.begin(), regionDistsA.end(), sortRegionFunc);
            sort(regionDistsB.begin(), regionDistsB.end(), sortRegionFunc);
            int top10A = regionDistsA.size()/10;
            int top10B = regionDistsB.size()/10;
            vector<VID> vecVID;
            vecVID.push_back(regionDistsA[top10A].first);
            vecVID.push_back(regionDistsB[top10B].first);
            //m_pt->GetRoot()->AddChild(new LeafPartitionNode(m_pt->GetRoot(), new Partition(rdmp, vecVID)));
            m_pt->GetRoot()->AddChild(new LeafPartitionNode(new Partition(rdmp, vecVID)));
         }
      }
      CollectMinMaxBBX();
   }
   else{cout<<"NEED OVERLAP METHOD"<<endl; exit(1);}

//all methods need to extend region by a robot radius
   for(int x= 0; x<3; x++){
      for(size_t i = 0; i<m_min.size();i++){
         double maxbb = bb->GetRange(x).second;
         double minbb = bb->GetRange(x).first;
         if(m_min[i][x]-robot_radius>minbb){
            m_min[i][x]-=robot_radius;
         }
         else{
            m_min[i][x]=minbb;
         }
         if(m_max[i][x]+robot_radius<maxbb){
            m_max[i][x]+=robot_radius;
         }
         else{
            m_max[i][x]=maxbb;
         }
      }
   }
}

void UAStrategy::IntToStr(int myInt, string &myString){
	std::stringstream ss;
	ss << myInt;
	ss >> myString;
}

int UAStrategy::GetRandRegion(vector<double> probs){
   int i = 0;
   double sum = 0;
   double r = (double)(rand()%10000/10000.00);
	if(r==0){
		return 0;
	}
   cout<<"Random is "<<r<<endl<<flush;
   while(r>sum){
      sum+=probs[i];
      i++;
   }
   cout<<"Region is "<<i-1<<endl<<flush;
   return i-1;
}

vector<double> UAStrategy::GetProbabilities(){
   vector<vector<VID>* > Clusters = GetPartitionsVID();

   cout<<"Beginning to redistribute nodes"<<endl;

   vector<double> averages;

   typedef vector<vector<VID>* >::iterator CIT;
   typedef vector<double>::iterator DIT;
   for(CIT cit = Clusters.begin(); cit!=Clusters.end(); cit++){
      vector<double> features = GetMPProblem()->GetMPStrategy()->GetFeatures()->GetFeature(m_DistributionFeature)->Collect(**cit);
      double average = 0;
      for(DIT dit = features.begin(); dit!=features.end(); dit++){
         average+=(*dit);
      }
      average/=(double)features.size();
      averages.push_back(average);
   }

   double sum=0;
   vector<double> result;

   for(size_t RegionID = 0; RegionID<averages.size();RegionID++){

      cout<<"doing the redistribiution now"<<endl;
      sum=0;
      for(size_t i=0;i<averages.size(); i++){
         sum += (1-averages[i])*(1-averages[i]);
      }
      double VisibilityOfRegion = averages[RegionID];
      cout<<"V="<<VisibilityOfRegion<<"\t(1-v)^2="<<(1-averages[RegionID])*(1-averages[RegionID])<<"\tsum="<<sum<<endl<<flush;
      result.push_back((1-averages[RegionID])*(1-averages[RegionID])/sum);
   }

   cout<<"done with redistribute now display results"<<result.size()<<endl<<flush;
   for(size_t probs = 0; probs<result.size(); probs++){
      cout<<probs<<" = "<<result[probs]<<endl<<flush;
   }

   return result;
}

void UAStrategy::UpdateBBToRange(size_t region){
  m_hold.clear();
  if( region >=0 && region < m_min.size() ){
    boost::shared_ptr<Boundary> pMPEBoundBox = (GetMPProblem()->GetEnvironment())->GetBoundary();
    boost::shared_ptr<Boundary> pBoundBox = (GetMPProblem()->GetEnvironment())->GetBoundary();

    int size = (m_min.at(region)).size();
    for(int i=0; i < size;i++){
      pBoundBox->GetRange(i);
      pair< double, double> tmp( pBoundBox->GetRange(i));
      m_hold.push_back(tmp);
      if(i==0){
        pBoundBox->SetParameter(i, m_hold[i].first, m_hold[i].second);
        pMPEBoundBox->SetParameter(i, m_hold[i].first, m_hold[i].second);
      }
      else{
        pBoundBox->SetParameter(i, (m_min.at(region)).at(i), (m_max.at(region)).at(i));
        pMPEBoundBox->SetParameter(i, (m_min.at(region)).at(i), (m_max.at(region)).at(i));
      }
    }
    cout<<"START RESIZE BOUNDING BOX (NODE GENERATION/MAP PRINTING) TO:"<<endl<<flush;
    pBoundBox->Print(cout, ':', ';');
  }
}

void UAStrategy::RestoreBB(){
  boost::shared_ptr<Boundary> pMPEBoundBox = (GetMPProblem()->GetEnvironment())->GetBoundary();
  boost::shared_ptr<Boundary> pBoundBox = (GetMPProblem()->GetEnvironment())->GetBoundary();
  size_t i=0;
  for(i=0; i<m_hold.size();i++){
    pBoundBox->SetParameter(i, m_hold[i].first, m_hold[i].second);
    pMPEBoundBox->SetParameter(i, m_hold[i].first, m_hold[i].second);
  }
  if(i>0){
    cout<<"START RESTORE BOUNDING BOX (CONNECTION) TO:"<<endl<<flush;
    pBoundBox->Print(cout, ':', ';');
  }
}

vector<Partition*> UAStrategy::GetPartitions(){
   vector<PartitionNode*> nodes;
   vector<Partition*> parts;

   nodes=m_pt->GetRoot()->GetChildren();
   typedef vector<PartitionNode*>::iterator PIT;
   for (PIT pit=nodes.begin(); pit!=nodes.end(); pit++){
      parts.push_back((*pit)->GetPartition());
   }
   return parts;
}

vector<vector<VID>* > UAStrategy::GetPartitionsVID(){
   vector<vector<VID>* > Clusters;
   for(size_t itrCluster =0; itrCluster < m_pt->GetRoot()->GetChildren().size(); ++itrCluster){
      Clusters.push_back( m_pt->GetRoot()->GetChildren()[itrCluster]->GetVIDs() );
   }
   return Clusters;
}

void UAStrategy::EvaluatePartitions(){
   string filename=GetBaseFilename()+".partitions";
   vector<vector<double> > eval=GetMPProblem()->GetMPStrategy()->GetPartitioningEvaluators()->Evaluate(filename, GetPartitions());
}

void UAStrategy::WriteRegionsSeparate(){
   string baseFileName = GetBaseFilename();
   int count=0;
   RoadmapGraph<CfgType,WeightType>* rg = GetMPProblem()->GetRoadmap()->m_pRoadmap;
   vector<PartitionNode*> pnodes = m_pt->GetRoot()->GetChildren();
   typedef vector<PartitionNode*>::iterator PIT;
   for (PIT pit=pnodes.begin(); pit!=pnodes.end(); pit++, count++) {
      vector<VID> vi = (*pit)->GetPartition()->GetVID();
      BoundingBox bb(6,3);
      for(int i = 0; i <3; i++){
         bb.SetParameter(i, m_min[count][i], m_max[count][i]);
      }
      MPProblem regionProblem(GetMPProblem()->GetEnvironment(), GetMPProblem()->GetDistanceMetric(), GetMPProblem()->GetValidityChecker(), GetMPProblem()->GetNeighborhoodFinder());
      typedef vector<VID>::iterator VIT;
      for(VIT vit = vi.begin(); vit!=vi.end(); vit++){
         CfgType cfg = rg->find_vertex(*vit)->property();
         regionProblem.GetRoadmap()->m_pRoadmap->AddVertex(cfg);
      }
      for(unsigned int k = 0; k<vi.size()-1; k++){
         CfgType cfg1 = rg->find_vertex(vi[k])->property();
         CfgType cfg2 = rg->find_vertex(vi[k+1])->property();
         regionProblem.GetRoadmap()->m_pRoadmap->AddEdge(cfg1,cfg2, WeightType());
         //regionProblem.GetRoadmap()->m_pRoadmap->AddEdge(cfg2,cfg1,0);
      }
      CfgType cfg1 = rg->find_vertex(vi[0])->property();
      CfgType cfg2 = rg->find_vertex(vi[vi.size()-1])->property();
      //regionProblem.GetRoadmap()->m_pRoadmap->AddEdge(cfg1,cfg2,0);
      //regionProblem.GetRoadmap()->m_pRoadmap->AddEdge(cfg2,cfg1,0);
      ostringstream oss;
      oss<<baseFileName<<"."<<count<<".map";
      ofstream  myofstream(oss.str().c_str());
      if (!myofstream) {
         cerr << "print_feature_maps::WriteRoadmapForVizmo: can't open outfile: " << endl;
         exit(-1);
      }
      regionProblem.WriteRoadmapForVizmo(myofstream);
      myofstream.close();
   }
};


