#include "UAStrategy.h"

UAStrategy::UAStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem):MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0){
   LOG_DEBUG_MSG("UAStrategy::UAStrategy()");
   
   ParseXML(in_Node);
   
   LOG_DEBUG_MSG("~UAStrategy::UAStrategy()");
}
   
void UAStrategy::ParseXML(XMLNodeReader& in_Node){
   XMLNodeReader::childiterator citr;
   for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
      if(citr->getName()=="training_strategy"){
         m_TrainingStrategy = citr->stringXMLParameter("Strategy", true, "", "Training Roadmap Creator");
      }
      else if(citr->getName()=="region_strategy"){
         string rs = citr->stringXMLParameter("Strategy", true, "", "Region Roadmap augmentor");
         m_RegionStrategies.push_back(rs);
      }
      else if(citr->getName()=="region_identifier"){
         m_PartitioningMethod = citr->stringXMLParameter("Method", true, "", "Region Identification Method");
      }
      else if(citr->getName()=="distribution_feature"){
         m_DistributionFeature = citr->stringXMLParameter("Feature", true, "", "Probability Distribution Feature");
      }
      else if(citr->getName()=="evaluation_method"){
         string eval = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
         m_EvaluatorLabels.push_back(eval);
      }
      else
         citr->warnUnknownNode();
      citr->warnUnrequestedAttributes();
   }
}

void UAStrategy::Initialize(int in_RegionID){
   cout<<"\nInitializing UAStrategy::"<<in_RegionID<<endl;



   cout<<"\nEnd Initializing UAStrategy"<<endl;
}

void UAStrategy::Run(int in_RegionID){
   cout<<"\nRunning UAStrategy::"<<in_RegionID<<endl;

   MPProblem* mp = GetMPProblem();
   MPStrategy* ms = mp->GetMPStrategy();

   //create training roadmap
   ms->GetMPStrategyMethod(m_TrainingStrategy)->operator()(in_RegionID);
   
   //identify regions
   IdentifyRegions();
   EvaluatePartitions();
   CollectMinMaxBBX();
   vector<double> probabilities = GetProbabilities(); 

   //initialize all the region strategies
   vector<vector<MPStrategyMethod*> > regionStrategyMethods;
   for(unsigned int i = 0; i<m_min.size(); i++){
      typedef vector<string>::iterator SIT;
      vector<MPStrategyMethod*> tmp;
      for(SIT sit = m_RegionStrategies.begin(); sit!=m_RegionStrategies.end(); sit++){
         tmp.push_back(ms->CreateMPStrategyMethod(*(ms->GetXMLNodeForStrategy(*sit))));
         tmp[tmp.size()-1]->Initialize(0);
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
         (*mit)->Run(0);
      }

      //restore bounding box values
      RestoreBB();

      mapPassedEvaluation = EvaluateMap(in_RegionID); 
   } 

   //finalize the region solvers
   typedef vector<vector<MPStrategyMethod*> >::iterator VIT;
   typedef vector<MPStrategyMethod*>::iterator MIT;
   for(VIT vit=regionStrategyMethods.begin(); vit!=regionStrategyMethods.end(); vit++){
      for(MIT mit=vit->begin(); mit!=vit->end(); mit++){
         (*mit)->Finalize(0);
      }
   }

   cout<<"\nEnd Running UAStrategy"<<endl;
}

void UAStrategy::Finalize(int in_RegionID){
   cout<<"\nFinalizing UAStrategy::"<<in_RegionID<<endl;
 
   //setup region variables
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* regionStats = region->GetStatClass();
 
   string str;
  
   //output final map
   str = getBaseFilename() + ".map";
   ofstream osMap(str.c_str());
   region->WriteRoadmapForVizmo(osMap);
   osMap.close();
  
   //output stats
   str = getBaseFilename() + ".stat";
   ofstream  osStat(str.c_str());
   streambuf* sbuf = cout.rdbuf(); // to be restored later
   cout.rdbuf(osStat.rdbuf());   // redirect destination of std::cout
   cout << "NodeGen+Connection Stats" << endl;
   regionStats->PrintAllStats(region->GetRoadmap());
   //regionStats->PrintFeatures();
   cout.rdbuf(sbuf);  // restore original stream buffer
   osStat.close();



   cout<<"\nEnd Finalizing UAStrategy"<<endl;
}

void UAStrategy::PrintOptions(ostream& out_os){
   out_os<<"UAStrategy Options:"<<endl;
   out_os<<"\tTraining Strategy:"<<m_TrainingStrategy<<endl;
   GetMPProblem()->GetMPStrategy()->GetMPStrategyMethod(m_TrainingStrategy)->PrintOptions(out_os);
   out_os<<"\tRegion Identifier:"<<m_PartitioningMethod<<endl;
   out_os<<"\tDistribution Feature:"<<m_DistributionFeature<<endl;
   out_os<<"\tRegion Strategies:"<<endl;
   for(vector<string>::iterator sit = m_RegionStrategies.begin(); sit!=m_RegionStrategies.end(); sit++){
      out_os<<"\t\tStrategy:"<<*sit<<endl;
      GetMPProblem()->GetMPStrategy()->GetMPStrategyMethod(*sit)->PrintOptions(out_os);
   }
   out_os<<"\tEvaluators:"<<endl;
   for(vector<string>::iterator sit = m_EvaluatorLabels.begin(); sit!=m_EvaluatorLabels.end(); sit++){
      out_os<<"\t\tEvaluator:"<<*sit<<endl;
      GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*sit)->PrintOptions(out_os);
   }
   out_os<<endl;
}

void UAStrategy::IdentifyRegions(){   
   LOG_DEBUG_MSG("CREATE PARTITIONTREE");
   Partition* p=new Partition(GetMPProblem()->GetMPRegion(0)->GetRoadmap(), 0);
   LeafPartitionNode lp(p);
   m_pt = new PartitionTree(lp);
   LOG_DEBUG_MSG("END PARTITION TREE CREATION");	
   cout<<GetMPProblem()->GetMPStrategy()->GetPartitioningMethods()->GetPartitioningMethod(m_PartitioningMethod)->GetName()<<endl<<flush;
   LOG_DEBUG_MSG("START PARTITIONING");
   m_pt->CreateTree(GetMPProblem()->GetMPStrategy()->GetPartitioningMethods()->GetPartitioningMethod(m_PartitioningMethod), dynamic_cast<LeafPartitionNode*>(m_pt->GetRoot()), new InternalPartitionNode());
   LOG_DEBUG_MSG("END PARTITIONING");
   LOG_DEBUG_MSG("START PARTITION EVALUATION");
   if(GetMPProblem()->GetMPStrategy()->GetPartitioningEvaluators()!=NULL)
      EvaluatePartitions();
   LOG_DEBUG_MSG("END PARTITION EVALUATION");
   m_pt->WritePartitions(GetMPProblem(), GetMPProblem()->GetMPStrategy()->GetPartitioningMethods()->GetPartitioningMethod(m_PartitioningMethod)->GetClusteringDestination()+"region.");
}

void UAStrategy::CollectMinMaxBBX(){
   m_min.clear();
   m_max.clear();
   
   vector< vector< VID > > Clusters = GetPartitionsVID();
   typedef vector< vector< VID > >::iterator CIT;
   typedef vector< VID >::iterator NIT;

   Roadmap < CfgType, WeightType > * rm = GetMPProblem()->GetMPRegion(0)->GetRoadmap();
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
      
      if( itrCluster->size() < 1 ){ 
         Max[0] = Max[1] = Max[2] = Min[0] = Min[1] = Min[2] = 0;
      }
      else{
         //
         // At least 1 node in the cluster,
         // so grab it and use as the basis for the min/max values.
         //
         NIT itrNode = itrCluster->begin();
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
         while( itrNode != itrCluster->end() ){
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

   Environment *env = GetMPProblem()->GetEnvironment();
   BoundingBox *bb = GetMPProblem()->GetMPRegion(0)->GetBoundingBox();
   double robot_radius = env->GetMultiBody(env->GetRobotIndex())->GetBoundingSphereRadius();
   
   for(int x= 0; x<3; x++){
      for(int i = 0; i<m_min.size();i++){
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
   for(int x = 0; x<3; x++){
      vector<int> indx;
      for(int i =0; i<m_min.size();i++)indx.push_back(i);
      bool swapped;
      do{
         swapped=false;
         for(int i = 0; i<m_min.size()-1; i++){
            if(m_min[indx[i]][x]>m_min[indx[i+1]][x]){
               int temp = indx[i];
               indx[i]=indx[i+1];
               indx[i+1]=temp;
               swapped=true;
            }
         }
      }while(swapped);
      for(int i=0;i<indx.size()-1;i++){
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
   vector<vector<VID> > Clusters = GetPartitionsVID();

   cout<<"Beginning to redistribute nodes"<<endl;

   vector<double> averages;

   typedef vector<vector<VID> >::iterator CIT;
   typedef vector<double>::iterator DIT;
   for(CIT cit = Clusters.begin(); cit!=Clusters.end(); cit++){
      vector<double> features = GetMPProblem()->GetMPStrategy()->GetFeatures()->GetFeature(m_DistributionFeature)->Collect(*cit);
      double average = 0;
      for(DIT dit = features.begin(); dit!=features.end(); dit++){
         average+=(*dit);
      }
      average/=(double)features.size();
      averages.push_back(average);
   }
   
   double sum=0;
   vector<double> result;
   
   for(int RegionID = 0; RegionID<averages.size();RegionID++){
      
      cout<<"doing the redistribiution now"<<endl;
      sum=0;
      for(int i=0;i<averages.size(); i++){
         sum += (1-averages[i])*(1-averages[i]);
      }
      double VisibilityOfRegion = averages[RegionID];
      cout<<"V="<<VisibilityOfRegion<<"\t(1-v)^2="<<(1-averages[RegionID])*(1-averages[RegionID])<<"\tsum="<<sum<<endl<<flush;
      result.push_back((1-averages[RegionID])*(1-averages[RegionID])/sum);	
   }
   
   cout<<"done with redistribute now display results"<<result.size()<<endl<<flush;
   for(int probs = 0; probs<result.size(); probs++){
      cout<<probs<<" = "<<result[probs]<<endl<<flush;
   }

   return result;
}

void UAStrategy::UpdateBBToRange(int region){
   LOG_DEBUG_MSG("UAS::Enter UpdateBBToRange");
   m_hold.clear();
   if( region >=0 && region < m_min.size() ){
      BoundingBox *pMPEBoundBox = (GetMPProblem()->GetEnvironment())->GetBoundingBox();
      BoundingBox *pBoundBox = (GetMPProblem()->GetMPRegion(0))->GetBoundingBox();
      
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
   LOG_DEBUG_MSG("UAS::Exit UpdateBBToRange");
}

void UAStrategy::RestoreBB(){
   LOG_DEBUG_MSG("UAS::Enter RestoreBB");
   BoundingBox *pMPEBoundBox = (GetMPProblem()->GetEnvironment())->GetBoundingBox();
   BoundingBox *pBoundBox = (GetMPProblem()->GetMPRegion(0))->GetBoundingBox();
	int i=0;
   for(i=0; i<m_hold.size();i++){
      pBoundBox->SetParameter(i, m_hold[i].first, m_hold[i].second);
      pMPEBoundBox->SetParameter(i, m_hold[i].first, m_hold[i].second);
   }
	if(i>0){
      cout<<"START RESTORE BOUNDING BOX (CONNECTION) TO:"<<endl<<flush;
      pBoundBox->Print(cout, ':', ';');
	}
   
   LOG_DEBUG_MSG("UAS::Exit RestoreBB");
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

vector<vector<VID> > UAStrategy::GetPartitionsVID(){
   vector<vector<VID> > Clusters;
   for(int itrCluster =0; itrCluster < m_pt->GetRoot()->GetChildren().size(); ++itrCluster){
      Clusters.push_back( m_pt->GetRoot()->GetChildren()[itrCluster]->GetVIDs() );
   }
   return Clusters;
}

void UAStrategy::EvaluatePartitions(){   
   string filename=getBaseFilename()+".partitions";
   vector<vector<double> > eval=GetMPProblem()->GetMPStrategy()->GetPartitioningEvaluators()->Evaluate(filename, GetPartitions());
}

bool UAStrategy::EvaluateMap(int in_RegionID){
   bool mapPassedEvaluation = false;
   if(!m_EvaluatorLabels.empty()){
      Clock_Class EvalClock;
      stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Map Evaluation"; 
      EvalClock.StartClock(clockName.str().c_str());
      
      mapPassedEvaluation = true;
      for(vector<string>::iterator I = m_EvaluatorLabels.begin(); 
          I != m_EvaluatorLabels.end(); ++I){
         MapEvaluator<CfgType, WeightType>::conditional_type pEvaluator;
         pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
         Clock_Class EvalSubClock;
         stringstream evaluatorClockName; evaluatorClockName << "Iteration " << m_CurrentIteration << ", " << pEvaluator->GetName();
         EvalSubClock.StartClock(evaluatorClockName.str().c_str());
         
         cout << "\n\t";
         mapPassedEvaluation = pEvaluator->operator()(in_RegionID);
         
         cout << "\t";
         EvalSubClock.StopPrintClock();
         if(mapPassedEvaluation){
            cout << "\t  (passed)\n";
            return true;
         }
         else
            cout << "\t  (failed)\n";
         //if(!mapPassedEvaluation)
            //break;
      }
      EvalClock.StopPrintClock();
   }
   else{mapPassedEvaluation=true;}//avoid the infinite loop
   return mapPassedEvaluation;
}

void UAStrategy::WriteRegionsSeparate(){
   string baseFileName = getBaseFilename();
   int count=0;
   vector<PartitionNode*> pnodes = m_pt->GetRoot()->GetChildren();
   typedef vector<PartitionNode*>::iterator PIT;
   for (PIT pit=pnodes.begin(); pit!=pnodes.end(); pit++, count++) {
      vector<VID> vi = (*pit)->GetPartition()->GetVID();
      BoundingBox bb(6,3);// = (*pit)->GetPartition()->GetBoundingBox();
      for(int i = 0; i <3; i++){
         bb.SetParameter(i, m_min[count][i], m_max[count][i]);
      }
      MPRegion<CfgType,WeightType> eachRgn(*(GetMPProblem()->GetEnvironment()), bb, count+10, NULL);
      eachRgn.SetMPProblem(GetMPProblem());
      typedef vector<VID>::iterator VIT;
      for(VIT vit = vi.begin(); vit!=vi.end(); vit++){
         //vector<VID> tmp;
         //tmp.push_back(*vit);
         CfgType cfg = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap->find_vertex(*vit)->property();
         //vector<CfgType> vcfg = GetMPProblem()->GetMPRegion(0)->GetCfgFromVID(GetMPProblem()->GetMPRegion(0)->GetRoadmap(), tmp);
         eachRgn.GetRoadmap()->m_pRoadmap->AddVertex(cfg);
      }
      for(unsigned int k = 0; k<vi.size()-1; k++){
         //vector<VID> tmp;
         //tmp.push_back(vi[k]);
         //tmp.push_back(vi[k+1]);
         //vector<CfgType> vcfg = GetMPProblem()->GetMPRegion(0)->GetCfgFromVID(GetMPProblem()->GetMPRegion(0)->GetRoadmap(), tmp);
         CfgType cfg1 = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap->find_vertex(vi[k])->property();
         CfgType cfg2 = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap->find_vertex(vi[k+1])->property();
         eachRgn.GetRoadmap()->m_pRoadmap->AddEdge(cfg1,cfg2,0);
      }
      ostringstream oss;
      oss<<baseFileName<<"."<<count<<".map";
      ofstream  myofstream(oss.str().c_str());    
      if (!myofstream) {
         LOG_ERROR_MSG("print_feature_maps::WriteRoadmapForVizmo: can't open outfile: ");
         exit(-1);
      } 
      eachRgn.WriteRoadmapForVizmo(myofstream);
      myofstream.close();
   }
};

