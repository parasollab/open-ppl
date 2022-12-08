#include "GrowablePartitions.h"
#include "MPStrategy.h"
#include "Features.h"
#include "Partition.h"

GrowablePartitions::GrowablePartitions():PartitioningMethod(){};

GrowablePartitions::GrowablePartitions(XMLNode& in_Node, MPProblem* in_pProblem) : PartitioningMethod(in_Node, in_pProblem){
  this->SetName("growable");
   ParseXML(in_Node);
};

GrowablePartitions::~GrowablePartitions(){};

vector<Partition*> GrowablePartitions::MakePartitions(Partition &p){
   vector<string> featureLabels;
   typedef vector<pair<string, double> >::iterator FIT;
   for(FIT fit=m_Features.begin(); fit!=m_Features.end(); fit++){
      featureLabels.push_back(fit->first);
   }
   vector<vector<double> > features = GetMPProblem()->GetMPStrategy()->GetFeatures()->Collect(featureLabels, p.GetVID());
   vector<double> vidData;
   vector<double> minF, maxF;
   typedef vector<vector<double> >::iterator VDIT;
   for(VDIT vdit = features.begin(); vdit!=features.end(); vdit++){
      minF.push_back(*min_element(vdit->begin(), vdit->end()));
      maxF.push_back(*max_element(vdit->begin(), vdit->end()));
   }
   for(unsigned int i = 0; i<features[0].size(); i++){
      double avg = 0;
      int featureIndex = 0;
      for(VDIT vdit = features.begin(); vdit!=features.end(); vdit++){
         avg+=m_Features[featureIndex].second*((*vdit)[i]-minF[featureIndex])/(maxF[featureIndex]-minF[featureIndex]);
         featureIndex++;
      }
      avg/=features.size();
      vidData.push_back(avg);
   }


   typedef vector<pair<double, Partition*> >::iterator PIT;
   typedef vector<VID>::iterator VIT;

   vector<pair<double, Partition*> > freepartitions;
   double max =-0.1;
   double min = 1.1;
   //right now assumes visibility is used
   int vidCount = 0;
   for(VIT vit = p.GetVID().begin(); vit!=p.GetVID().end(); vit++){
      vector<VID> temp;
      temp.push_back(*vit);
      double vis = vidData[vidCount];
      freepartitions.push_back(pair<double, Partition*>(vis , new Partition(p.GetRoadmap(), temp)));
      if(vis > max) max = vis;
      if(vis < min) min = vis;
      vidCount++;
   }
   double sum1=0,sum2=0;
   double n = (double)freepartitions.size();
   for(PIT pit = freepartitions.begin(); pit!=freepartitions.end(); pit++){
      double vis = pit->first;
      sum1+=vis;
      sum2+=vis*vis;
   }
   double mean = sum1/n;
   double std = sqrt((sum2-sum1*mean)/(n-1.0));
   double range = (max - min)/3.0;
   double mid1 = min + range;
   double mid2 = min + 2.0 * range;
   cout<<"std:"<<std<<"\tmean:"<<mean<<"\tmin:"<<min<<"\tmax:"<<max<<"\tmid1:"<<mid1<<"\tmid2:"<<mid2<<endl<<flush;
   bool merge = false;
   do{
      merge=false;
      for(PIT pit = freepartitions.begin(); pit!=freepartitions.end(); pit++){
         int closest = FindClosestPartition(freepartitions, pit->second, pit->first, std);
         cout<<"Closest Indx::"<<closest<<endl;
         if(closest!=-1){
            Partition* closestpart = freepartitions[closest].second;
            double compvis = pit->first;
            double newvis = freepartitions[closest].first;
            compvis*=pit->second->GetVID().size();
            newvis*=closestpart->GetVID().size();
            int sizeOfMerge = pit->second->GetVID().size()+closestpart->GetVID().size();
            double vismerge = (compvis+newvis)/((double)sizeOfMerge);
            freepartitions.push_back(pair<double, Partition*>(vismerge, MergePartitions(pit->second, closestpart)));
            RemovePartition(freepartitions, pit->second);
            RemovePartition(freepartitions, closestpart);
            merge = true;
            pit--;
            cout<<"merge"<<endl;
         }
      }
   }while(merge);
   vector<Partition*> partition;
   int count = 0;
   for(PIT pit = freepartitions.begin(); pit!=freepartitions.end(); pit++){
      if(pit->second->GetVID().size()>1){
         partition.push_back(new Partition(*(pit->second)));
         partition[count]->GetBoundingBox().Print(cout);
         count++;
      }
   }
   return partition;
};

int GrowablePartitions::FindClosestPartition(vector<pair<double, Partition*> > &vp, Partition* p, double compare, double std){
   typedef vector<pair<double, Partition*> >::iterator PIT;
   typedef vector<VID>::iterator VIT;

   Roadmap<CfgType, WeightType> *rdmp= GetMPProblem()->GetRoadmap();
   vector< pair<size_t,VID> > ccs;
   stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>,size_t > cmap;
   get_cc_stats(*(rdmp->m_pRoadmap),cmap,ccs);

   int indx=-1;
   for(PIT pit = vp.begin(); pit!=vp.end(); pit++){
      indx++;
      if(!(*p==*(pit->second))){
         cmap.reset();
         if(is_same_cc(*(rdmp->m_pRoadmap), cmap, pit->second->GetVID()[0], p->GetVID()[0])){
            return indx;
         }
      }
   }

   vector<double> center = GetCenterOfPartition(p);
   double closestdistance = 100000000;
   int closeidx = -1;
   int index = -1;
   bool objfound = false;
   for(PIT pit = vp.begin(); pit!=vp.end(); pit++){
      index++;
      if(!(*p==*(pit->second))){
         vector<double> centerA = GetCenterOfPartition(pit->second);
         double total=0;
         for(size_t j=0; j<center.size(); j++){
            total+=(center[j]-centerA[j])*(center[j]-centerA[j]);
         }
         if(total<closestdistance){
            closestdistance=total;
            closeidx = index;
         }
      }
   }
   if(vp[closeidx].first>compare-std && vp[closeidx].first<compare+std){
      objfound=true;
   }
   if(objfound)return closeidx;
   else return -1;
};

void GrowablePartitions::RemovePartition(vector<pair<double, Partition*> > &vp, Partition* p){
   typedef vector<pair<double, Partition*> >::iterator PIT;
   for (PIT it = vp.begin(); it!=vp.end(); ++it){
      if(*((*it).second)==*p){
         vp.erase(it);
         return;
      }
   }
};

Partition* GrowablePartitions::MergePartitions(Partition* a, Partition* b){
   Partition* temp = new Partition(a->GetRoadmap(), a->GetID()+b->GetID());
   vector<VID> avid=a->GetVID();
   vector<VID> bvid=b->GetVID();
   typedef vector<VID>::iterator VIT;
   for(VIT vit = bvid.begin(); vit!=bvid.end(); vit++)
      avid.push_back(*vit);
   temp->SetVID(avid);
   return new Partition(*temp);
};

vector<double> GrowablePartitions::GetCenterOfPartition(Partition *p){
   vector<double> center;
   center.push_back(0);
   center.push_back(0);
   center.push_back(0);
   center.push_back(0);
   center.push_back(0);
   center.push_back(0);
   vector<VID> vid = p->GetVID();
   typedef vector<VID>::iterator VIT;
   for(VIT vit = vid.begin(); vit!=vid.end(); vit++){
      CfgType cfg = p->GetRoadmap()->m_pRoadmap->find_vertex(*vit)->property();
      vector<double> position = cfg.GetPosition();
      vector<double> orientation = cfg.GetOrientation();
      center[0]+=position[0];
      center[1]+=position[1];
      center[2]+=position[2];
      center[3]+=orientation[0];
      center[4]+=orientation[1];
      center[5]+=orientation[2];
   }
   center[0]/=vid.size();
   center[1]/=vid.size();
   center[2]/=vid.size();
   center[3]/=vid.size();
   center[4]/=vid.size();
   center[5]/=vid.size();
   return center;
};

