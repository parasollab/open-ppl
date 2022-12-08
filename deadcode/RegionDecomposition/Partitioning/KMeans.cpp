#include "KMeans.h"
#include "MPStrategy.h"
#include "../../utils/Kmeans/KMlocal.h"
#include "Features.h"
#include "Partition.h"

KMeans::KMeans():PartitioningMethod(){};

KMeans::KMeans(XMLNode& in_Node, MPProblem* in_pProblem):PartitioningMethod(in_Node, in_pProblem){
  this->SetName("kmeans");
   ParseXML(in_Node);
};

KMeans::~KMeans(){};

vector<Partition*> KMeans::MakePartitions(Partition &p){
   vector< vector<VID> > Clusters;
   vector<string> features;
   typedef vector<pair<string, double> >::iterator FIT;
   for(FIT fit=m_Features.begin(); fit!=m_Features.end(); fit++){
      features.push_back(fit->first);
   }
   vector<vector<double> > vidData = GetMPProblem()->GetMPStrategy()->GetFeatures()->Collect(features, p.GetVID());
   Cluster(p.GetVID(), Clusters, vidData);
   vector<Partition*> vp;
   for(size_t i =0 ; i<Clusters.size(); i++){
      vp.push_back(new Partition(p.GetRoadmap(), p.GetID()+i));
      vp[i]->SetVID(Clusters[i]);
      vp[i]->GetBoundingBox().Print(cout);
   }
   return vp;
};

void KMeans::Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features) {

   //define necessary variables
   int numberFeatures=features.size();
   int maxPts=10000;  	// max number nodes/itr requested
   int nPts=IdSet.size();

   KMterm  term(100, 0, 0, 0,              // run for 100 stages
                0.10,                   // min consec RDL
                0.10,                   // min accum RDL
                3,                      // max run stages
                0.50,                   // init. prob. of acceptance
                10,                     // temp. run length
                0.95);                  // temp. reduction factor

   cout <<"START KMEANS" << endl << flush;
   cout <<"Number of Features: " << numberFeatures << " Training Set Size: " << IdSet.size() << endl << flush;

   //exit prematurely if the IdSet is too small

   if( IdSet.size() < 3 ){

      cout << "VID set must have have at least the same number of elements as clusters requested." << endl << "VID Set Size: " << IdSet.size() << "\nClusters Requested: " << 3 << endl << "Unable to cluster this group...cluster_region_level call exiting... " << endl << endl << " WARNING: Final Cluster Count will not likely match requested number!!! " <<endl << endl;

      if( IdSet.size() )
         RetClusters.push_back( IdSet );

      return;
   }

   //gather the features and feature min/max values

   KMdata dataPts(numberFeatures, maxPts);
   vector< pair<double,double> > FeatureMinMax(numberFeatures);

   typedef vector<vector<double> >::iterator VIT;
   typedef vector<double>::iterator DIT;

   int featureIndex=0;

   for(VIT vit=features.begin(); vit!=features.end(); vit++){
      int nodeIndex=0;
      FeatureMinMax[featureIndex].first = *min_element(vit->begin(), vit->end());
      FeatureMinMax[featureIndex].second = *max_element(vit->begin(), vit->end());
      for(DIT dit=vit->begin();dit!=vit->end();dit++){
         dataPts[nodeIndex][featureIndex]=*dit;
         nodeIndex++;
      }
      featureIndex++;
   }

   //print min/max feature values

   for(int featureNdx=0;featureNdx<numberFeatures;featureNdx++){
      cout << featureNdx << ") MIN: "<< FeatureMinMax[featureNdx].first <<"  \tMAX: " <<FeatureMinMax[featureNdx].second << endl <<flush;
   }

   //normalize the features

   if( nPts > 1 ){
      for(int PtNdx = 0; PtNdx < nPts; PtNdx++){
         for(int featureNdx=0;featureNdx<numberFeatures;featureNdx++){
            dataPts[PtNdx][featureNdx]=m_Features[featureNdx].second*
               (dataPts[PtNdx][featureNdx] - FeatureMinMax[featureNdx].first) /
               (FeatureMinMax[featureNdx].second - FeatureMinMax[featureNdx].first );
         }
      }
   }

   //do kmeans 15 times to figure out what the optimal k value is

   if( nPts > 0 ){
      vector<double> data_set;
      for(int i = 1; i<15; i++){
         cout<<i<<" setNPts()"<<endl<<flush;
         dataPts.setNPts(nPts);
         cout<<"buildKCTree()"<<endl<<flush;
         dataPts.buildKcTree();
         cout<<"ctrs"<<endl<<flush;
         KMfilterCenters ctrs(i, dataPts);
         cout<<"Lloyds"<<endl<<flush;
         KMlocalLloyds kmLloyds(ctrs, term);         // repeated Lloyd's
         cout<<"execute"<<endl<<flush;
         ctrs = kmLloyds.execute();                  // execute
         cout<<"after execute"<<endl<<flush;
         cout << "(Center Points:\n"<<flush;
         ctrs.print();
         cout << ")\n";
         int NumCenters = ctrs.getK();
         cout<<"NumCenters::"<<NumCenters<<endl<<flush;
         data_set.push_back(ctrs.getDist(false)/double(ctrs.getNPts()));
         cout << "Average distortion: " <<  ctrs.getDist(false)/double(ctrs.getNPts()) << "\n" << flush;
      }

      //take the first derivative of the distortion data
      vector<double> firstD;
      for(size_t i = 0; i<data_set.size()-1; i++){
         firstD.push_back(data_set[i+1]-data_set[i]);
      }

      //second derivative
      vector<double> secondD;
      for(size_t i = 0; i<firstD.size()-1; i++){
         secondD.push_back(firstD[i+1]-firstD[i]);
      }
      //find the max gain in information to determine the optimal number of clusters
      size_t max = 0;
      for(size_t i = 0; i<secondD.size(); i++){
         if(secondD[i]>secondD[max]){
            max=i;
         }
      }

      //DO the final Kmeans iteration with the optimal number of clusters

      max+=3;

      cout<<"setPts()"<<endl<<flush;
      dataPts.setNPts(nPts);
      cout<<"buildKCTree()"<<endl<<flush;
      dataPts.buildKcTree();
      cout<<"ctrs"<<endl<<flush;
      KMfilterCenters ctrs(max, dataPts);
      cout<<"Lloyds"<<endl<<flush;
      KMlocalLloyds kmLloyds(ctrs, term);         // repeated Lloyd's
      cout<<"execute"<<endl<<flush;
      ctrs = kmLloyds.execute();                  // execute
      cout<<"after execute"<<endl<<flush;
      cout << "(Center Points:\n"<<flush;
      ctrs.print();
      cout << ")\n";
      int NumCenters = ctrs.getK();
      cout<<"NumCenters::"<<NumCenters<<endl<<flush;

      cout << "Average distortion: " <<  ctrs.getDist(false)/double(ctrs.getNPts()) << "\n" << flush;

      //assign node VIDs to the centers

      KMctrIdxArray closeCtr = new KMctrIdx[dataPts.getNPts()];
      double* sqDist = new double[dataPts.getNPts()];
      ctrs.getAssignments(closeCtr, sqDist);

      int cluster=-1;
	   RetClusters.clear();
      RetClusters = vector< vector< VID > >( NumCenters );
      for(int i=0;i<nPts;i++){
         cluster=closeCtr[i];
         RetClusters[cluster].push_back( IdSet[i] );
      }

      delete [] closeCtr;
      delete [] sqDist;
   }
   else{
      cout << " No Points available to cluster " << endl << endl <<flush;
      exit(-2);
   }
};
