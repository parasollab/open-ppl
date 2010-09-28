#include "PGMeansClustering.h"
#include "MPStrategy.h"

using namespace std;

PGMeansClustering::PGMeansClustering():PartitioningMethod("PGMeansClustering", NULL){
};

PGMeansClustering::PGMeansClustering(XMLNodeReader& in_Node, MPProblem* in_pProblem):PartitioningMethod("PGMeansClustering", in_pProblem){
   ParseXML(in_Node);
};

PGMeansClustering::~PGMeansClustering(){};

void PGMeansClustering::ParseXML(XMLNodeReader& in_Node){

   XMLNodeReader::childiterator citr;
   for(citr = in_Node.children_begin(); citr!=in_Node.children_end(); citr++){
      if(citr->getName()=="Feature"){
         string name = citr->stringXMLParameter(string("Name"), true, string(""), string("FeatureName"));
         double weight = citr->numberXMLParameter(string("Weight"), true, 1.0, 0.0, 1.0, string("FeatureWeight"));
         m_Features.push_back(pair<string, double>(name, weight));
      }
   }

   SetLabel(in_Node.stringXMLParameter(string("Label"), true, string(""), string("PartitioningMethod")));
   SetClusteringDestination(in_Node.stringXMLParameter(string("destination"), true, string(""), string("PartitioningMethod")));
   in_Node.warnUnrequestedAttributes();
}

vector<Partition*> PGMeansClustering::MakePartitions(Partition &p){
   LOG_DEBUG_MSG("START PGMeans Clustering::MAKEPARTITIONS()");
   vector< vector<VID> > Clusters;
   vector<string> features;
   typedef vector<pair<string, double> >::iterator FIT;
   for(FIT fit=m_Features.begin(); fit!=m_Features.end(); fit++){
      features.push_back(fit->first);
   }
   vector<vector<double> > vidData = m_pProblem->GetMPStrategy()->GetFeatures()->Collect(features, p.GetVID());
   Cluster(p.GetVID(), Clusters, vidData);
   vector<Partition*> vp;
   for(unsigned int i =0 ; i<Clusters.size(); i++){
      vp.push_back(new Partition(p.GetRoadmap(), p.GetID()+i));
      vp[i]->SetVID(Clusters[i]);
      vp[i]->GetBoundingBox().Print(cout);
   }
   LOG_DEBUG_MSG("END PGMeans Clustering::MAKEPARTITIONS()");
   return vp;
};

void PGMeansClustering::Cluster(vector<VID> &IdSet, vector< vector< VID > > &RetClusters, vector<vector<double> >& features){ 
   
   cout <<"START PGMeans Clustering" << endl << flush;
   cout <<"Number of Features: " << features.size() << " Training Set Size: " << IdSet.size() << endl << flush; 

   //collect feature minumums and maximums
   
   vector< pair<double,double> > FeatureMinMax(features.size());

   typedef vector<vector<double> >::iterator VIT;
   typedef vector<double>::iterator DIT;

   int featureIndex=0;

   for(VIT vit=features.begin(); vit!=features.end(); vit++){
      int nodeIndex = 0;
      for(DIT dit=vit->begin();dit!=vit->end();dit++){

         if( nodeIndex == 0 ){
            FeatureMinMax[featureIndex].second = *dit;
            FeatureMinMax[featureIndex].first = *dit;
            nodeIndex++;
         }
         else if( *dit > FeatureMinMax[featureIndex].second )
            FeatureMinMax[featureIndex].second = *dit;
         else if( *dit < FeatureMinMax[featureIndex].first )
            FeatureMinMax[featureIndex].first = *dit;
      }
      featureIndex++;
   }

   //this exit is for IdSet being too small

   if( IdSet.size() < 3 ){

      cout << "VID set must have have at least the same number of elements as clusters requested." << endl << "VID Set Size: " << IdSet.size() << "\nClusters Requested: " << 3 << endl << "Unable to cluster this group...cluster_region_level call exiting... " << endl << endl << " WARNING: Final Cluster Count will not likely match requested number!!! " <<endl << endl;

      if( IdSet.size() )
         RetClusters.push_back( IdSet );
      
      return;
   }

   //print out the feature min/max values

   for(int featureNdx=0;featureNdx<features.size();featureNdx++){
      cout << featureNdx << ") MIN: "<< FeatureMinMax[featureNdx].first <<"  \tMAX: " <<FeatureMinMax[featureNdx].second << endl <<flush;
   }

   //normalize and output to a file

   ofstream outfile;
	string tempFileName;
	tempFileName = m_ClusteringDestination+"/SampleDataPoints.txt";
   outfile.open(tempFileName.c_str());
   if( IdSet.size() > 1 ){ 
      for(int PtNdx = 0; PtNdx < IdSet.size(); PtNdx++){
         for(int featureNdx=0;featureNdx<features.size();featureNdx++){
            features[featureNdx][PtNdx]=m_Features[featureNdx].second*
               (features[featureNdx][PtNdx] - FeatureMinMax[featureNdx].first) / 
               (FeatureMinMax[featureNdx].second - FeatureMinMax[featureNdx].first );
            //Output the data to the textfile such that matlab can read it
            outfile<<features[featureNdx][PtNdx]<<" "<<flush;
         }
         //add new line after every node data
         outfile<<endl<<flush;
      }
	}
   outfile.close();

   if( IdSet.size() > 0 ){

      //execute matlab code
      char currentWorkingDirectory[1024];
      getcwd(currentWorkingDirectory,PATH_MAX);
      cout<<"Current Working Directory = "<<currentWorkingDirectory<<endl<<flush;
      ostringstream oss;
      cout<<"Checking if processor is available..."<<endl;
      if (system(NULL)) puts ("System Ok");
      else exit (1);
      //For Bigspring
      setenv("LD_LIBRARY_PATH","/usr/lib:/lib:/share/apps/matlab704/bin/glnx86:/usr/java/jdk1.6.0_01/jre/lib/i386/client:/usr/java/jdk1.6.0_01/jre/lib/i386:.:/share/apps/matlab704/bin/glnx86/../../sys/os/glnx86/",1);
      //For matlock
      //  setenv("LD_LIBRARY_PATH","/usr/lib:/lib:/export/research/matlock/matlab704/bin/glnx86:/usr/java/jdk1.6.0_01/jre/lib/i386/client:/usr/java/jdk1.6.0_01/jre/lib/i386:/users/anshula/Documents/icra10Stuff/code/partitioning/Problems/PGMeansClusteringExecutable:.",1);

      oss<<currentWorkingDirectory<<"/../pgMeansClusteringExecutable/pgMeansClustering "<<currentWorkingDirectory<<"/"<<m_ClusteringDestination.c_str()<<"/";
      cout<<"Executing the following system command => "<<oss.str()<<endl<<flush;
      string buffer = oss.str();
      system (buffer.c_str());
      cout<<flush;

      //Read the number of clusters
      tempFileName=m_ClusteringDestination+"/numClusters.txt";
      ifstream inpFileNumCenters(tempFileName.c_str());
      int NumClusters;
      inpFileNumCenters>>NumClusters;
      cout<<"NumClusters::"<<NumClusters<<";"<<endl<<flush;
      inpFileNumCenters.close();
      //Read from the output file of the pgMeans clustering.
      //Put those vertices in the corresponding clusters
      int readData=0;
      tempFileName=m_ClusteringDestination+"/pathClusterInformation.txt";
      ifstream inFile(tempFileName.c_str());
      vector<int> tempClusters;
      RetClusters = vector< vector< VID > >(NumClusters);
      for(int i =0;i<IdSet.size();i++){
         inFile>>readData;
         cout<<"Reading: "<<readData<<flush;
         RetClusters[readData-1].push_back(i);
      }
      inFile.close();
   }    
   else{
      cout << " No Points available to cluster " << endl << endl <<flush;
      exit(-2);
   }
};
