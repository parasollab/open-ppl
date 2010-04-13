#ifndef ClosedChainStrategy_cpp
#define ClosedChainStrategy_cpp

#include "ClosedChainStrategy.h"
ClosedChainStrategy::ClosedChainStrategy(XMLNodeReader& in_Node, ClosedChainProblem* in_pProblem) : MPStrategy(in_Node, in_pProblem)
{
  CCProblem=in_pProblem;
  //read input
  ParseXML(in_Node);

}
ClosedChainStrategy::~ClosedChainStrategy(){}

void ClosedChainStrategy::ParseXML(XMLNodeReader& in_Node){
  is_gamma_random=false;
  num_iterations  = in_Node.numberXMLParameter("iterations", false, int(10),
					       int(1), int(100000),
					       "number of iterations");

  gamma = in_Node.numberXMLParameter("gamma", false, double(0.0),
					       double(0.0), double(1.0),
					       "gamma used by reachable code");

}

void ClosedChainStrategy::PrintOptions(ostream& out_os){
  out_os << "ClosedChainStrategy::PrintOptions:\n";
}

/*
void ClosedChainStrategy::operator()()
{
  (*this)(GetMPProblem()->CreateMPRegion());
}
*/

void ClosedChainStrategy::operator()(int in_RegionID){}
void ClosedChainStrategy::Solve(){
  cout<<"in ClosedChainStrategy::Solve()"<<endl;
  //num_iterations=10;//debugging code
  cout << "Sampling " << num_iterations << " configurations...\n";
  double sampling_time = 0.0, collision_time;
  int attempts = 0;
  //MultiBody mb; 
  //boost::shared_ptr<MultiBody> subsetOfRobot;
  for(int count = 0; count < num_iterations; ++count){
    bool colliding = true;
    while(colliding)
    {
      if(is_gamma_random)
        gamma = drand48();
      //cout << "\tgamma = " << gamma << "...\n";

      //sample closed cfg  
      bool bSampleSucceeded = false;  
      Clock_Class GenClock;  
      GenClock.StartClock("Node generation");  
      while(!bSampleSucceeded)  
      {  
        attempts++;

        //re-initialize cfg  
        for(size_t i=0; i<CCProblem->g_loopRoots.size(); ++i)  
          CCProblem->g_loopRoots[i]->ResetTree();  
        CCProblem->ConfigBase(CCProblem->GetEnvironment());
        //ConfigEnvironment(&env);
         
        bSampleSucceeded = true;  
        for(size_t i=0; i< CCProblem->g_loopRoots.size(); ++i)  
        {      
          //sample closed loop 

          //set ear constraint
          double ear_length = 0;
          if(!CCProblem->g_non_ears[i].empty())
          {
            //compute ear required length
            triple<int,int,int> ear_joint1 = *(find_if(CCProblem->g_earJoints.begin(), CCProblem->g_earJoints.end(), first_is<int, triple<int,int,int> >(CCProblem->g_ears[i].front())));
            triple<int,int,int> ear_joint2 = *(find_if(CCProblem->g_earJoints.begin(), CCProblem->g_earJoints.end(), first_is<int, triple<int,int,int> >(CCProblem->g_ears[i].back())));
            /*
            cout << "setting ear constraint for loop " << i << endl;
            cout << "\tear_joint1: " << ear_joint1.first << "," << ear_joint1.second << "," << ear_joint1.third << endl;
            cout << "\tear_joint2: " << ear_joint2.first << "," << ear_joint2.second << "," << ear_joint2.third << endl;
            */

            Vector3D joint1, joint2;
            {
            //note, following assumes link orientated along x-axis for longest length
            //get world coords for joint1.second
	      GMSPolyhedron& joint1_bbox1 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint1.second)->GetWorldBoundingBox();
            //cout << "\tjoint1_bbox1:\n";
            //for_each(joint1_bbox1.vertexList, joint1_bbox1.vertexList + joint1_bbox1.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");

            Vector3D joint1_bbox1_endpoint1;
            for(int j=0; j<4; ++j)
              joint1_bbox1_endpoint1 = joint1_bbox1_endpoint1 + joint1_bbox1.vertexList[j];
            joint1_bbox1_endpoint1 = joint1_bbox1_endpoint1 / 4;
            //cout << "\tjoint1_bbox1_endpoint1: " << joint1_bbox1_endpoint1 << endl;

            Vector3D joint1_bbox1_endpoint2;
            for(int j=4; j<8; ++j)
              joint1_bbox1_endpoint2 = joint1_bbox1_endpoint2 + joint1_bbox1.vertexList[j];
            joint1_bbox1_endpoint2 = joint1_bbox1_endpoint2 / 4;
            //cout << "\tjoint1_bbox1_endpoint2: " << joint1_bbox1_endpoint2 << endl;

            //get world coords for joint1.third
            GMSPolyhedron& joint1_bbox2 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint1.third)->GetWorldBoundingBox();
            //cout << "\tjoint1_bbox2:\n";
            //for_each(joint1_bbox2.vertexList, joint1_bbox2.vertexList + joint1_bbox2.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");
            
            Vector3D joint1_bbox2_endpoint1;
            for(int j=0; j<4; ++j)
              joint1_bbox2_endpoint1 = joint1_bbox2_endpoint1 + joint1_bbox2.vertexList[j];
            joint1_bbox2_endpoint1 = joint1_bbox2_endpoint1 / 4;
            //cout << "\tjoint1_bbox2_endpoint1: " << joint1_bbox2_endpoint1 << endl;

            Vector3D joint1_bbox2_endpoint2;
            for(int j=4; j<8; ++j)
              joint1_bbox2_endpoint2 = joint1_bbox2_endpoint2 + joint1_bbox2.vertexList[j];
            joint1_bbox2_endpoint2 = joint1_bbox2_endpoint2 / 4;
            //cout << "\tjoint1_bbox2_endpoint2: " << joint1_bbox2_endpoint2 << endl;

            //find intersection coord
            Vector3D a = joint1_bbox1_endpoint1 - joint1_bbox2_endpoint1;
            Vector3D b = joint1_bbox1_endpoint1 - joint1_bbox2_endpoint2;
            Vector3D c = joint1_bbox1_endpoint2 - joint1_bbox2_endpoint1;
            Vector3D d = joint1_bbox1_endpoint2 - joint1_bbox2_endpoint2;
            //cout << "\t\ta: " << a << "\tb: " << b << "\tc: " << c << "\td: " << d << endl;

            if(a.magnitude() <= b.magnitude() && a.magnitude() <= c.magnitude() && a.magnitude() <= d.magnitude())
              joint1 = (joint1_bbox1_endpoint1 + joint1_bbox2_endpoint1)/2;
            else if(b.magnitude() <= a.magnitude() && b.magnitude() <= c.magnitude() && b.magnitude() <= d.magnitude())
              joint1 = (joint1_bbox1_endpoint1 + joint1_bbox2_endpoint2)/2;
            else if(c.magnitude() <= a.magnitude() && c.magnitude() <= b.magnitude() && c.magnitude() <= d.magnitude())
              joint1 = (joint1_bbox1_endpoint2 + joint1_bbox2_endpoint1)/2;
            else
              joint1 = (joint1_bbox1_endpoint2 + joint1_bbox2_endpoint2)/2;

            //cout << "\tIntersection point for joint1: " << joint1 << endl;
            }

            {
            //repeat for joint2
            //get world coords for joint2.second
	      GMSPolyhedron& joint2_bbox1 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint2.second)->GetWorldBoundingBox();
            //cout << "\tjoint2_bbox1:\n";
            //for_each(joint2_bbox1.vertexList, joint2_bbox1.vertexList + joint2_bbox1.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");
            
            Vector3D joint2_bbox1_endpoint1;
            for(int j=0; j<4; ++j)
              joint2_bbox1_endpoint1 = joint2_bbox1_endpoint1 + joint2_bbox1.vertexList[j];
            joint2_bbox1_endpoint1 = joint2_bbox1_endpoint1 / 4;
            //cout << "\tjoint2_bbox1_endpoint1: " << joint2_bbox1_endpoint1 << endl;

            Vector3D joint2_bbox1_endpoint2;
            for(int j=4; j<8; ++j)
              joint2_bbox1_endpoint2 = joint2_bbox1_endpoint2 + joint2_bbox1.vertexList[j];
            joint2_bbox1_endpoint2 = joint2_bbox1_endpoint2 / 4;
            //cout << "\tjoint2_bbox1_endpoint2: " << joint2_bbox1_endpoint2 << endl;

            //get world coords for joint2.third
            GMSPolyhedron& joint2_bbox2 = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBody(ear_joint2.third)->GetWorldBoundingBox();
            //cout << "\tjoint2_bbox2:\n";
            //for_each(joint2_bbox2.vertexList, joint2_bbox2.vertexList + joint2_bbox2.numVertices, cout << boost::lambda::constant("\t\t") << boost::lambda::_1 << "\n");
            
            Vector3D joint2_bbox2_endpoint1;
            for(int j=0; j<4; ++j)
              joint2_bbox2_endpoint1 = joint2_bbox2_endpoint1 + joint2_bbox2.vertexList[j];
            joint2_bbox2_endpoint1 = joint2_bbox2_endpoint1 / 4;
            //cout << "\tjoint2_bbox2_endpoint1: " << joint2_bbox2_endpoint1 << endl;

            Vector3D joint2_bbox2_endpoint2;
            for(int j=4; j<8; ++j)
              joint2_bbox2_endpoint2 = joint2_bbox2_endpoint2 + joint2_bbox2.vertexList[j];
            joint2_bbox2_endpoint2 = joint2_bbox2_endpoint2 / 4;
            //cout << "\tjoint2_bbox2_endpoint2: " << joint2_bbox2_endpoint2 << endl;

            //find intersection coord
            Vector3D a = joint2_bbox1_endpoint1 - joint2_bbox2_endpoint1;
            Vector3D b = joint2_bbox1_endpoint1 - joint2_bbox2_endpoint2;
            Vector3D c = joint2_bbox1_endpoint2 - joint2_bbox2_endpoint1;
            Vector3D d = joint2_bbox1_endpoint2 - joint2_bbox2_endpoint2;
            //cout << "\t\ta: " << a << "\tb: " << b << "\tc: " << c << "\td: " << d << endl;

            if(a.magnitude() <= b.magnitude() && a.magnitude() <= c.magnitude() && a.magnitude() <= d.magnitude())
              joint2 = (joint2_bbox1_endpoint1 + joint2_bbox2_endpoint1)/2;
            else if(b.magnitude() <= a.magnitude() && b.magnitude() <= c.magnitude() && b.magnitude() <= d.magnitude())
              joint2 = (joint2_bbox1_endpoint1 + joint2_bbox2_endpoint2)/2;
            else if(c.magnitude() <= a.magnitude() && c.magnitude() <= b.magnitude() && c.magnitude() <= d.magnitude())
              joint2 = (joint2_bbox1_endpoint2 + joint2_bbox2_endpoint1)/2;
            else
              joint2 = (joint2_bbox1_endpoint2 + joint2_bbox2_endpoint2)/2;

            //cout << "\tIntersection point for joint2: " << joint2 << endl;
            }

            //subtract to get lengtth
            ear_length = (joint1 - joint2).magnitude();

          }
          //cout << "\tsetting length: " << ear_length << "\tfor link " << g_ear_roots[i]->GetID() << endl;
          //g_ear_roots[i]->SetAvailableRange(Range(ear_length, ear_length));

          //cout << "Attempting to sample loop rooted at " << g_loopRoots[i]->GetID() << "...\n";  
          //cout << "Attempting to sample loop rooted at " << g_ear_roots[i]->GetID() << "...\n";  
          //if(!g_loopRoots[i]->RecursiveSample(0, true, gamma))  
          if(!CCProblem->g_ear_roots[i]->RecursiveSample(ear_length, true, gamma))
          {  
            cout << "\tCan't close loop " << i << "!\n";  
            bSampleSucceeded = false;  
            break;  
          } else {
            //ConfigEar(&env, g_loopRoots[i]);
            //cout << "configuring successfully sampled ear\n";
            CCProblem->ConfigEar(CCProblem->GetEnvironment(), CCProblem->g_ear_roots[i], CCProblem->g_loopRoots[i]);
	  
            /*
            {
              for(int e=0; e<=i; ++e)
              {
                cout << "Joint Coords for ear " << e << ":\n";
                PrintEarJointCoords(&env, g_ear_roots[e]);
              }
            }
            */
            Clock_Class CollisionClock;
            CollisionClock.StartClock("Collision check");
            string CallName = "RandomSample";
            // WARNING: DANGEROUS CollisionDetection Optimization....
            //adjust num free bodies
            int num_bodies = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->GetFreeBodyCount();
            vector<int> actual_ear_links;
	    get_actual_links_of(CCProblem->g_ear_roots[i], actual_ear_links);
	    boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody(CCProblem->GetEnvironment()));
	    //MultiBody mb = *(CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex()).get());
	    //boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody);
	    //for(vector<int>::iterator iter = actual_ear_links.begin(); iter<actual_ear_links.end(); iter++){
	    //cout<<"here "<<actual_ear_links.back()<<"_"<<<num_bodies<<endl;
	    for(size_t l=0; l<actual_ear_links.back();  ++l){
	   
	      shared_ptr<FreeBody> freeBody = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex()).get()->GetFreeBody(l);
	      subsetOfRobot.get()->AddBody(freeBody);
	    }
            get_actual_links_of(CCProblem->g_ear_roots[i], actual_ear_links);
	    CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->SetFreeBodyCount(actual_ear_links.back());
	    //colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo, boost::shared_ptr<MultiBody>((MultiBody*)NULL), true, &CallName);
	    colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo, subsetOfRobot, true, &CallName);
	    //colliding=false;//debugging code
            //reset num free bodies
      
	    /*
	    if(i<CCProblem->g_loopRoots.size()-1){
	      colliding=false;
	    }
	    */
	       
            CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex())->SetFreeBodyCount(num_bodies);

            CollisionClock.StopClock();
            collision_time += CollisionClock.GetClock_SEC();
	    //if(i<CCProblem->g_loopRoots.size()){
	    //  colliding=false;//debugging code
	    //}
       

            if(colliding){
              bSampleSucceeded = false;
              break;
            }
           
          }
	  //cout<<"end for loop"<<endl;
        }  
	//cout<<"out for loop"<<endl;
      }
      //cout<<"sample succeded"<<endl;
      /*
      //inport cfg
      for(size_t i=0; i<g_loopRoots.size(); ++i)
      {
        g_loopRoots[i]->ImportTreeLinkLength(input_lengths[count][i], input_convexities[count][i]);
      }
      */
    
      //export lengths/convexities  
      vector<vector<double> > lengths;  
      vector<vector<int> >  convexities;  
      for(size_t i=0; i<CCProblem->g_loopRoots.size(); ++i)  
      {  
        vector<double> l;  
        vector<int> c;  
        CCProblem->g_loopRoots[i]->ExportTreeLinkLength(l, c);  
        lengths.push_back(l);  
        convexities.push_back(c);  
      }
      GenClock.StopClock();  
      sampling_time += GenClock.GetClock_SEC();

      //output sampled cfg  
      /*
      for(size_t i=0; i<g_loopRoots.size(); ++i)  
      {  
        cout << "Tree " << i << endl;  
        cout << "length of tree "<< i <<  ":" << endl;  
        copy(lengths[i].begin(), lengths[i].end(), ostream_iterator<double>(cout, " "));  
        cout << endl;  
        cout << "convexities of tree "<< i <<  ":" << endl;  
        copy(convexities[i].begin(), convexities[i].end(), ostream_iterator<double>(cout, " "));  
        cout << endl;  
      }
      PrintConfiguration(&env, cout); cout << endl;
      */

      //compute collision
      Clock_Class CollisionClock;
      CollisionClock.StartClock("Collision check");
      //ConfigEnvironment(&env);
      string CallName = "RandomSample";
     
      //colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo,  boost::shared_ptr<MultiBody>((MultiBody*)NULL), true, &CallName);
      //for(vector<int>::iterator iter = actual_ear_links.begin(); iter<actual_ear_links.end(); iter++){
      boost::shared_ptr<MultiBody> subsetOfRobot =  boost::shared_ptr<MultiBody>(new MultiBody(CCProblem->GetEnvironment()));
      for(int i=0; i<CCProblem->g_loopRoots.size(); ++i){
	//cout<<"ear link ="<<*iter<<endl;
	shared_ptr<FreeBody> freeBody = CCProblem->GetEnvironment()->GetMultiBody(CCProblem->GetEnvironment()->GetRobotIndex()).get()->GetFreeBody(i);
	subsetOfRobot.get()->AddBody(freeBody);
      }
      colliding = CCProblem->GetCollisionDetection()->IsInCollision(CCProblem->GetEnvironment(), Stats, _cdInfo, subsetOfRobot, true, &CallName);
      
      colliding=false;
      CollisionClock.StopClock();
      collision_time += CollisionClock.GetClock_SEC();
      
       
    }
    /*
    //output sampled cfg  
    for(size_t i=0; i<g_loopRoots.size(); ++i)  
    {  
      vector<double> l;  
      vector<int> c;  
      g_loopRoots[i]->ExportTreeLinkLength(l, c);  
      
      cout << "Tree " << i << endl;  
      cout << "length of tree "<< i <<  ":" << endl;  
      copy(l.begin(), l.end(), ostream_iterator<double>(cout, " "));  
      cout << endl;  
      cout << "convexities of tree "<< i <<  ":" << endl;  
      copy(c.begin(), c.end(), ostream_iterator<double>(cout, " "));  
      cout << endl;  
    }
    */
    CCProblem->PrintConfiguration(CCProblem->GetEnvironment(), cout);
	  
  }
  cout << "Total sampling time: " << sampling_time << endl;
  cout << "Total collision time: " << collision_time << endl;
  cout << "Total time: " << (sampling_time + collision_time) << endl;
  cout << "Attempts: " << attempts << endl;
  //end move to solve function
}
#endif

