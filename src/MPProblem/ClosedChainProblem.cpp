#ifndef ClosedChainProblem_cpp
#define ClosedChainProblem_cpp

#include "ClosedChainProblem.h"
#include "ValidityChecker.h"

ClosedChainProblem::
ClosedChainProblem(XMLNode& in_Node) : MPProblem(in_Node, false)
{
  ParseXML(in_Node);

  vector<cd_predefined> cdtypes = m_pValidityChecker->GetSelectedCDTypes();
  for(vector<cd_predefined>::iterator C = cdtypes.begin(); C != cdtypes.end(); ++C)
    m_pEnvironment->buildCDstructure(*C);
}

ClosedChainProblem::
~ClosedChainProblem()
{
  //deallocate all vectors of pointers
  vector<Link*>::iterator linkDel;
  for (linkDel = g_baseLinks.begin(); linkDel != g_baseLinks.end(); linkDel++){
    delete *linkDel;
    *linkDel = NULL;
  }
  for (linkDel = g_loopRoots.begin(); linkDel != g_loopRoots.end(); linkDel++){
    *linkDel = NULL;
    delete *linkDel;
  }
  for (linkDel = g_ear_roots.begin(); linkDel != g_ear_roots.end(); linkDel++){
    delete *linkDel;
    *linkDel = NULL;
  }

  vector<pair<Link*,Link*> >::iterator pairDelIter;
  for (pairDelIter = g_cfgJoints.begin(); pairDelIter != g_cfgJoints.end(); pairDelIter++){
    delete (*pairDelIter).first;
    (*pairDelIter).first = NULL;
    delete (*pairDelIter).second;
    (*pairDelIter).second = NULL;
  }
}

void ClosedChainProblem::
ParseXML(XMLNode& in_Node) {

  for(XMLNode::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr)
    if(!this->ParseChild(citr)){
      if(citr->getName() == "links_file")
      {
        string filename = citr->Read(string("filename"), true, string(""),string("Links File Name"));

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
        bool is_closed_chain = citr->Read("closed_chain", true, true, "Flag if tree represents a closed chain or not");
        double rdres = citr->Read("rdres", false, 0.05, 0.0, 1000.0, "resolution for interpolating in reachable distance space");
        cout<<"filename="<<filename<<endl;
        cout<<"parsing file"<<endl;
        //ParseLinksFile(filename.c_str());
        CfgType::initialize_link_tree(filename.c_str());
        CfgType::is_closed_chain = is_closed_chain;
        CfgType::rdres = rdres;
        cout<<"file "<<filename<<" has been parsed"<<endl;
#else
        cerr << "Warning, attempting to use ClosedChainProblem with a non-reachable distance cfg type, exiting.\n";
        exit(-1);
#endif
      } else
        citr->warnUnknownNode();
    }
}


bool ClosedChainProblem::ParseRealLink(ifstream &fin)
  {
    int myID = -1;
    fin >> myID;

    double minRange = -1, maxRange = -1;
    fin >> minRange;
    fin >> maxRange;

    cout << "myID:" << myID << " minRange:" << minRange << " maxRange:" << maxRange << endl;

    if(g_baseLinks[myID] == NULL)
      {
	g_baseLinks[myID] = new Link(myID, minRange, maxRange);
      }
    else
      {
	cerr << "ERROR: duplicate definition of link " << myID << endl;
	exit(1);
      }

    return true;
  }


bool ClosedChainProblem::ParseVirtualLink(ifstream &fin)
{
  int myID = -1, lcID = -1, rcID = -1;
  fin >> myID;
  fin >> lcID;
  fin >> rcID;
  cout << "myID: " << myID << " lcID:" << lcID << " rcID:" << rcID << endl;

  if(g_baseLinks[myID] == NULL)
    {
      if(g_baseLinks[lcID] && g_baseLinks[rcID])
	{
	  g_baseLinks[myID] = new Link(myID, g_baseLinks[lcID], g_baseLinks[rcID]);
	}
      else
	{
	  cerr << "ERROR: " << myID << endl;
	  exit(1);
	}
    }
  else
    {
      cerr << "ERROR: duplicate definition of link " << myID << endl;
      exit(1);
    }

  return true;
}




bool ClosedChainProblem::ParseLoop(ifstream &fin)
{
  int loopSize = 0;
  fin >> loopSize;

  vector<int> loopLinksID;
  vector<Link *> loopLinks;
  for(int i=0; i<loopSize; ++i)
  {
    int linkID = -1;
    fin >> linkID;
    loopLinksID.push_back(linkID);
    loopLinks.push_back(g_baseLinks[linkID]);
  }
  cout << "Loop: ";
  copy(loopLinksID.begin(), loopLinksID.end(), ostream_iterator<int>(cout, " "));
  cout << endl;

  Link *newTree = BuildTree(0, loopLinks.size()-1, loopLinks);
  g_loopRoots.push_back(newTree);

  return true;
}



bool ClosedChainProblem::ParseCfgJoints(ifstream &fin)
{
 int numAngles = 0;
 fin >> numAngles;

 vector<pair<int, int> > jointLinkIDs;
 for(int i=0; i<numAngles; ++i)
 {
   int linkID1 = -1, linkID2 = -1;
   fin >> linkID1;
   fin >> linkID2;
   jointLinkIDs.push_back(pair<int, int>(linkID1, linkID2));
   g_cfgJoints.push_back(pair<Link*, Link *>(g_baseLinks[linkID1], g_baseLinks[linkID2]));
 }

 cout << "reading joints: " << endl;
 for(size_t i=0; i<jointLinkIDs.size(); ++i)
   cout << jointLinkIDs[i].first << "," << jointLinkIDs[i].second << endl;

 return true;
}


bool ClosedChainProblem::ParseEarJoints(ifstream &fin)
{
 int numJoints = 0;
 fin >> numJoints;

 for(int i=0; i<numJoints; ++i)
 {
   int linkID1 = -1, linkID2 = -1, linkID3 = -1;
   fin >> linkID1;
   fin >> linkID2;
   fin >> linkID3;
   g_earJoints.push_back(triple<int, int, int>(linkID1, linkID2, linkID3));
 }

 cout << "reading ear joints: " << endl;
 for(size_t i=0; i<g_earJoints.size(); ++i)
   cout << g_earJoints[i].first << "," << g_earJoints[i].second << "," << g_earJoints[i].third << endl;

 return true;
}


 void  ConfigEar(Environment*env, Link* ear_root, Link* loop_root, double base_link_angle);


double
ClosedChainProblem::MyCalculateJointAngle(Environment* env, Link* link1, Link* link2)
{
  for(vector<Link*>::iterator E = g_ear_roots.begin(); E != g_ear_roots.end(); ++E)
  {
    vector<int> actual_links;
    get_actual_links_of(*E, actual_links);

    if(find(actual_links.begin(), actual_links.end(), link1->GetID()) != actual_links.end() &&
       find(actual_links.begin(), actual_links.end(), link2->GetID()) != actual_links.end())
    {
      //cout << "\tjoint angle w/in an ear, calling old method\n";
      return Link::CalculateJointAngle(link1, link2);
    }
  }

  //cout << "MyCalculateJointAngle(" << link1->GetID() << ", " << link2->GetID() << ")\n";
  //cout << "\tjoint angle not w/in an ear\n";
  Link* ear_root_link = NULL;
  Link* tree_root_link = NULL;
  for(size_t i=0; i<min(g_loopRoots.size(), g_ear_roots.size()); ++i)
  {
    vector<int> actual_links;
    get_actual_links_of(g_ear_roots[i], actual_links);

    //assume ear link is link2
    if(find(actual_links.begin(), actual_links.end(), link2->GetID()) != actual_links.end())
    {
      ear_root_link = g_ear_roots[i];
      tree_root_link = g_loopRoots[i];
      break;
    }
  }
  if(ear_root_link == NULL)
  {
    cerr << "Error, did not find link " << link2->GetID() << " in an ear\n";
    exit(-1);
  }

  double angle = PI;


  //cout << "\t\tcalling ConfigEar with ear_root = " << ear_root_link->GetID() << ", tree_root_link = " << tree_root_link->GetID() << ", and angle = " << angle << "\n";
  ConfigEar(env, ear_root_link, tree_root_link, angle);

  //compute new angle
  //assume orientations of links now, later compute automatically from earJoints

  GMSPolyhedron& link1_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(link1->GetID())->GetWorldPolyhedron();
  Vector3d joint1; //end of link1_poly
  for(int i=0; i<4; ++i)
    joint1 = joint1 + link1_poly.m_vertexList[i];
  joint1 = joint1 / 4;
  //cout << "\t\tjoint1 = " << joint1 << endl;

/*
  GMSPolyhedron& link2_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(link2->GetID())->GetWorldPolyhedron();
  Vector3d end_link2; //end of link2_poly
  for(int i=0; i<4; ++i)
    end_link2 = end_link2 + link2_poly.m_vertexList[i];
  end_link2 = end_link2 / 4;
  //cout << "\t\tend_link2 = " << end_link2 << endl;
*/




  vector<int> actual_ear_links;
  get_actual_links_of(ear_root_link, actual_ear_links);
  vector<int> actual_loop_links;
  get_actual_links_of(tree_root_link, actual_loop_links);

  int link3_id = actual_ear_links.back();
  GMSPolyhedron& link3_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(link3_id)->GetWorldPolyhedron();
  Vector3d end_link3; //end of link3_poly
  for(int i=0; i<4; ++i)
    end_link3 = end_link3 + link3_poly.m_vertexList[i];
  end_link3 = end_link3 / 4;
  //cout << "\t\tend_link3 = " << end_link3 << endl;

  int link4_id = -1;
  for(vector<int>::iterator L = actual_loop_links.begin(); L != actual_loop_links.end(); ++L)
    if(find(actual_ear_links.begin(), actual_ear_links.end(), *L) == actual_ear_links.end())
    {
      link4_id = *L;
      break;
    }
  if(link4_id == -1)
  {
    cerr << "Error in MyCalculateJointAngles: did not find link for ear connection\n";
    exit(-1);
  }
  //cout << "\t\tlink4_id = " << link4_id << endl;
  GMSPolyhedron& link4_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(link4_id)->GetWorldPolyhedron();
  Vector3d joint2; //start of link4_poly
  for(int i=4; i<8; ++i)
    joint2 = joint2 + link4_poly.m_vertexList[i];
  joint2 = joint2 / 4;
  //cout << "\t\tjoint2 = " << joint2 << endl;

  //cout << "\t\t\tear_length = " << (end_link3 - joint1).magnitude() << endl;

  //beta = angle between (joint2-joint1) and (end_link3 - joint1)
  Vector3d V1 = joint2 - joint1;
  V1.normalize();
  Vector3d V2 = end_link3 - joint1;
  V2.normalize();

  /*
  cout << "\nMyCalculateJointAngle(" << link1->GetID() << "," << link2->GetID() << ")" << endl;
  cout << "\tjoint1 = " << joint1 << endl;
  cout << "\tjoint2 = " << joint2 << endl;
  cout << "\tend_link3 = " << end_link3 << endl;
  */




  double beta = atan2(V2.getY(), V2.getX()) - atan2(V1.getY(), V1.getX());
  //cout << "\tbeta = " << beta << endl;

  //return angle + beta;
  angle += beta;
  while(angle < 0)
    angle += TWO_PI;
  while(angle > TWO_PI)
    angle -= TWO_PI;

  ConfigEar(env, ear_root_link, tree_root_link, angle);

  return angle;
}


bool ClosedChainProblem::ParseLinksFile(const char* linksFileName)
{
  ifstream fin(linksFileName, ios::in);
  if(!fin.good())
  {
    cerr << " Can not open " << linksFileName << endl;
    exit(1);
  }

  int numLinks = 0;

  char strData[256];
  fin >> strData;
  if(strcmp(strData, "numLinks") != 0)
    return false;
  else
    fin >> numLinks;

  for(int i=0; i<numLinks; ++i)
    g_baseLinks.push_back(NULL);

  while(!fin.eof())
  {
    strData[0] = '\0';
    fin >> strData;
    cout << "strData:" << strData << endl;

    if(strcmp(strData, "RealLink") == 0)
    {
      if(!ParseRealLink(fin))
        return false;
    }
    else if(strcmp(strData, "VirtualLink") == 0)
    {
      if(!ParseVirtualLink(fin))
        return false;
    }
    else if(strcmp(strData, "Loop") == 0)
    {
      if(!ParseLoop(fin))
        return false;
    }
    else if(strcmp(strData, "CfgJoints") == 0)
    {
      if(!ParseCfgJoints(fin))
        return false;
    }
    else if(strcmp(strData, "EarJoints") == 0)
    {
      if(!ParseEarJoints(fin))
        return false;
    }
  }

  fin.close();

  /*
  //look for implied constraints:
  cout << "Implied constraints found:\n";
  set<Link*> seen;
  for(size_t i=0; i<loopRoots.size(); ++i)
  {
    cout << "\tLoop " << i << ":";
    set<Link*> implied;
    seen_both_children(loopRoots[i], seen, implied);
    for(set<Link*>::iterator I = implied.begin(); I != implied.end(); ++I)
      cout << " " << (*I)->GetID();
    cout << endl;
  }
  */

  //find ears
  cout << "Partitioning loops into (unseen) (seen):\n";
  g_ears.clear();
  g_non_ears.clear();
  set<Link*> links_seen;
  for(size_t i=0; i<g_loopRoots.size(); ++i)
  {
    vector<int> seen, unseen;
    partition_loop(g_loopRoots[i], links_seen, seen, unseen);
    g_ears.push_back(unseen);
    g_non_ears.push_back(seen);
    cout << "\tLoop " << i << ": (";
    for_each(unseen.begin(), unseen.end(), cout << boost::lambda::_1 << " ");
    cout << ") (";
    for_each(seen.begin(), seen.end(), cout << boost::lambda::_1 << " ");
    cout << ")\n";
  }

  //find ear virtual link roots
  g_ear_roots.clear();
  vector<int> g_ear_rootIDs;
  for(size_t i=0; i<g_loopRoots.size(); ++i)
  {
    Link* parent = get_parent_of(g_loopRoots[i], g_ears[i]);
    if(parent == NULL)
    {
      cerr << "Error, parent link not found for ear " << i << "in loop " << i << endl;
      exit(-1);
    }
    else
    {
      g_ear_roots.push_back(parent);
      g_ear_rootIDs.push_back(parent->GetID());
    }
  }
  cout << "Ear root ids:\n";
  for(size_t i=0; i<g_ear_rootIDs.size(); ++i)
    cout << "\tEar " << i << ": " << g_ear_rootIDs[i] << endl;

  return true;
}

void ClosedChainProblem::PrintConfiguration(Environment* env, ostream & ofPath)
{
  ofPath << "0 0 0 0 0 0 ";
  for(size_t i=0; i<g_cfgJoints.size(); ++i)
    {
      double extAng = (PI - MyCalculateJointAngle(env, g_cfgJoints[i].first, g_cfgJoints[i].second)) / TWO_PI;
      while(extAng < 0)
	extAng += 1;
      while(extAng > 1)
	extAng -= 1;
      ofPath << extAng << " ";
    }
  ofPath << endl;
}


//void ClosedChainProblem::ConfigBase(Environment* env, const vector<double>& v = vector<double>(6, 0))
void ClosedChainProblem::ConfigBase(Environment* env, const vector<double>& v)
{
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, v[5]*TWOPI, v[4]*TWOPI, v[3]*TWOPI), Vector3d(v[0], v[1], v[2]));
  env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(0)->Configure(T1);
}





void ClosedChainProblem::ConfigureRobot(Environment* env)
{
  int robot = env->GetRobotIndex();

  //configure base
  //cout << "\tconfiguring base\n";
  /*
  Transformation T1 = Transformation(Orientation(Orientation::FixedXYZ, 0, 0, 0), Vector3d(0, 0, 0)); //fixed base
  env->GetMultiBody(robot)->GetFreeBody(0)->Configure(T1);
  */
  ConfigBase(env);

  //for each cfg joint, configure link corresponding to .second
  for(vector<pair<Link*, Link*> >::iterator J = g_cfgJoints.begin(); J != g_cfgJoints.end(); ++J)
    {
      double angle = (PI - MyCalculateJointAngle(env, J->first, J->second)) / TWO_PI;
      while(angle < 0)
	angle += 1;
      //cout << "\tconfiguring connection " << J->first->GetID() << " -> " << J->second->GetID() << " with theta " << angle <<       endl;
      //find appropriate backward connection...
      FreeBody* link2 = env->GetMultiBody(robot)->GetFreeBody(J->second->GetID()).get();
      bool found_connection = false;
      for(int i=0; i<link2->BackwardConnectionCount(); ++i) {
	FreeBody* link1 = env->GetMultiBody(robot)->GetFreeBody(J->first->GetID()).get();
	Connection conn = link2->GetBackwardConnection(i);
	boost::shared_ptr<Body> *boostPtr = new boost::shared_ptr<Body>(link1);
	if(conn.IsFirstBody(*boostPtr))
	  {
	    found_connection = true;
	    conn.GetDHparameters().theta = angle * 360.0;
	    break;
	  }
      }
      if(!found_connection)
	{
	  cerr << "Error in ConfigEnv: couldn't find connection " << J->first->GetID() << " -> " << J->second->GetID() << endl;
	  exit(-1);
	}
    }

  //cout << "\tcomputing world transformation\n";
  //env->GetMultiBody(robot)->GetFreeBody(0)->GetWorldTransformation();
  set<int> transformed;
  for(int i=0; i<env->GetMultiBody(robot)->GetFreeBodyCount(); ++i)
    {
      //cout << "Calling ComputeWorldTransformation for FreeBody " << i << endl;
      env->GetMultiBody(robot)->GetFreeBody(i)->ComputeWorldTransformation(transformed);
      //cout << "done\n";
    }
}


void ClosedChainProblem::ConfigEar(Environment* env, Link* ear_root, vector<int>& actual_ear_links, int base_link_id, double base_link_angle)
{
  int robot = env->GetRobotIndex();

  if(find(actual_ear_links.begin(), actual_ear_links.end(), base_link_id) != actual_ear_links.end())
    base_link_id = -1;

  /*
  cout << "\n\nDEBUG::ConfigEar:\n";
  cout << "\tear_root = " << ear_root->GetID() << endl;
  cout << "\tactual_ear_links = "; for_each(actual_ear_links.begin(), actual_ear_links.end(), cout << boost::lambda::_1 << "

"); cout << endl;
  cout << "\tbase_link_id = " << base_link_id << endl;
  cout << "\tbase_link_angle = " << base_link_angle << endl;
  cout << endl;
  */
      vector<double> angles;
  //for each loop joint, configure link
  for(vector<pair<Link*, Link*> >::iterator J = g_cfgJoints.begin(); J != g_cfgJoints.end(); ++J)
    {

      if((J->first->GetID() == base_link_id || find(actual_ear_links.begin(), actual_ear_links.end(), J->first->GetID()) !=

	  actual_ear_links.end()) &&
	 (J->second->GetID() == base_link_id || find(actual_ear_links.begin(), actual_ear_links.end(), J->second->GetID()) !=

	  actual_ear_links.end()))
	{
	  //cout << "getting angle for " << J->first->GetID() << " -> " << J->second->GetID() << endl;

	  double angle = 0;
	  if(J->first->GetID() == base_link_id || J->second->GetID() == base_link_id)
	    angle = (PI - base_link_angle) / TWO_PI;
	  else
	    angle = (PI - MyCalculateJointAngle(env, J->first, J->second)) / TWO_PI;
	  while(angle < 0)
	    angle += 1;

	  //find appropriate backward connection...
	  FreeBody* link2 = env->GetMultiBody(robot)->GetFreeBody(J->second->GetID()).get();
	  bool found_connection = false;
	  for(int i=0; i<link2->BackwardConnectionCount(); ++i) {
	    FreeBody* link1 = env->GetMultiBody(robot)->GetFreeBody(J->first->GetID()).get();
	    Connection conn = link2->GetBackwardConnection(i);
	    boost::shared_ptr<Body> *boostPtr = new boost::shared_ptr<Body>(link1);
	    if(conn.IsFirstBody(*boostPtr))
	      {
		found_connection = true;
		conn.GetDHparameters().theta = angle * 360.0;
		link2->GetBackwardConnection(i).GetDHparameters().theta = angle * 360.0;
		//cout << "setting connection theta of link " << J->first->GetID() << endl;
		break;
	      }
	  }
	  if(!found_connection)
	    {
	      cout<< "Error in ConfigEar: couldn't find connection "<<endl;
	      cerr << "Error in ConfigEar: couldn't find connection " << J->first->GetID() << " -> " << J->second->GetID() << endl;
	      exit(-1);
	    }
	}
    }
  set<int> transformed;
  //mark base
  transformed.insert(0);
  //mark all links in prior ears
  for(vector<Link*>::iterator E = g_ear_roots.begin(); E != g_ear_roots.end() && (*E) != ear_root; ++E)
    {
      vector<int> _actual_links;
      get_actual_links_of(*E, _actual_links);
      transformed.insert(_actual_links.begin(), _actual_links.end());
    }
  //transformed.insert(base_link_id);
  //cout << "links marked as transformed before configuring: "; for_each(transformed.begin(), transformed.end(), cout <<  boost::lambda::_1 << " "); cout << endl;
for(int i=0; i<env->GetMultiBody(robot)->GetFreeBodyCount(); ++i)
  if(find(actual_ear_links.begin(), actual_ear_links.end(), i) != actual_ear_links.end())
    {
      //cout << "calling ComputeWorldTransformation for link " << i << endl;
      env->GetMultiBody(robot)->GetFreeBody(i)->ComputeWorldTransformation(transformed);
    }
//cout<<"printing after get ear"<<endl;
 //env->GetMultiBody(robot)->Write(cout);
//cout << "DEBUG::Exiting ConfigEar\n\n";
}


void ClosedChainProblem::ConfigEar(Environment* env, Link* ear_root, Link* loop_root)
{
  vector<int> actual_ear_links;
  get_actual_links_of(ear_root, actual_ear_links);

  vector<int> actual_loop_links;
  get_actual_links_of(loop_root, actual_loop_links);

  /*
  vector<int> actual_non_ear_links;
  for(vector<int>::iterator L = actual_loop_links.begin(); L != actual_loop_links.end(); ++L)
    if(find(actual_ear_links.begin(), actual_ear_links.end(), *L) == actual_ear_links.end())
      actual_non_ear_links.push_back(*L);
  */

  //cout << "actual_ear_links: "; for_each(actual_ear_links.begin(), actual_ear_links.end(), cout << boost::lambda::_1 << " "); cout << endl;
  //cout << "actual_loop_links: "; for_each(actual_loop_links.begin(), actual_loop_links.end(), cout << boost::lambda::_1 << " "); cout << endl;
  //cout << "actual_non_ear_links: "; for_each(actual_non_ear_links.begin(), actual_non_ear_links.end(), cout << boost::lambda::_1 << " "); cout << endl;

  /*

 int base_link_id;
  double base_link_angle;
  if(actual_non_ear_links.empty())
  {
    cout << "\tloop and ear are the same\n";
    base_link_id = actual_ear_links.front();
    base_link_angle = PI;
  }
  else
  {
    cout << "\tloop and ear different\n";
    base_link_id = actual_loop_links.back(); //or actual_non_ear_links.back()
    base_link_angle = MyCalculateJointAngle(env, g_baseLinks[base_link_id], g_baseLinks[actual_ear_links.front()]);
  }
  */
  int base_link_id = actual_loop_links.back(); //or actual_non_ear_links.back()
  double base_link_angle = MyCalculateJointAngle(env, g_baseLinks[base_link_id], g_baseLinks[actual_ear_links.front()]);

  //cout << "in ConfigEar: ear_root = " << ear_root->GetID() << ", loop_root = " << loop_root->GetID() << ", base_link = " << base_link_id << endl;
  //cout << "\tfound base_link_angle: " << base_link_angle << endl;
  ConfigEar(env, ear_root, actual_ear_links, base_link_id, base_link_angle);
}


void ClosedChainProblem::ConfigEar(Environment*env, Link* ear_root, Link* loop_root, double base_link_angle)
{
  vector<int> actual_loop_links;
  get_actual_links_of(loop_root, actual_loop_links);

  vector<int> actual_ear_links;
  get_actual_links_of(ear_root, actual_ear_links);

  int base_link_id = actual_ear_links.front();
  if(!actual_loop_links.empty())
    base_link_id = actual_loop_links.back();

  ConfigEar(env, ear_root, actual_ear_links, base_link_id, base_link_angle);
}



void ClosedChainProblem::PrintEarJointCoords(Environment* env, Link* ear_root)
{
  vector<int> actual_links;
  get_actual_links_of(ear_root, actual_links);
  for(vector<int>::iterator L = actual_links.begin(); L != actual_links.end(); ++L)
  {
    GMSPolyhedron& link_poly = env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(*L)->GetWorldPolyhedron();

    Vector3d endpoint2;
    for(int i=0; i<4; ++i)
      endpoint2 = endpoint2 + link_poly.m_vertexList[i];
    endpoint2 = endpoint2 / 4;

    Vector3d endpoint1;
    for(int i=4; i<8; ++i)
      endpoint1 = endpoint1 + link_poly.m_vertexList[i];
    endpoint1 = endpoint1 / 4;

    cout << "\tendpoints of link " << *L << ":\t" << endpoint1 << "\t" << endpoint2 << endl;
  }
}






#endif
