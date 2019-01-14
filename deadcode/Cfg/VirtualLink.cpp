#include "math.h"
#include <iostream>
#include "VirtualLink.h"
#include "MPUtils.h"

// #define XINYU_DEBUG 1

ostream& operator<<(ostream &o, const Range& r)
{
  o << "(" << r.min << ", " << r.max << ")";
  return o;
}


int Link::IDCount = 0;

Link::Link(double rmin, double rmax)
{
  ID = IDCount++;
  InitializeFromRange(rmin, rmax);
}

Link::Link(int myID, double rmin, double rmax)
{
  if(IDCount <= myID)
    IDCount = myID + 1;
  ID = myID;
  InitializeFromRange(rmin, rmax);
}

Link::Link(Link* c0, Link* c1)
{
  ID = IDCount++;
  InitializeFromChildren(c0, c1);
}

Link::Link(int myID, Link * c0, Link *c1)
{
  if(IDCount <= myID)
    IDCount = myID + 1;
  ID = myID;
  InitializeFromChildren(c0, c1);
}

Link::~Link() {}

void Link::InitializeFromRange(double rmin, double rmax)
{
  length = -1;
  convexity = 1;
  coord = pair<double, double>(0,0);

  leftChild = NULL;
  rightChild = NULL;

  reachableRange.min = rmin;
  reachableRange.max = rmax;
  availableRange = reachableRange;
}

void Link::InitializeFromChildren(Link *c0, Link *c1)
{
  length = -1;
  convexity = 1;
  coord = pair<double,double>(-1, -1);

  leftChild = c0;
  c0->families.push_back(FamilyInfo(this, c1));

  rightChild = c1;
  c1->families.push_back(FamilyInfo(this, c0));

  reachableRange = RangeUnion(c0->reachableRange, c1->reachableRange);
  availableRange = reachableRange;
}

void Link::UpdateARange(Link *parent, Link *sibling, bool bSampleConvex)
{
  Range toReachRange;
  if(!bSampleConvex && parent->convexity == 0)
  { //it's enforced to be a flat configuration
    toReachRange = RangeMinus(parent->availableRange, sibling->availableRange);
  }
  else
  {
    toReachRange = RangeUnion(parent->availableRange, sibling->availableRange);
  }
  availableRange = RangeIntersection(availableRange, toReachRange);

#ifdef XINYU_DEBUG
  cout << "parent->availableRange:" << parent->availableRange << endl;
  cout << "sibling->availableRange:" << sibling->availableRange << endl;
  cout << "toReachRange:" << toReachRange << endl;
  cout << "availableRange:" << availableRange << endl;
#endif
}

double Link::SampleLength(bool bSampleConvex, double gama)
{
  //Randomly select a length within availableRange;
  if(!families.empty())
    for(vector<FamilyInfo>::iterator itrF = families.begin(); itrF != families.end(); itrF++)
      UpdateARange(itrF->parent, itrF->sibling, bSampleConvex);
  if(availableRange.Size() < -EPS_ZERO)
  {
    cerr << "available Range is empty! " << endl;
    cerr << "ID:" << ID << endl;
    cerr << "[" << availableRange.min << "," << availableRange.max << "]" << endl;
    return -1;
  }

  SetLength(availableRange.min + (availableRange.max - availableRange.min)*DRand());
#ifdef XINYU_DEBUG
  cout << "length:" << length << endl;
#endif

  if (bSampleConvex)
  {
    //Later on convexity will be added to familyInfo (actually LoopInfo)
    //and the convexity not only depends on links, but also depends on which loop we are looking at
    double dSample = DRand(); //todoInfo
    convexity = dSample < gama ? 1: -1; //todoInfo
  }

  return length;
}

double Link::SampleLength(boost::variate_generator<boost::rand48&,
    boost::uniform_real<>
    >& rand)
{
  for(vector<FamilyInfo>::iterator itrF = families.begin(); itrF != families.end(); itrF++)
  {
    if(itrF->parent != NULL)
      UpdateARange(itrF->parent, itrF->sibling);
    if(availableRange.Size() < -EPS_ZERO)
    {
      cerr << "available Range is empty! " << endl;
      return -1;
    }
  }

  SetLength(availableRange.min + (availableRange.max - availableRange.min)*rand());

  return length;
}

void Link::ExportTreeLinkLength(vector<double> &lengths)
{
  lengths.push_back(length);

  if(leftChild)
    leftChild->ExportTreeLinkLength(lengths);

  if(rightChild)
    rightChild->ExportTreeLinkLength(lengths);
}

int Link::ImportTreeLinkLength(const vector<double> &lengths, int index)
{
  length = lengths[index++];

  if(leftChild)
    index = leftChild->ImportTreeLinkLength(lengths, index);

  if(rightChild)
    index = rightChild->ImportTreeLinkLength(lengths, index);

  return index;
}

void Link::ExportTreeLinkReachableRange(vector<Range>& ranges) const {
  ranges.push_back(reachableRange);

  if(leftChild)
    leftChild->ExportTreeLinkReachableRange(ranges);
  if(rightChild)
    rightChild->ExportTreeLinkReachableRange(ranges);
}

void Link::ExportTreeLinkAvailableRange(vector<Range> &ranges)
{
  ranges.push_back(availableRange);

  if(leftChild)
    leftChild->ExportTreeLinkAvailableRange(ranges);

  if(rightChild)
    rightChild->ExportTreeLinkAvailableRange(ranges);
}

int Link::ImportTreeLinkAvailableRange(const vector<Range> &ranges, int index)
{
  availableRange = ranges[index++];

  if(leftChild)
    index = leftChild->ImportTreeLinkAvailableRange(ranges, index);

  if(rightChild)
    index = rightChild->ImportTreeLinkAvailableRange(ranges, index);

  return index;
}

void Link::ExportTreeLinkConvexity(vector<int> &convexities)
{
  convexities.push_back(convexity);

  if(leftChild)
    leftChild->ExportTreeLinkConvexity(convexities);

  if(rightChild)
    rightChild->ExportTreeLinkConvexity(convexities);
}

  int
Link::ImportTreeLinkConvexity(const vector<int> &convexities, int index)
{
  convexity = convexities[index++];

  if(leftChild)
    index = leftChild->ImportTreeLinkConvexity(convexities, index);

  if(rightChild)
    index = rightChild->ImportTreeLinkConvexity(convexities, index);

  return index;
}

void Link::ExportTreeLinkLength(vector<double> &lengths, vector<int> &convexities)
{
  lengths.push_back(length);
  convexities.push_back(convexity);//todoInfo

  if(leftChild)
    leftChild->ExportTreeLinkLength(lengths, convexities);

  if(rightChild)
    rightChild->ExportTreeLinkLength(lengths, convexities);
}

int Link::ImportTreeLinkLength(const vector<double> &lengths, const vector<int> &convexities, int index)
{
  availableRange.min = availableRange.max = length = lengths[index];
  convexity = convexities[index++];

  if(leftChild)
    index = leftChild->ImportTreeLinkLength(lengths, convexities, index);

  if(rightChild)
    index = rightChild->ImportTreeLinkLength(lengths, convexities, index);

  return index;
}

//Link*
void Link::FindRootPathways(Link *leaf, vector<Link*> &curPathway, vector<vector<Link*> > &pathwayEnsemble)
{
  curPathway.push_back(leaf);

  if(leaf->families.empty()) // the root
    pathwayEnsemble.push_back(curPathway);
  else
    for(size_t i=0; i<leaf->families.size(); ++i)
      FindRootPathways(leaf->families[i].parent, curPathway, pathwayEnsemble);

  curPathway.pop_back();
}

bool Link::MatchPathways(const vector<vector<Link *> > &pathEnsembleLeft, const vector<vector<Link *> > &pathEnsembleRight,
    vector<Link *> &leftPath, vector<Link *> &rightPath)
{
  for(size_t i=0; i<pathEnsembleLeft.size(); ++i)
    for(size_t j=0; j<pathEnsembleRight.size(); ++j)
      if(pathEnsembleLeft[i].back() == pathEnsembleRight[j].back())
      {
        leftPath = pathEnsembleLeft[i];
        rightPath = pathEnsembleRight[j];
        return true;
      }
  return false;
}

double Link::CalculateJointAngle(Link *left, Link *right)
{
  vector<Link*> leftAncestors, rightAncestors;
  {
    vector<vector<Link*> > pathEnsembleLeft;
    vector<Link*> curPath;
    FindRootPathways(left, curPath, pathEnsembleLeft);

    vector<vector<Link*> > pathEnsembleRight;
    curPath.clear();
    FindRootPathways(right, curPath, pathEnsembleRight);

    if(!MatchPathways(pathEnsembleLeft, pathEnsembleRight, leftAncestors, rightAncestors))
    {
      cerr << "Cannot match pathways which do not belong to the same family. " << endl;
      exit (1);
    }
  }
  Link *ancestor = NULL;
  while(!leftAncestors.empty() && !rightAncestors.empty() &&
      leftAncestors.back() == rightAncestors.back())
  {
    ancestor = leftAncestors.back();
    leftAncestors.pop_back();
    rightAncestors.pop_back();
  }

  double topJointAngle = CosineAngle(leftAncestors.back()->GetLength(), rightAncestors.back()->GetLength(), ancestor->GetLength());
  if (ancestor->leftChild == rightAncestors.back())
    topJointAngle *= -1;
  if (ancestor->convexity < 0)
    topJointAngle *= -1;

  double leftAngle = 0;
  while(leftAncestors.size() > 1)
  {
    Link *parent = leftAncestors.back();
    leftAncestors.pop_back();

    Link *me = leftAncestors.back();

    double myConvexity = parent->convexity;
    Link *sibling = parent->leftChild;
    if(me == parent->leftChild)
    {
      myConvexity *= -1;
      sibling = parent->rightChild;
    }

    leftAngle += myConvexity * CosineAngle(me->GetLength(), parent->GetLength(), sibling->GetLength());
  }

  double rightAngle = 0;
  while(rightAncestors.size() > 1)
  {
    Link *parent = rightAncestors.back();
    rightAncestors.pop_back();

    Link *me = rightAncestors.back();

    double myConvexity = parent->convexity;
    Link *sibling = parent->rightChild;
    if(me == parent->rightChild)
    {
      myConvexity *= -1;
      sibling = parent->leftChild;
    }

    rightAngle += myConvexity * CosineAngle(me->GetLength(), parent->GetLength(), sibling->GetLength());
  }

  return topJointAngle + leftAngle + rightAngle;
}

bool Link::RecursiveSample(double l, bool bSampleConvex, double gama)
{
  if(l < -EPS_ZERO)
  {
    if(SampleLength(bSampleConvex, gama) < 0)
      return false;
  }
  else
  {
    if(!SetLength(l))
    {
      cerr << "Could not close for length " << l  << endl;
      return false;
    }
  }
  if(leftChild != NULL && rightChild != NULL)
  {
    if(!leftChild->RecursiveSample(-1, bSampleConvex, gama))
      return false;
    if(!rightChild->RecursiveSample(-1, bSampleConvex, gama))
      return false;
  }
  return true;
}

bool Link::RecursiveSample(boost::variate_generator<boost::rand48&,
    boost::uniform_real<>
    >& rand,
    double l)
{
  if(l < -EPS_ZERO)
  {
    if(SampleLength(rand) < 0)
      return false;
  }
  else
  {
    if(!SetLength(l))
    {
      cerr << "Could not close for length " << l  << endl;
      return false;
    }
  }
  if(leftChild != NULL && rightChild != NULL)
  {
    if(!leftChild->RecursiveSample(rand, -1))
      return false;
    if(!rightChild->RecursiveSample(rand, -1))
      return false;
  }
  return true;
}

inline bool Link::SetLength(double l)
{
  if((l >= availableRange.min - EPS_ZERO) && (l <= availableRange.max + EPS_ZERO))
  {
    length = availableRange.min = availableRange.max = l;
    return true;
  }
  else
  {
    cerr << " Can't set length " << l << endl;
    cerr << "available range:" << availableRange << endl;
    cerr << "reachable range:" << reachableRange << endl;
    return false;
  }
}

void Link::PrintLink(ostream& os)
{
  os << "(" << ID;
  if(leftChild != NULL)
    os << "(" << leftChild->ID << "," << rightChild->ID << ")";
  os <<")" << endl;
}

void Link::PrintTree(ostream& os)
{
  PrintLink(os);

  if(leftChild != NULL)
    leftChild->PrintTree(os);
  if(rightChild != NULL)
    rightChild->PrintTree(os);
}

void Link::ResetTree()
{
  availableRange = reachableRange;
  convexity = 1;

  if(leftChild)
    leftChild->ResetTree();
  if(rightChild)
    rightChild->ResetTree();
}

bool Link::CanClose()
{
  if(!leftChild && !rightChild)
    return true;
  Range r = RangeUnion(leftChild->availableRange, rightChild->availableRange);
  if(availableRange.HasIntersection(r))
    return true;
  else
    return false;
}

bool Link::CanRecursiveClose()
{
  if(!leftChild && !rightChild)
    return true;
  if(CanClose() && leftChild->CanRecursiveClose() && rightChild->CanRecursiveClose())
    return true;
  else
    return false;
}

void Link::FindBreachesRecursive(list<Link*> &breaches)
{
  if(!leftChild && !rightChild)
    return;
  if(!CanClose())
    breaches.push_back(this);
  leftChild->FindBreachesRecursive(breaches);
  rightChild->FindBreachesRecursive(breaches);
}

  void
Link::InterpolateLinkLength(const vector<double> &start,
    const vector<double> &increment,
    vector<double> &intermediate,
    int index)
{
  intermediate.resize(start.size());
  for(size_t i=0; i<start.size(); ++i)
    intermediate[i] = start[i] + increment[i]*index;
}

  void
Link::InterpolateLinkLength(const vector<double> &start,
    const vector<double> &goal,
    vector<double> &intermediate,
    int nSteps,
    int index)
{
  intermediate.resize(start.size());
  for(size_t i=0; i<start.size(); ++i)
    intermediate[i] = start[i] + (goal[i]-start[i])*index/nSteps;
}

  void
Link::FindIncrement(const vector<double> &start,
    const vector<double> &goal,
    vector<double> &increment,
    int nSteps)
{
  increment.resize(start.size());
  for(size_t i=0; i<start.size(); ++i)
  {
    if(i<6)
      increment[i] = (goal[i] - start[i]) / nSteps;
    else if(i>=start.size()-CfgType::GetNumOfJoints()){

      double a = start[i];
      double b = goal[i];

      a = a - floor(a);
      b = b - floor(b);

      if(a>=0.5)a-=1.0;
      if(b>=0.5)b-=1.0;
      increment[i]=((b-a)/nSteps);
    }


  }
}

void Link::RecursiveBuildAvailableRange(bool bFlat)
{
  if(leftChild == NULL && rightChild == NULL)
  {
    availableRange = reachableRange;
    return;
  }

  leftChild->RecursiveBuildAvailableRange(bFlat);
  rightChild->RecursiveBuildAvailableRange(bFlat);

  if(bFlat)
  {
    if(convexity == 0)
    {
      Range sumRange = RangePlus(leftChild->availableRange, rightChild->availableRange);
      availableRange  = RangeIntersection(reachableRange, sumRange);
    }
    else
    {
      Range unionRange = RangeUnion(leftChild->availableRange, rightChild->availableRange);
      availableRange = RangeIntersection(reachableRange, unionRange);
    }
  }
  else
  {
    availableRange = reachableRange;
  }
}

Link *BuildTree(int i, int j, vector<Link *>& baseLinks)
{
  Link *root;
  int numLinks = j-i+1;

  if(numLinks >=2)
  {
    int numLeftLinks = numLinks/2;
    Link *leftChild = BuildTree(i, i+numLeftLinks-1, baseLinks);
    Link *rightChild = BuildTree(i+numLeftLinks, j, baseLinks);
    root = new Link(leftChild, rightChild);
  }
  else if (numLinks == 1)
  {
    root = baseLinks[i];
  }
  else
  {
    root = NULL;
  }

  return root;
}

double CosineAngle(double a, double b, double c)
{
  //cout << " a: " << a << " b:" << b << " c:" << c << endl;//debuginfo
  double retAngle;
  if((fabs(a) < EPS_ZERO) || (fabs(b) < EPS_ZERO))
    retAngle = PI/2;
  else
    retAngle = acos((a*a+b*b-c*c)/(2*a*b));
  return retAngle;
}

Range RangeIntersection(const Range &r1, const Range &r2)
{
  Range r3;
  r3.min = r1.min > r2.min? r1.min: r2.min;
  r3.max = r1.max < r2.max? r1.max: r2.max;
  return r3;
}

Range RangeUnion(const Range &r1, const Range &r2)
{
  Range r3;
  r3.max = r1.max + r2.max;
  if(r2.max < r1.min)
  {
    r3.min = r1.min - r2.max;
  }
  else if (r1.max < r2.min)
  {
    r3.min = r2.min - r1.max;
  }
  else
  {
    r3.min = 0;
  }
  return r3;
}

Range RangeMinus(const Range &r1, const Range &r2)
{
  Range r3;
  double l1 = r1.min - r2.max;
  double l2 = r1.max - r2.min;
  r3.min = l1;
  r3.max = l2;
  return r3;
}

Range RangePlus(const Range &r1, const Range &r2)
{
  Range r3;
  r3. max = r1.max + r2.max;
  r3. min = r1.min + r2.min;
  return r3;
}

void get_actual_links_of(Link* link, vector<int>& ids)
{
  if(link->leftChild != NULL)
    get_actual_links_of(link->leftChild, ids);

  if(link->rightChild != NULL)
    get_actual_links_of(link->rightChild, ids);

  if(link->leftChild == NULL && link->rightChild == NULL)
    ids.push_back(link->GetID());
}

  Link*
get_parent_of(Link* link, const vector<int>& ear)
{
  vector<int> actual_links;
  get_actual_links_of(link, actual_links);

  if(actual_links.size() < ear.size())
    return NULL;

  sort(actual_links.begin(), actual_links.end());
  vector<int> ear_sorted = ear; sort(ear_sorted.begin(), ear_sorted.end());

  if(actual_links == ear_sorted)
    return link;

  Link* left_ear = get_parent_of(link->leftChild, ear);
  if(left_ear != NULL)
    return left_ear;

  Link* right_ear = get_parent_of(link->rightChild, ear);
  if(right_ear != NULL)
    return right_ear;

  return NULL;
}

  void
seen_both_children(Link* link, set<Link*>& seen, set<Link*>& implied)
{
  if(link->leftChild == NULL || link->rightChild == NULL)
    return;

  if(find(seen.begin(), seen.end(), link->leftChild) != seen.end() &&
      find(seen.begin(), seen.end(), link->rightChild) != seen.end())
    implied.insert(link);

  seen.insert(link->leftChild);
  seen_both_children(link->leftChild, seen, implied);

  seen.insert(link->rightChild);
  seen_both_children(link->rightChild, seen, implied);
}

  void
partition_loop(Link* link, set<Link*>& links_seen, vector<int>& seen, vector<int>& unseen)
{
  if(link->leftChild == NULL && link->rightChild == NULL) //actual_link
  {
    if(find(links_seen.begin(), links_seen.end(), link) != links_seen.end())
      seen.push_back(link->GetID());
    else
      unseen.push_back(link->GetID());

    links_seen.insert(link);
  }

  if(link->leftChild != NULL)
    partition_loop(link->leftChild, links_seen, seen, unseen);

  if(link->rightChild != NULL)
    partition_loop(link->rightChild, links_seen, seen, unseen);
}

