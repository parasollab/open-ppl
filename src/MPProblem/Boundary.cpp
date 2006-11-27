#include "Boundary.h"
#include "MPProblem.h"
#include "Parameters.h"

Boundary::Boundary() {
/*     cout << "Boundary(). TODO ALL " << endl; */
}

Boundary::~Boundary() {
/*     cout << "~Boundary(). TODO ALL " << endl; */
}





BoundingBox::
BoundingBox(int i_dofs, int i_pos_dofs ) :
  dofs(i_dofs), 
  pos_dofs(i_pos_dofs) {
  bounding_box.clear();
  for (int i = 0; i < dofs; i++) {
    bounding_box.push_back(pair<double,double>(0.0,1.0));
    if (i < pos_dofs)
      par_type.push_back(TRANSLATIONAL);
    else
      par_type.push_back(REVOLUTE);
  }
}

BoundingBox::
BoundingBox(TiXmlNode* in_pNode,MPProblem* in_pproblem)
  : dofs(in_pproblem->GetDOFs()),   
    pos_dofs(in_pproblem->GetPosDOFs()) {
      LOG_DEBUG_MSG("BoundingBox::BoundingBox()");
  if(! (string(in_pNode->Value()) == "boundary")) {
    cout << "Error: I have not ben given <boundary>" << endl;
    exit(-1);
  }

  bounding_box.clear();
  for (int i = 0; i < dofs; i++) {
    bounding_box.push_back(pair<double,double>(0.0,1.0));
    if (i < pos_dofs)
      par_type.push_back(TRANSLATIONAL);
    else
      par_type.push_back(REVOLUTE);
  }

  for (TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
    if (string(pChild->Value()) == "parameter") {
     
      int par_id;
      pChild->ToElement()->QueryIntAttribute("id",&par_id);
      string par_label = string(pChild->ToElement()->Attribute("Label")); //@todo par_label is not used in bbox parameters, may want to use it
      double par_min, par_max;
      pChild->ToElement()->QueryDoubleAttribute("min",&par_min);
      pChild->ToElement()->QueryDoubleAttribute("max",&par_max);
      LOG_DEBUG_MSG("BoundingBox:: setting parameter par_id="<<par_id<<" par_min="
          <<par_min<<" par_max="<<par_max);
      
      SetParameter(par_id,par_min,par_max);
      if (string(pChild->ToElement()->Attribute("type")) == "translational")
	par_type[par_id] = TRANSLATIONAL;
      else
	par_type[par_id] = REVOLUTE;
    } else {
      if (!pChild->Type() == TiXmlNode::COMMENT) {
	cout << "  I don't know: " << *pChild << endl;
      }
    }
  }
  double translational_scale;
  in_pNode->ToElement()->QueryDoubleAttribute("translational_scale", &translational_scale);
  TranslationalScale(translational_scale);
  LOG_DEBUG_MSG("~BoundingBox::BoundingBox()");
}

BoundingBox::
BoundingBox(const BoundingBox &from_bbox)  {
  dofs = from_bbox.GetDOFs();
  pos_dofs = from_bbox.GetPosDOFs();
  bounding_box.clear();
  for (int i = 0; i < dofs; i++) {
    bounding_box.push_back(from_bbox.GetRange(i));
    par_type.push_back(from_bbox.GetType(i));
  }
}

BoundingBox::
~BoundingBox() {
  //cout << " ~BoundingBox(). TODO ALL " << endl;
}

unsigned int 
BoundingBox::
GetDOFs() const {
  return bounding_box.size();
}

unsigned int 
BoundingBox::
GetPosDOFs() const {
  return bounding_box.size();
}

const std::pair<double,double> 
BoundingBox::
GetRange(int par) const {
  return bounding_box[par];
}

double 
BoundingBox::
GetClearance(Vector3D point3d) const {
  double min_clearance = -1;
  double clearance = 0;
  for (int i = 0; i < pos_dofs; ++i) {
    clearance = min(abs((int)( point3d[i] - bounding_box[i].first)), 
			abs((int)( bounding_box[i].second - point3d[i])));
    if (clearance < min_clearance || i == 0)
      min_clearance = clearance;
  }
  return min_clearance;
}

BoundingBox::parameter_type 
BoundingBox::
GetType(int par) const {
  return par_type[par];
}

void 
BoundingBox::
SetParameter(int par, double p_first, double p_second) {
  bounding_box[par].first = p_first;
  bounding_box[par].second = p_second;
}

void
BoundingBox::
SetRanges(std::vector<double> &ranges) {
  std::vector<double>::iterator itr;
  unsigned int i = 0;
  for (itr = ranges.begin(); itr < ranges.end() && i < dofs; itr = itr+2, i++) {
    SetParameter(i,*itr,*(itr+1));
  }
}


std::vector<BoundingBox > 
BoundingBox::
Partition(int par, double p_point, double epsilon) {
  std::vector<BoundingBox > result;
  BoundingBox leftBB(*this);
  leftBB.SetParameter(par, (leftBB.GetRange(par)).first, p_point+epsilon);
  result.push_back(leftBB);
  BoundingBox rightBB(*this);
  rightBB.SetParameter(par, p_point-epsilon, rightBB.GetRange(par).second);
  result.push_back(rightBB);
  return result;
}

BoundingBox
BoundingBox::
GetCombination(BoundingBox &o_bounding_box) {
  BoundingBox combination(*this);
  for (int par=0; par < o_bounding_box.GetDOFs() && par < combination.GetDOFs(); par++) {
    combination.SetParameter(par,
			     min(combination.bounding_box[par].first,
				 o_bounding_box.bounding_box[par].first),
			     max(combination.bounding_box[par].second,
				 o_bounding_box.bounding_box[par].second));
  }
  return combination;
}

int
BoundingBox::
FindSplitParameter(BoundingBox &o_bounding_box) {
  int split_par = -1;
  for (int par=0; par < bounding_box.size(); par++) {
    if (bounding_box[par].first != o_bounding_box.bounding_box[par].first ||
	bounding_box[par].second != o_bounding_box.bounding_box[par].second) {
      if (split_par == -1) { // check to see that only one parameter varies
	split_par = par;
      } else { // a split par was previously found, not right.
	split_par = -1;
	break;
      }
    }
  }
  return split_par;
}

bool
BoundingBox::
IfWrap(int par) {
  if (par_type[par] == REVOLUTE) { // orientation angle
    if (bounding_box[par].first == 0 && bounding_box[par].second == 1)
      return true;
    if (bounding_box[par].first > bounding_box[par].second)
      return true;
  //may need to consider other cases of second < first ?
  }
  return false;
}

void
BoundingBox::
Print(std::ostream& _os, char range_sep, char par_sep) {
  std::vector< std::pair<double, double> >::iterator itrb;
  _os << "[ " ;
  for (itrb = bounding_box.begin(); itrb < bounding_box.end(); ++itrb) {
    if (itrb+1 != bounding_box.end())
      _os << itrb->first << range_sep << itrb->second << " " << par_sep << " ";
    else
      _os << itrb->first << range_sep << itrb->second << "";
  }
  _os << " ]";
}


void
BoundingBox::
Parse(std::stringstream &i_bbox) {
  vector<double> boundingBox;
  char c;
  bool done = false;

  try {
    do {
      c = i_bbox.peek();
      if ( c==' ' || c=='\n' || c==',' || c=='[' || c==']') {
	if (c==']')
	  done = true;
	if (!i_bbox.get())
	  done=true;
      } else if ((c>='0' && c<='9') || c=='-' || c=='.') {
	double dtmp;
	if (i_bbox >> dtmp)
	  boundingBox.push_back(dtmp);
	else {
	  cout << "Character not accepted" << endl;
	  throw BadUsage();
	}
      } else {
	cout << "Character not accepted" << endl;
	throw BadUsage();
      }
    } while (!done);
    if (boundingBox.size()%2 != 0) {
      cout << "Expecting even number of ranges" << endl;
      throw BadUsage();
    }
    if (boundingBox.size() < pos_dofs*2) {
      cout << "Insufficient number of ranges" << endl;
      throw BadUsage();
    } 
} catch (BadUsage) {
  cout << "BoundingBox:Parse: Err in BoundingBox parameters...expecting even number parameters...no letters...etc" << endl;
  exit(-1);
}
  SetRanges(boundingBox);
}

void
BoundingBox::
TranslationalScale(double scale_factor) {
  double center, new_first, new_second;
  if (scale_factor != 1.0) {    	
    for (int i = 0; i < pos_dofs; i++) {
      center = (bounding_box[i].first+bounding_box[i].second)/2;
      new_first = (bounding_box[i].first-center)*scale_factor+center;
      new_second = (bounding_box[i].second-center)*scale_factor+center;
      SetParameter(i,new_first,new_second);
    }
  }
}

double
BoundingBox::
GetRandomValueInParameter(int par) {
  double v;
  if (par < pos_dofs) {
    v = bounding_box[par].first + 
      (bounding_box[par].second - bounding_box[par].first)*OBPRM_drand();
  }
  else { // an orientation angle (not necessarily true for many robots)
    if( par > bounding_box.size() ) { //when par is not valid
      v = OBPRM_drand(); 
    }
    else { // want something in range of orientation angles
      if (bounding_box[par].first < bounding_box[par].second) { // regularly
	v = bounding_box[par].first + 
	  (bounding_box[par].second - bounding_box[par].first)*OBPRM_drand();
      }
      else { // check the other way, the angle loops around 1
	v = bounding_box[par].first + 
	  (1-(bounding_box[par].first - bounding_box[par].second))*OBPRM_drand();
	if (v > 1) 
	  v = v - 1;
	else if (v < 0) 
	  v = OBPRM_drand();
	if (v > 1) // only possible if first not in the range of 0-1, wrong
	  v = OBPRM_drand();
      }
    }      
  }
  return v;
}

bool 
BoundingBox::
IfEnoughRoom(int par, double room) {
  if (bounding_box[par].second > bounding_box[par].first) { // regularly
    if ((bounding_box[par].second - bounding_box[par].first) >= room)
      return true;
    else
      return false;
  } else { // only in orientation parameters that turn around
    if ((1-bounding_box[par].first - bounding_box[par].second) >= room)
      return true;
    else
      return false;
  }
}

bool
BoundingBox::
IfSatisfiesConstraints(Vector3D point3d) const {
  for (int i = 0; i < 3; i++) {
    if ( point3d[i] < bounding_box[i].first || point3d[i] > bounding_box[i].second)
      return false;
  }
  return true;
}

bool
BoundingBox::
IfSatisfiesConstraints(vector<double> point) const {
  for (int i = 0; i < point.size() && i < bounding_box.size(); i++) {
    if (par_type[i] == REVOLUTE) {
      if (point[i] < 0 || point[i] > 1) {
	cout << "Invalid range on REVOLUTE dof." << endl;
	exit(-1);
	return false;
      }
      if (bounding_box[i].first > bounding_box[i].second) { // wrap around parameter
	if (point[i] > bounding_box[i].second && point[i] < bounding_box[i].first)
	  return false;
      }
    } else { //no wrap around in other kinds of parameters
      if (point[i] < bounding_box[i].first || point[i] > bounding_box[i].second)
	return false;
      // may still need to consider physical constraints not explicitely set by the bounding box
    }
  }
  return true;
}

