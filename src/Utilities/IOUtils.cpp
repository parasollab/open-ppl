#include "IOUtils.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
//  XML Wrapper
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

XMLNode::
XMLNode(const string& _filename, const string& _desiredNode) :
  m_childBuilt(false), m_accessed(false), m_filename(_filename) {
    m_doc = new TiXmlDocument(_filename);

    if(!m_doc->LoadFile())
      throw ParseException(
          Where(_filename, m_doc->ErrorRow(), m_doc->ErrorCol(), false),
          m_doc->ErrorDesc());

    m_node = m_doc->FirstChild(_desiredNode.c_str());

    if(!m_node)
      throw ParseException(_filename,
          "Unable to find XML node '" + _desiredNode + "'.");
  }

XMLNode::iterator
XMLNode::
begin() {
  BuildChildVector();
  return m_children.begin();
}

XMLNode::iterator
XMLNode::
end() {
  BuildChildVector();
  return m_children.end();
}

bool
XMLNode::
Read(const string& _name,
    bool _req,
    bool _default,
    const string& _desc) {
  m_accessed = true;
  m_reqAttributes.insert(_name);
  const char* attrVal =  m_node->ToElement()->Attribute(_name.c_str());
  string toReturn;
  if(attrVal == NULL) {
    if(_req == true)
      throw ParseException(Where(), AttrMissing(_name, _desc));
    else
      return _default;
  }
  else
    toReturn = attrVal;

  transform(toReturn.begin(), toReturn.end(), toReturn.begin(), ::toupper);
  if(toReturn == "TRUE")
    return true;
  else if(toReturn == "FALSE")
    return false;
  else
    throw ParseException(Where(), AttrWrongType(_name, _desc));
  return false;
}

string
XMLNode::
Read(const string& _name,
    bool _req,
    const string& _default,
    const string& _desc) {
  m_accessed = true;
  m_reqAttributes.insert(_name);
  const char* attrVal =  m_node->ToElement()->Attribute(_name.c_str());
  string toReturn;
  if(attrVal == NULL) {
    if(_req == true)
      throw ParseException(Where(), AttrMissing(_name, _desc));
    else
      toReturn = _default;
  }
  else
    toReturn = attrVal;
  return toReturn;
}

void
XMLNode::
WarnAll(bool _warningsAsErrors) {
  ComputeAccessed();
  bool anyWarnings = false;
  WarnAllRec(anyWarnings);
  if(anyWarnings && _warningsAsErrors)
    throw ParseException(m_filename, "Reported Warnings are errors.");
}

string
XMLNode::
Where() const {
  return Where(m_filename, m_node->Row(), m_node->Column());
}

void
XMLNode::
BuildChildVector() {
  if(!m_childBuilt) {
    TiXmlNode* child = m_node->FirstChild();
    while(child != NULL) {
      if(child->Type() == TiXmlNode::ELEMENT)
        m_children.push_back(XMLNode(child, m_filename, m_doc));
      else if(child->Type() != TiXmlNode::COMMENT)
        throw ParseException(Where(m_filename, child->Row(), child->Column()),
            "Invalid XML element.");
      child = child->NextSibling();
    }

    m_childBuilt = true;
  }
}

XMLNode::
XMLNode(TiXmlNode* _node, const string& _filename, TiXmlDocument* _doc) :
  m_node(_node), m_childBuilt(false), m_accessed(false),
  m_filename(_filename), m_doc(_doc) {
  }

string
XMLNode::
AttrWrongType(const string& _name, const string& _desc) const {
  ostringstream oss;
  oss << "Wrong attribute type requested on '" << _name << "'." << endl;
  oss << "\tAttribute description: " << _desc << ".";
  return oss.str();
}

string
XMLNode::
AttrMissing(const string& _name, const string& _desc) const {
  ostringstream oss;
  oss << "Missing required attribute '" << _name << "'." << endl;
  oss << "\tAttribute description: " << _desc << ".";
  return oss.str();
}

void
XMLNode::
ComputeAccessed() {
  for(auto& child : *this) {
    child.ComputeAccessed();
    m_accessed = m_accessed || child.m_accessed;
  }
}

void
XMLNode::
WarnAllRec(bool& _anyWarnings) {
  if(m_accessed) {
    for(auto& child : *this)
      child.WarnAllRec(_anyWarnings);
    if(WarnUnrequestedAttributes())
      _anyWarnings = true;
  }
  else {
    WarnUnknownNode();
    _anyWarnings = true;
  }
}

void
XMLNode::
WarnUnknownNode() {
  cerr << "*************************************************************" << endl;
  cerr << "XML Warning:: Unknown or Unrequested Node" << endl;
  cerr << "File:: " << m_filename << endl;
  cerr << "Node: " << Name() << endl;
  cerr << "Line: " << m_node->Row() << endl;
  cerr << "Col: " << m_node->Column() << endl;
  cerr << "*************************************************************" << endl;
}

bool
XMLNode::
WarnUnrequestedAttributes() {
  vector<string> unreqAttr;
  const TiXmlAttribute* attr = m_node->ToElement()->FirstAttribute();
  while(attr != NULL) {
    if(m_reqAttributes.count(attr->Name()) == 0)
      unreqAttr.push_back(attr->Name());
    attr = attr->Next();
  }
  if(unreqAttr.size() > 0) {
    cerr << "*************************************************************" << endl;
    cerr << "XML Warning:: Unrequested Attributes Exist" << endl;
    cerr << "File:: " << m_filename << endl;
    cerr << "Node: " << Name() << endl;
    cerr << "Line: " << m_node->Row() << endl;
    cerr << "Col: " << m_node->Column() << endl;
    cerr << "Unrequested Attributes::" << endl;
    for(auto& a : unreqAttr)
      cerr << "\t" << a << endl;
    cerr << "*************************************************************" << endl;
    return true;
  }
  return false;
}

string
XMLNode::
Where(const string& _f, int _l, int _c, bool _name) const {
  ostringstream oss;
  oss << "File: " << _f;
  if(_name)
    oss << "\n\tNode: " << Name();
  oss << "\n\tLine: " << _l;
  oss << "\n\tCol: " << _c;
  return oss.str();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
//  Vizmo Debug output
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

ofstream* vdo = NULL;

void VDInit(string _filename){
  if(_filename!=""){
    vdo = new ofstream(_filename.c_str());
  }
};

void VDClose(){
  if(vdo!=NULL){
    vdo->close();
    delete vdo;
    vdo = NULL;
  }
}

void VDComment(string _s){
  if(vdo!=NULL){
    (*vdo) << "Comment " << _s << endl;
  }
};

void VDClearAll(){
  if(vdo!=NULL){
    (*vdo) << "ClearAll " << endl;
  }
};

void VDClearLastTemp(){
  if(vdo!=NULL){
    (*vdo) << "ClearLastTemp " << endl;
  }
};

void VDClearComments(){
  if(vdo!=NULL){
    (*vdo) << "ClearComments " << endl;
  }
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
//  Reading Fields, used in Multibody.h
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//determine if a file exists or not
bool
FileExists(const string& _filename, bool _err) {
  ifstream ifs(_filename.c_str());
  if(!ifs.good()) {
    if(_err) cerr << "File (" << _filename << ") not found";
    return false;
  }
  return true;
}

//discard all commented lines util the next uncommented line is found
void
GoToNext(istream& _is) {
  string line;
  while(!_is.eof()) {
    char c;
    while(isspace(_is.peek()))
      _is.get(c);

    c = _is.peek();
    if(!IsCommentLine(c))
      return;
    else
      getline(_is, line);
  }
}

//GetPathName from given filename
string
GetPathName(const string& _filename) {
  size_t pos = _filename.rfind('/');
  return pos == string::npos ? "" : _filename.substr(0, pos+1);
}

//read the string using above ReadField and tranform it to upper case
string
ReadFieldString(istream& _is, const string& _where, const string& _error, bool _toUpper) {
  string s = ReadField<string>(_is, _where, _error);
  if(_toUpper)
    transform(s.begin(), s.end(), s.begin(), ::toupper);
  return s;
}

//optionally read a color from a comment line
/*Color4
  GetColorFromComment(istream& _is) {
  string line;
  while(!_is.eof()) {
  char c;
  while(isspace(_is.peek()))
  _is.get(c);

  c = _is.peek();
  if(!IsCommentLine(c))
  return Color4(0.5, 0.5, 0.5, 1);
  else{
  getline(_is, line);
//colors begin with VIZMO_COLOR
if(line[7] == 'C'){
size_t loc = line.find(" ");
string sub = line.substr(loc+1);
istringstream iss(sub);
float r, g, b;
if(!(iss >> r >> g >> b))
throw ParseException(WHERE, "Cannot parse MultiBody color");
return Color4(r, g, b, 1);
}
}
}
return Color4();
}
*/

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
// GetTags: split string based on delimiter.
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
vector<string> GetTags(string _stags, string _delim) {
  vector<string> tokens;
  int cutAt;
  while( (cutAt = _stags.find_first_of(_delim)) != (int)_stags.npos ) {
    if(cutAt > 0) {
      tokens.push_back(_stags.substr(0,cutAt));
    }
    _stags = _stags.substr(cutAt+1);
  }
  if(_stags.length() > 0){
    tokens.push_back(_stags);
  }
  return tokens;
}

