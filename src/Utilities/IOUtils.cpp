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

XMLNodeReader::XMLNodeReader(TiXmlNode* _node, const string& _filename ):
  m_node(_node),m_childBuilt(false),m_xmlFilename(_filename) {
    if(_node == NULL) {
      cerr << "XMLNodeReader::XMLNodeReader() Invalid TiXMLNode pointer" << endl;
    }
  }

XMLNodeReader::XMLNodeReader(const string& _filename, TiXmlDocument& _doc,
    const string& _desiredNode) {

  bool loadOkay = _doc.LoadFile();

  if (!loadOkay) {
    cerr << "Could not load test file. Error=" << _doc.ErrorDesc() <<". Exiting.\n";
    exit(-1);
  }

  TiXmlNode* node = NULL;
  node = _doc.FirstChild(_desiredNode.c_str());
  if(node == NULL) {
    cerr << "parseInputXMLFile Unable to find Node: " << _desiredNode << endl;
  }
  m_node = node;
  m_xmlFilename= _filename;
  m_childBuilt = false;
}

string
XMLNodeReader::getName() const {
  if(m_node == NULL) {
    cerr << "XMLNodeReader::getName() -- error" << endl << flush;
    exit(-1);
  }
  return m_node->ValueStr();
}

bool
XMLNodeReader::hasChild(const string& _childName) const {
  for( TiXmlNode* child = m_node->FirstChild(); child !=0; child = child->NextSibling()) {
    if(child->Type() == TiXmlNode::ELEMENT) {
      if(child->ValueStr() == _childName) {
        return true;
      }
    }
  }
  return false;
}


XMLNodeReader
XMLNodeReader::getFirstChild(const string& _childName) {
  for( TiXmlNode* child = m_node->FirstChild(); child !=0; child = child->NextSibling()) {
    if(child->Type() == TiXmlNode::ELEMENT) {
      if(child->ValueStr() == _childName) {
        return XMLNodeReader(child,m_xmlFilename);
      }
    }
  }
  //return false;
  PrintMissingRequestedChild(_childName);
  exit(-1);
}


XMLNodeReader::childiterator
XMLNodeReader::children_begin() {
  BuildChildVector();
  return m_children.begin();
}

XMLNodeReader::childiterator
XMLNodeReader::children_end() {
  BuildChildVector();
  return m_children.end();
}



string
XMLNodeReader::stringXMLParameter(const string& _name,
    bool _req,
    const string& _default,
    const string& _desc) {
  VerifyElement();
  m_reqAttributes.push_back(_name);
  const char* attrVal =  m_node->ToElement()->Attribute(_name.c_str());
  string toReturn;
  if(attrVal == NULL) {
    if(_req == true) {
      PrintAttrMissing(_name, _desc);
      exit(-1);
    } else {
      toReturn = _default;
    }
  } else {
    toReturn = string(attrVal);
  }
  return toReturn;
}

bool
XMLNodeReader::hasXMLParameter(const string& _name) {
  const char* attrVal =  m_node->ToElement()->Attribute(_name.c_str());
  if(attrVal == NULL)
    return false;
  else
    return true;
}

bool
XMLNodeReader::boolXMLParameter(const string& _name, bool _req,
    bool _default, const string& _desc) {
  // like a string, then to lower, check for == "true" || "false"
  VerifyElement();
  m_reqAttributes.push_back(_name);
  const char* attrVal =  m_node->ToElement()->Attribute(_name.c_str());
  string toReturn;
  if(attrVal == NULL) {
    if(_req == true) {
      PrintAttrMissing(_name, _desc);
      exit(-1);
    } else {
      return _default;
    }
  } else {
    toReturn = string(attrVal);
  }

  transform(toReturn.begin(), toReturn.end(), toReturn.begin(), (int(*)(int)) toupper);
  if(toReturn == "TRUE") {
    return true;
  } else if(toReturn == "FALSE") {
    return false;
  } else {
    PrintAttrWrongType(_name,_desc);
  }
  return false;
}

void
XMLNodeReader::warnUnrequestedAttributes() {
  VerifyElement();
  vector<string>::iterator strItr;
  vector<string> unreqAttr;
  const TiXmlAttribute* attr = m_node->ToElement()->FirstAttribute();
  for(; attr !=NULL; attr = attr->Next()) {
    if(HasRequestedAttr(string(attr->Name())) == false)
      unreqAttr.push_back(string(attr->Name()));
  }
  if(unreqAttr.size() > 0) {
    cout << "*************************************************************" << endl;
    cout << "XML Warning:: Unrequested Attributes Exist" << endl;
    cout << "XML file:: " << m_xmlFilename << endl;
    cout << "Parent Node: " << getName() << endl;
    cout << "Unrequested Attributes::" << endl;
    for(strItr = unreqAttr.begin(); strItr != unreqAttr.end(); ++strItr)
      cout << "          " << *strItr << endl;
    cout << *m_node << endl;
    cout << "*************************************************************" << endl;
  }
}

void
XMLNodeReader::warnUnknownNode() {
  cout << "*************************************************************" << endl;
  cout << "XML Warning:: Unknown Node -- Ignoring" << endl;
  cout << "XML file:: " << m_xmlFilename << endl;
  cout << "Node Name: " << getName() << endl;
  cout << *m_node << endl;
  cout << "*************************************************************" << endl;
}


void
XMLNodeReader::verifyName(const string& _name) {
  if(getName() != _name) {
    cout << "*************************************************************" << endl;
    cout << "XML ERROR:: Node Name Mismatch" << endl;
    cout << "XML file:: " << m_xmlFilename << endl;
    cout << "Node Name: " << getName() << " Requested Name: " << _name << endl;
    cout << *m_node << endl;
    cout << "*************************************************************" << endl;
    exit(-1);
  }
}

void
XMLNodeReader::VerifyElement() const {
  if(!IsElement()) {
    cerr << "XMLNode::verifyElement() ERROR: Node is not an element." << endl;
    cerr << *m_node << endl;
    exit(-1);
  }
}

void
XMLNodeReader::PrintAttrWrongType(const string& _name, const string& _desc) const {
  cerr << "*************************************************************" << endl;
  cerr << "XML Error:: Wrong Type of Attribute requested" << endl;
  cerr << "XML file:: " << m_xmlFilename << endl;
  cerr << "Parent Node: " << getName() << ", Attribute Requested: ";
  cerr << _name << endl;
  cerr << "Attribute Description: " << _desc << endl;
  cerr << *m_node << endl;
  cerr << "*************************************************************" << endl;
}

void
XMLNodeReader::PrintAttrMissing(const string& _name, const string& _desc) const {
  cerr << "*************************************************************" << endl;
  cerr << "XML Error:: Missing required Attribute" << endl;
  cerr << "XML file:: " << m_xmlFilename << endl;
  cerr << "Parent Node: " << getName() << ", Attribute Missing: ";
  cerr << _name << endl;
  cerr << "Attribute Description: " << _desc << endl;
  cerr << *m_node << endl;
  cerr << "*************************************************************" << endl;
}

void
XMLNodeReader::PrintMissingRequestedChild(const string& _childName) {
  cerr << "*************************************************************" << endl;
  cerr << "XML Error:: Error Finding Child" << endl;
  cerr << "XML file:: " << m_xmlFilename << endl;
  cerr << "Parent Node: " << getName() << " Requested Child: " << _childName << endl;
  cerr << *m_node << endl;
  cerr << "*************************************************************" << endl;
}

void
XMLNodeReader::BuildChildVector() {
  if(!m_childBuilt) {
    for( TiXmlNode* child = m_node->FirstChild(); child !=NULL; child = child->NextSibling()) {
      if(child->Type() == TiXmlNode::ELEMENT) {
        m_children.push_back(XMLNodeReader(child,m_xmlFilename));
      }
    }
    m_childBuilt=true;
  }
}

bool
XMLNodeReader::HasRequestedAttr(string _name) {
  vector<string>::iterator strItr;
  for(strItr = m_reqAttributes.begin(); strItr != m_reqAttributes.end(); ++strItr) {
    if(*strItr == _name) {
      return true;
    }
  }
  return false;
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

