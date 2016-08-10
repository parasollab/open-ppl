#include "IOUtils.h"


XMLNode::
XMLNode(const string& _filename, const string& _desiredNode) :
    m_filename(_filename) {
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
  m_node(_node), m_filename(_filename), m_doc(_doc) {
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

CountingStreamBuffer::
CountingStreamBuffer(string _filename) :
  m_filename(_filename),
  m_fileStream(_filename),
  m_streamBuffer(m_fileStream.rdbuf()),
  m_line(1), m_prevLine(1),
  m_column(0), m_prevColumn(static_cast<size_t>(-1)),
  m_filePos(0) {
  }

string
CountingStreamBuffer::
Where() const {
  ostringstream oss;
  oss << "File: " << m_filename
    << "\n\tLine: " << m_line
    << "\n\tColumn: " << m_column;
  return oss.str();
}

streambuf::int_type
CountingStreamBuffer::
uflow() {
  int_type rc = m_streamBuffer->sbumpc();

  m_prevLine = m_line;
  if(traits_type::eq_int_type(rc, traits_type::to_int_type('\n'))) {
    ++m_line;
    m_prevColumn = m_column + 1;
    m_column = static_cast<size_t>(-1);
  }

  ++m_column;
  ++m_filePos;
  return rc;
}

streambuf::int_type
CountingStreamBuffer::
pbackfail(streambuf::int_type _c) {
  if(traits_type::eq_int_type(_c, traits_type::to_int_type('\n'))) {
    --m_line;
    m_prevLine = m_line;
    m_column = m_prevColumn;
    m_prevColumn = 0;
  }

  --m_column;
  --m_filePos;

  if(_c != traits_type::eof())
    return m_streamBuffer->sputbackc(traits_type::to_char_type(_c));
  else
    return m_streamBuffer->sungetc();
}

ios::pos_type
CountingStreamBuffer::
seekoff(ios::off_type _pos, ios_base::seekdir _dir, ios_base::openmode _mode) {
  if(_dir == ios_base::beg && _pos == static_cast<ios::off_type>(0)) {
    m_prevLine = 1;
    m_line = 1;
    m_column = 0;
    m_prevColumn = static_cast<size_t>(-1);
    m_filePos = 0;

    return m_streamBuffer->pubseekoff(_pos, _dir, _mode);
  }
  else
    return streambuf::seekoff(_pos, _dir, _mode);
}

ios::pos_type
CountingStreamBuffer::
seekpos(ios::pos_type _pos, ios_base::openmode _mode) {
  if(_pos == static_cast<ios::pos_type>(0)) {
    m_prevLine = 1;
    m_line = 1;
    m_column = 0;
    m_prevColumn = static_cast<size_t>(-1);
    m_filePos = 0;

    return m_streamBuffer->pubseekpos(_pos, _mode);
  }
  else
    return streambuf::seekpos(_pos, _mode);
}

bool
FileExists(const string& _filename) {
  ifstream ifs(_filename.c_str());
  if(!ifs.good())
    return false;
  return true;
}

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

string
GetPathName(const string& _filename) {
  size_t pos = _filename.rfind('/');
  return pos == string::npos ? "" : _filename.substr(0, pos+1);
}

string
ReadFieldString(istream& _is, CountingStreamBuffer& _cbs,
    const string& _desc, bool _toUpper) {
  string s = ReadField<string>(_is, _cbs, _desc);
  if(_toUpper)
    transform(s.begin(), s.end(), s.begin(), ::toupper);
  return s;
}

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

