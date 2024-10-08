#include "XMLNode.h"

#include <algorithm>
#include <functional>

#include "tinyxml2.h"

/*------------------------------ Construction --------------------------------*/

XMLNode::XMLNode(const std::string& _filename, const std::string& _desiredNode) : m_filename(_filename) {
    // Read the XML file into a tinyxml2 document.
    m_doc = std::make_shared<tinyxml2::XMLDocument>();
    tinyxml2::XMLError loadResult = m_doc->LoadFile(_filename.c_str());
    if(loadResult != tinyxml2::XML_SUCCESS)
        throw ParseException(
                Where(_filename, m_doc->ErrorLineNum(), 0),  // No direct support for column in TinyXML2.
                "TinyXML2 failed to load file: " + std::string(m_doc->ErrorStr()) +
                " Note that TinyXML2's error reporting is more detailed compared to its previous version, but it's still important to check the actual error.");

    FindNode(_desiredNode);
}

XMLNode::
XMLNode(const std::string& filename, std::shared_ptr<tinyxml2::XMLDocument> doc, const std::string& desiredNode) :
    m_filename(filename), m_doc(doc) {

    FindNode(desiredNode);
}

void
XMLNode::
FindNode(const std::string& _desiredNode) {
  // Define a DFS search to locate the desired node in the XML tree.
  std::function<tinyxml2::XMLNode*(tinyxml2::XMLNode* const)> findNode =
      [&findNode, &_desiredNode](tinyxml2::XMLNode* const _node) -> tinyxml2::XMLNode* {
        // Skip nodes that don't represnt elements.
        if(!_node->ToElement())
          return nullptr;

        // Check this node for the desired tag.
        if(_node->Value() == _desiredNode)
          return _node;

        // If this isn't it, check direct children.
        tinyxml2::XMLNode* child = _node->FirstChild();
        while(child) {
          tinyxml2::XMLNode* result = findNode(child);
          if(result)
            return result;
          child = child->NextSibling();
        }

        // If we're still here, the tag wasn't found in the subtree. Return null.
        return nullptr;
      };

  // Search the XML node tree for the node with the desired description.
  m_node = findNode(m_doc->RootElement());
  if(!m_node)
    throw ParseException(m_filename, "Unable to find XML node '" + _desiredNode +
        "'.");
}


XMLNode::
XMLNode(tinyxml2::XMLNode* _node, const std::string& _filename,
    std::shared_ptr<tinyxml2::XMLDocument> _doc) :
    m_node(_node), m_filename(_filename), m_doc(_doc) {
}

/*-------------------------------- Iteration ---------------------------------*/

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

/*--------------------------- Metadata Accessors -----------------------------*/

const std::string& XMLNode::Name() const {
    static std::string nameValue;
    nameValue = m_node->Value();
    return nameValue;
}


const std::string&
XMLNode::
Filename() const {
  return m_filename;
}


std::string
XMLNode::
GetPath() const {
  // Determine the file path.
  const size_t sl = m_filename.rfind("/");
  return m_filename.substr(0, sl == std::string::npos ? 0 : sl + 1);
}

/*--------------------------- Content Accessors ------------------------------*/

std::string
XMLNode::
GetText() const {
  return std::string(m_node->ToElement()->GetText());
}

/*----------------------------------------------------------------------------*/

bool
XMLNode::
Read(const std::string& _name, const bool _req, const bool _default,
    const std::string& _desc) {
  m_accessed = true;
  m_reqAttributes.insert(_name);
  const char* attrVal =  m_node->ToElement()->Attribute(_name.c_str());
  std::string toReturn;
  if(attrVal == NULL) {
    if(_req == true)
      throw ParseException(Where(), AttrMissing(_name, _desc));
    else
      return _default;
  }
  else
    toReturn = attrVal;

  std::transform(toReturn.begin(), toReturn.end(), toReturn.begin(), ::toupper);
  if(toReturn == "TRUE")
    return true;
  else if(toReturn == "FALSE")
    return false;
  else
    throw ParseException(Where(), AttrWrongType(_name, _desc));
  return false;
}


std::string
XMLNode::
Read(const std::string& _name, const bool _req, const char* _default,
    const std::string& _desc) {
  return Read(_name, _req, std::string(_default), _desc);
}


std::string
XMLNode::
Read(const std::string& _name, const bool _req, const std::string& _default,
    const std::string& _desc) {
  m_accessed = true;
  m_reqAttributes.insert(_name);
  const char* attrVal =  m_node->ToElement()->Attribute(_name.c_str());
  std::string toReturn;
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


template <>
size_t
XMLNode::
Read(const std::string& _name, const bool _req, const size_t& _default, const size_t& _min,
    const size_t& _max, const std::string& _desc) {
    m_accessed = true;
  m_reqAttributes.insert(_name);
  int toReturn;

  tinyxml2::XMLError qr = m_node->ToElement()->QueryAttribute(_name.c_str(), &toReturn);
  switch(qr) {
    case tinyxml2::XML_WRONG_ATTRIBUTE_TYPE:
      throw ParseException(Where(), AttrWrongType(_name, _desc));
      break;
    case tinyxml2::XML_NO_ATTRIBUTE:
      {
        if(_req)
          throw ParseException(Where(), AttrMissing(_name, _desc));
        else
          return _default;
        break;
      }
    case tinyxml2::XML_SUCCESS:
      {
        size_t stoReturn = size_t(toReturn);
        if(stoReturn < _min || stoReturn > _max)
          throw ParseException(Where(),
              AttrInvalidBounds(_name, _desc, _min, _max, stoReturn));
        break;
      }
    default:
      throw RunTimeException(WHERE, "Logic shouldn't be able to reach this.");
  }

  return size_t(toReturn);
}


void
XMLNode::
Ignore() {
  m_accessed = true;

  // Set the list of requested attributes to those found in the node.
  m_reqAttributes.clear();
  for(const tinyxml2::XMLAttribute* attr = m_node->ToElement()->FirstAttribute();
      attr != nullptr; attr = attr->Next()) {
    m_reqAttributes.insert(attr->Name());
  }
}


void
XMLNode::
WarnAll(const bool _warningsAsErrors) {
  ComputeAccessed();
  bool anyWarnings = false;
  WarnAllRec(anyWarnings);
  if(anyWarnings && _warningsAsErrors)
    throw ParseException(m_filename, "Reported Warnings are errors.");
}


std::string
XMLNode::
Where() const {
  return Where(m_filename, m_node->GetLineNum(), 0);
}

/*--------------------------------- Helpers ----------------------------------*/

std::string
XMLNode::
Where(const std::string& _f, const int _l, const int _c, const bool _name) const {
  std::ostringstream oss;
  oss << "File: " << _f;
  if(_name)
    oss << "\n\tNode: " << Name();
  oss << "\n\tLine: " << _l
      << "\n\tCol: " << _c;
  return oss.str();
}


void XMLNode::BuildChildVector() {
    // Guard against building the child vector more than once.
    if(m_childBuilt)
        return;
    m_childBuilt = true;

    tinyxml2::XMLNode* child = m_node->FirstChild();
    while(child != NULL) {
        if(child->ToElement()) // Checks if it is an element.
            m_children.push_back(XMLNode(child, m_filename, m_doc));
        else if(!child->ToComment()) // Checks if it is NOT a comment.
            // TinyXML2 doesn't support Column(). So, it's set to 0 for now.
            throw ParseException(Where(m_filename, child->GetLineNum(), 0),"Invalid XML element.");
        child = child->NextSibling();
    }
}


std::string
XMLNode::
AttrWrongType(const std::string& _name, const std::string& _desc) const {
  std::ostringstream oss;
  oss << "Wrong attribute type requested on '" << _name << "'."
      << "\n\tAttribute description: " << _desc;
  return oss.str();
}


std::string
XMLNode::
AttrMissing(const std::string& _name, const std::string& _desc) const {
  std::ostringstream oss;
  oss << "Missing required attribute '" << _name << "'."
      << "\n\tAttribute description: " << _desc;
  return oss.str();
}


void
XMLNode::
ComputeAccessed() {
  // Count a node as accessed if any child was accessed.
  for(auto& child : *this) {
    child.ComputeAccessed();
    m_accessed = m_accessed || child.m_accessed;
  }

  // Count a node as accessed if it has no children and no attributes.
  if(this->begin() == this->end() && !m_node->ToElement()->FirstAttribute())
    m_accessed = true;
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
  std::cerr << "*************************************************************"
            << "\nXML Warning:: Unknown or Unrequested Node"
            << "\nFile:: " << m_filename
            << "\nNode: " << Name()
            << "\nLine: " << m_node->GetLineNum()
            << "\nIf you expected this to work, make sure:"
            << "\n\t- There is a default (no arguments) constructor."
            << "\n\t- No constructor is defined with = default."
            << "\n\t- You have called this->SetName with the XML node name in "
            << "every constructor."
            << "\n\t- You have added this method to the traits."
            << "\n*************************************************************"
            << std::endl;
}


bool
XMLNode::
WarnUnrequestedAttributes() {
  // Read all the attributes and check if they were requested.
  std::vector<std::string> unreqAttr;
  const tinyxml2::XMLAttribute* attr = m_node->ToElement()->FirstAttribute();
  while(attr != NULL) {
    if(m_reqAttributes.count(attr->Name()) == 0)
      unreqAttr.push_back(attr->Name());
    attr = attr->Next();
  }

  // If unrequested attributes were found, display a warning message.
  if(unreqAttr.size() > 0) {
    std::cerr << "***********************************************************"
              << "\nXML Warning:: Unrequested Attributes Exist"
              << "\nFile:: " << m_filename
              << "\nNode: " << Name()
              << "\nLine: " << m_node->GetLineNum()
              << "\nUnrequested Attributes::";
    for(const auto& a : unreqAttr)
      std::cerr << "\n\t" << a;
    std::cerr << "\n***********************************************************"
              << std::endl;
    return true;
  }
  return false;
}

/*----------------------------------------------------------------------------*/