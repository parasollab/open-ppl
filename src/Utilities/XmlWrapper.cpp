///////////////////////////////////////////////////////////////////////////////
///\file XmlWrapper.cpp
/// This file provides a set of classes that wrap up
/// READONLY interaction with TinyXML.  
///
/// \author Roger Pearce
/// \date 2008.01.28
///////////////////////////////////////////////////////////////////////////////


#include "XmlWrapper.h"
#include <ostream>

XMLNodeReader::
XMLNodeReader(TiXmlNode* in_pnode, const std::string& in_filename ):
      m_pnode(in_pnode),m_bChildVecBuilt(false),m_xmlfilename(in_filename) { 
  if(in_pnode == NULL) {
    std::cerr << "XMLNodeReader::XMLNodeReader() Invalid TiXMLNode pointer" << std::endl;
  }
}

XMLNodeReader::
XMLNodeReader(const std::string& in_filename, TiXmlDocument& in_doc, 
                          const std::string& in_desiredNode) {
  
  //TiXmlDocument doc( in_fileName.c_str());
  bool loadOkay = in_doc.LoadFile();
  
  
  if ( !loadOkay ) {
    std::cerr << "Could not load test file. Error=" << in_doc.ErrorDesc() <<". Exiting.\n";
    exit( -1 );
  }

  TiXmlNode* in_pNode = NULL;
  in_pNode = in_doc.FirstChild( in_desiredNode.c_str() );
  if(in_pNode == NULL) {
    std::cerr << "parseInputXMLFile Unable to find Node: " << in_desiredNode << std::endl;
        
  }
  m_pnode = in_pNode;
  m_xmlfilename=in_filename;
  m_bChildVecBuilt = false;
}

std::string
XMLNodeReader::getName() const
{ 
  if(m_pnode == NULL) {
    std::cerr << "XMLNodeReader::getName() -- error" << std::endl << std::flush;
    exit(-1);
  }
  return m_pnode->ValueStr();
}
 
bool 
XMLNodeReader::
hasChild(const std::string& in_childName) const {
  for( TiXmlNode* pChild = m_pnode->FirstChild(); pChild !=0; pChild = pChild->NextSibling())
  {
    if(pChild->Type() == TiXmlNode::ELEMENT) {
      if(pChild->ValueStr() == in_childName) { 
        return true;
      }
    }
  }
  return false;
}


XMLNodeReader
XMLNodeReader::
getFirstChild(const std::string& in_childName) {
  for( TiXmlNode* pChild = m_pnode->FirstChild(); 
      pChild !=0; pChild = pChild->NextSibling())
  {
    if(pChild->Type() == TiXmlNode::ELEMENT) {
      if(pChild->ValueStr() == in_childName) { 
        return XMLNodeReader(pChild,m_xmlfilename);
      }
    }
  }
  //return false;
  printMissingRequestedChild(in_childName);
  exit(-1);
}


XMLNodeReader::childiterator 
XMLNodeReader::
children_begin() {
  buildChildVector();
  return m_vec_children.begin();
}

XMLNodeReader::childiterator 
XMLNodeReader::
children_end() {
  buildChildVector();
  return m_vec_children.end();
}



std::string
XMLNodeReader::
stringXMLParameter (const std::string& in_strName,
                                bool in_req,
                                const std::string& in_default,
                                const std::string& in_strDesc) {
  verifyElement();
  m_vec_req_attributes.push_back(in_strName);
  const char* attr_val =  m_pnode->ToElement()->Attribute(in_strName.c_str());
  std::string to_return;
  if(attr_val == NULL) {
    if(in_req == true) {
      printAttrMissing(in_strName, in_strDesc);
      exit(-1);
    } else {
      to_return = in_default;
    }
  } else {
    to_return = std::string(attr_val);
  } 

  return to_return;
}

bool 
XMLNodeReader::
hasXMLParameter(const std::string& in_strName) {
  const char* attr_val =  m_pnode->ToElement()->Attribute(in_strName.c_str());
  if(attr_val == NULL) 
    return false;
  else
    return true;
}

bool 
XMLNodeReader::
boolXMLParameter (const std::string& in_strName, bool in_req,
                        bool in_default, const std::string& in_strDesc) {
  
  // like a string, then to lower, check for == "true" || "false"
  verifyElement();
  m_vec_req_attributes.push_back(in_strName);
  const char* attr_val =  m_pnode->ToElement()->Attribute(in_strName.c_str());
  std::string to_return;
  if(attr_val == NULL) {
    if(in_req == true) {
      printAttrMissing(in_strName, in_strDesc);
      exit(-1);
    } else {
      //to_return = in_default;
      return in_default;
    }
  } else {
    to_return = std::string(attr_val);
  } 
  
  std::transform(to_return.begin(), to_return.end(), to_return.begin(), (int(*)(int)) std::toupper);
  if(to_return == std::string("TRUE")) {
    return true;
  } else if(to_return == std::string("FALSE")) {
    return false;
  } else {
    printAttrWrongType(in_strName,in_strDesc);
  }
 return false; 
}

void 
XMLNodeReader::
warnUnrequestedAttributes() {
  verifyElement();
  std::vector<std::string>::iterator vec_str_itr;
  std::vector<std::string> vec_unreq_attr;
  const TiXmlAttribute* pAttr = m_pnode->ToElement()->FirstAttribute();
  for(; pAttr !=NULL; pAttr = pAttr->Next()) {
    if(hasRequestedAttr(std::string(pAttr->Name())) == false)
      vec_unreq_attr.push_back(std::string(pAttr->Name()));
  }
  if(vec_unreq_attr.size() > 0) {
    std::cout << "*************************************************************" << std::endl;
    std::cout << "XML Warning:: Unrequested Attributes Exist" << std::endl;
    std::cout << "XML file:: " << m_xmlfilename << std::endl;
    std::cout << "Parent Node: " << getName() << std::endl; 
    std::cout << "Unrequested Attributes::" << std::endl;
    for(vec_str_itr = vec_unreq_attr.begin(); vec_str_itr != vec_unreq_attr.end(); ++vec_str_itr)
      std::cout << "          " << *vec_str_itr << std::endl;
    std::cout << *m_pnode << std::endl;
    std::cout << "*************************************************************" << std::endl;
  }
}

void 
XMLNodeReader::
warnUnknownNode() {
  std::cout << "*************************************************************" << std::endl;
  std::cout << "XML Warning:: Unknown Node -- Ignoring" << std::endl;
  std::cout << "XML file:: " << m_xmlfilename << std::endl;
  std::cout << "Node Name: " << getName() << std::endl; 
  std::cout << *m_pnode << std::endl;
  std::cout << "*************************************************************" << std::endl;

}
 

void 
XMLNodeReader::
verifyName(const std::string& in_strName) {
  if(getName() != in_strName) {
    std::cout << "*************************************************************" << std::endl;
    std::cout << "XML ERROR:: Node Name Mismatch" << std::endl;
    std::cout << "XML file:: " << m_xmlfilename << std::endl;
    std::cout << "Node Name: " << getName() << " Requested Name: " << in_strName << std::endl; 
    std::cout << *m_pnode << std::endl;
    std::cout << "*************************************************************" << std::endl;
    exit(-1);
  }
}

void
XMLNodeReader::
verifyElement() const {
  if(!isElement()) {
    std::cerr << "XMLNode::verifyElement() ERROR: Node is not an element." << std::endl;
    std::cerr << *m_pnode << std::endl;
    exit(-1);
  }
}

void 
XMLNodeReader::
printAttrWrongType(const std::string& in_strName, const std::string& in_strDesc) const {
  std::cerr << "*************************************************************" << std::endl;
  std::cerr << "XML Error:: Wrong Type of Attribute requested" << std::endl;
  std::cerr << "XML file:: " << m_xmlfilename << std::endl;
  std::cerr << "Parent Node: " << getName() << ", Attribute Requested: "; 
  std::cerr << in_strName << std::endl;
  std::cerr << "Attribute Description: " << in_strDesc << std::endl;
  std::cerr << *m_pnode << std::endl;
  std::cerr << "*************************************************************" << std::endl;
}

void 
XMLNodeReader::
printAttrMissing(const std::string& in_strName, const std::string& in_strDesc) const {
  std::cerr << "*************************************************************" << std::endl;
  std::cerr << "XML Error:: Missing required Attribute" << std::endl;
  std::cerr << "XML file:: " << m_xmlfilename << std::endl;
  std::cerr << "Parent Node: " << getName() << ", Attribute Missing: "; 
  std::cerr << in_strName << std::endl;
  std::cerr << "Attribute Description: " << in_strDesc << std::endl;
  std::cerr << *m_pnode << std::endl;
  std::cerr << "*************************************************************" << std::endl;
}
 
void
XMLNodeReader::
printMissingRequestedChild(const std::string& in_childName) {
  std::cerr << "*************************************************************" << std::endl;
  std::cerr << "XML Error:: Error Finding Child" << std::endl;
  std::cerr << "XML file:: " << m_xmlfilename << std::endl;
  std::cerr << "Parent Node: " << getName() << " Requested Child: " << in_childName << std::endl;
  std::cerr << *m_pnode << std::endl;
  std::cerr << "*************************************************************" << std::endl;
}

void 
XMLNodeReader::
buildChildVector() {
  if(!m_bChildVecBuilt) {
    for( TiXmlNode* pChild = m_pnode->FirstChild(); pChild !=NULL; pChild = pChild->NextSibling())
    {
      if(pChild->Type() == TiXmlNode::ELEMENT) { 
        m_vec_children.push_back(XMLNodeReader(pChild,m_xmlfilename));
      }
    }
    m_bChildVecBuilt=true;
  }
}

bool 
XMLNodeReader::
hasRequestedAttr(std::string in_strName) {
  std::vector<std::string>::iterator vec_str_itr;
  for(vec_str_itr = m_vec_req_attributes.begin(); 
      vec_str_itr != m_vec_req_attributes.end(); ++vec_str_itr) {
    if(*vec_str_itr == in_strName) {
      return true;
    }
  }
  return false;
}



